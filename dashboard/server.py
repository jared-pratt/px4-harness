"""
UAV Simulation Dashboard — FastAPI server

Connects to two PX4 SITL drones via MAVSDK, streams real-time telemetry
over WebSocket, and exposes REST endpoints for drone control and failure
injection.

Run from the px4-harness directory:
    uvicorn dashboard.server:app --host 0.0.0.0 --port 8080

With a custom PX4 directory:
    PX4_DIR=~/ResearchFall2025/Simulators/PX4-Autopilot uvicorn dashboard.server:app --host 0.0.0.0 --port 8080
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Set

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# ---------------------------------------------------------------------------
# Import runner.py from the parent directory (px4-harness root)
# ---------------------------------------------------------------------------
_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(_ROOT))
import runner  # noqa: E402

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Path to PX4-Autopilot checkout (used by PxhShell for mavlink_shell.py)
PX4_DIR = os.environ.get(
    "PX4_DIR",
    str(Path.home() / "ResearchFall2025" / "Simulators" / "PX4-Autopilot"),
)
SCENARIOS_FILE = _ROOT / "scenarios.json"

# Map dashboard drone ID → MAVSDK instance index (0-based)
# Instance 0 → port 14540 (drone1), Instance 1 → port 14541 (drone2)
DRONE_INSTANCES: Dict[int, int] = {1: 0, 2: 1}

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

_systems: Dict[int, Any] = {}                  # drone_id → mavsdk.System
_shells: Dict[int, runner.PxhShell] = {}        # drone_id → PxhShell
_telemetry: Dict[int, Dict] = {
    did: {"drone_id": did, "connected": False, "flight_mode": "—"}
    for did in DRONE_INSTANCES
}
_log_ring: deque = deque(maxlen=300)            # rolling log buffer
_ws_clients: Set[WebSocket] = set()
_history: Dict[int, deque] = {              # per-drone position history (last 500 pts)
    did: deque(maxlen=500) for did in DRONE_INSTANCES
}

# ---------------------------------------------------------------------------
# Logging helper
# ---------------------------------------------------------------------------

def _log(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    entry = f"[{ts}] {msg}"
    _log_ring.append(entry)
    print(entry, flush=True)


def _drone_logger(drone_id: int):
    """Return a log function that prefixes with drone id."""
    def _fn(msg: str) -> None:
        _log(f"drone{drone_id}: {msg}")
    return _fn


# ---------------------------------------------------------------------------
# MAVSDK helpers
# ---------------------------------------------------------------------------

async def _one(stream) -> Any:
    """Pull the first value from an MAVSDK async-generator stream."""
    try:
        async for v in stream:
            return v
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Background: per-drone telemetry polling
# ---------------------------------------------------------------------------

async def _poll_drone(drone_id: int) -> None:
    sysobj = _systems[drone_id]
    while True:
        try:
            fm = await _one(sysobj.telemetry.flight_mode())
            ia = await _one(sysobj.telemetry.in_air())
            ar = await _one(sysobj.telemetry.armed())
            bt = await _one(sysobj.telemetry.battery())
            ps = await _one(sysobj.telemetry.position())
            hl = await _one(sysobj.telemetry.health())
            vl = await _one(sysobj.telemetry.velocity_ned())

            raw_batt = getattr(bt, "remaining_percent", 0.0) or 0.0
            batt_pct = float(raw_batt) if raw_batt > 1.5 else float(raw_batt) * 100.0

            speed = 0.0
            if vl:
                speed = math.sqrt(
                    getattr(vl, "north_m_s", 0.0) ** 2
                    + getattr(vl, "east_m_s", 0.0) ** 2
                    + getattr(vl, "down_m_s", 0.0) ** 2
                )

            # Parse mode string: "FlightMode.HOLD" → "HOLD"
            mode_str = str(fm).split(".")[-1] if fm else "UNKNOWN"

            _telemetry[drone_id] = {
                "drone_id": drone_id,
                "connected": True,
                "flight_mode": mode_str,
                "in_air": ia,
                "armed": ar,
                "battery_pct": round(batt_pct, 1),
                "voltage_v": round(getattr(bt, "voltage_v", 0.0) or 0.0, 2),
                "lat": round(getattr(ps, "latitude_deg", 0.0) or 0.0, 6),
                "lon": round(getattr(ps, "longitude_deg", 0.0) or 0.0, 6),
                "abs_alt_m": round(getattr(ps, "absolute_altitude_m", 0.0) or 0.0, 1),
                "rel_alt_m": round(getattr(ps, "relative_altitude_m", 0.0) or 0.0, 1),
                "speed_m_s": round(speed, 1),
                "gyro_ok": getattr(hl, "is_gyrometer_calibration_ok", None) if hl else None,
                "accel_ok": getattr(hl, "is_accelerometer_calibration_ok", None) if hl else None,
                "mag_ok": getattr(hl, "is_magnetometer_calibration_ok", None) if hl else None,
                "gps_ok": getattr(hl, "is_global_position_ok", None) if hl else None,
                "home_ok": getattr(hl, "is_home_position_ok", None) if hl else None,
                "ts": time.time(),
            }
            # Append to position history (only when we have a real fix)
            lat = _telemetry[drone_id]["lat"]
            lon = _telemetry[drone_id]["lon"]
            if lat != 0.0 or lon != 0.0:
                _history[drone_id].append({
                    "ts": _telemetry[drone_id]["ts"],
                    "lat": lat,
                    "lon": lon,
                    "rel_alt_m": _telemetry[drone_id]["rel_alt_m"],
                    "speed_m_s": _telemetry[drone_id]["speed_m_s"],
                    "battery_pct": _telemetry[drone_id]["battery_pct"],
                    "flight_mode": _telemetry[drone_id]["flight_mode"],
                    "armed": _telemetry[drone_id]["armed"],
                })
        except Exception as exc:
            _telemetry[drone_id]["connected"] = False
            _telemetry[drone_id]["error"] = str(exc)

        await asyncio.sleep(0.25)


# ---------------------------------------------------------------------------
# Background: WebSocket broadcast
# ---------------------------------------------------------------------------

async def _broadcast_loop() -> None:
    global _ws_clients
    while True:
        if _ws_clients:
            msg = json.dumps({
                "type": "update",
                "drones": _telemetry,
                "logs": list(_log_ring)[-60:],
            })
            dead: Set[WebSocket] = set()
            for ws in list(_ws_clients):
                try:
                    await ws.send_text(msg)
                except Exception:
                    dead.add(ws)
            _ws_clients -= dead
        await asyncio.sleep(0.25)


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------

app = FastAPI(title="UAV Simulation Dashboard")
_STATIC = Path(__file__).parent / "static"


@app.on_event("startup")
async def _startup() -> None:
    for drone_id, mavsdk_inst in DRONE_INSTANCES.items():
        try:
            _log(f"Connecting to drone {drone_id} (MAVSDK instance {mavsdk_inst}, port {14540 + mavsdk_inst})…")
            sysobj = await runner.connect_system(mavsdk_inst, _drone_logger(drone_id))
            _systems[drone_id] = sysobj
            _shells[drone_id] = runner.PxhShell(PX4_DIR, mavsdk_inst)
            _telemetry[drone_id]["connected"] = True
            _log(f"Connected to drone {drone_id}")
        except Exception as exc:
            _log(f"Warning: could not connect to drone {drone_id}: {exc}")
            _telemetry[drone_id].update({"connected": False, "error": str(exc)})

    asyncio.create_task(_broadcast_loop())
    for drone_id in _systems:
        asyncio.create_task(_poll_drone(drone_id))


# ---------------------------------------------------------------------------
# Static files & root
# ---------------------------------------------------------------------------

@app.get("/", response_class=FileResponse)
async def index() -> FileResponse:
    return FileResponse(_STATIC / "index.html")


app.mount("/static", StaticFiles(directory=str(_STATIC)), name="static")


# ---------------------------------------------------------------------------
# WebSocket
# ---------------------------------------------------------------------------

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    _ws_clients.add(ws)
    try:
        # Push current state immediately on connect
        await ws.send_text(json.dumps({
            "type": "update",
            "drones": _telemetry,
            "logs": list(_log_ring),
        }))
        while True:
            await ws.receive_text()   # keep-alive; client sends pings
    except WebSocketDisconnect:
        pass
    finally:
        _ws_clients.discard(ws)


# ---------------------------------------------------------------------------
# Drone action endpoint
# ---------------------------------------------------------------------------

class ActionRequest(BaseModel):
    action: str
    params: Dict[str, Any] = {}


@app.post("/api/drone/{drone_id}/action")
async def drone_action(drone_id: int, req: ActionRequest) -> Dict:
    if drone_id not in _systems:
        raise HTTPException(404, f"Drone {drone_id} not connected")

    sysobj = _systems[drone_id]
    act = req.action.lower()

    try:
        if act == "arm":
            await sysobj.action.arm()
            _log(f"drone{drone_id}: armed")

        elif act == "disarm":
            await sysobj.action.disarm()
            _log(f"drone{drone_id}: disarmed")

        elif act == "takeoff":
            alt = float(req.params.get("alt", 5.0))
            await sysobj.action.set_takeoff_altitude(alt)
            await sysobj.action.takeoff()
            _log(f"drone{drone_id}: takeoff → {alt} m")

        elif act == "land":
            await sysobj.action.land()
            _log(f"drone{drone_id}: land")

        elif act == "hold":
            await sysobj.action.hold()
            _log(f"drone{drone_id}: hold")

        elif act == "return_to_launch":
            await sysobj.action.return_to_launch()
            _log(f"drone{drone_id}: return to launch")

        elif act == "goto_offset":
            north = float(req.params.get("north_m", 0.0))
            east  = float(req.params.get("east_m", 0.0))
            alt   = float(req.params.get("rel_alt_m", 10.0))
            home  = await runner._get_home(sysobj)
            if not home:
                raise HTTPException(400, "No home position available")
            lat, lon = runner._meters_to_latlon(
                north, east, home.latitude_deg, home.longitude_deg
            )
            alt_amsl = home.absolute_altitude_m + alt
            await sysobj.action.goto_location(lat, lon, alt_amsl, 0.0)
            _log(f"drone{drone_id}: goto N{north:+.1f}m E{east:+.1f}m alt{alt:.1f}m")

        else:
            raise HTTPException(400, f"Unknown action: {act}")

    except HTTPException:
        raise
    except Exception as exc:
        _log(f"drone{drone_id}: action '{act}' error: {exc}")
        raise HTTPException(500, str(exc))

    return {"status": "ok", "action": act}


# ---------------------------------------------------------------------------
# Failure injection endpoints
# ---------------------------------------------------------------------------

class FailureRequest(BaseModel):
    component: str
    ftype: str
    instance_idx: int = 0


@app.post("/api/drone/{drone_id}/inject")
async def inject_failure(drone_id: int, req: FailureRequest) -> Dict:
    if drone_id not in _shells:
        raise HTTPException(404, f"No pxh shell for drone {drone_id} (not connected?)")

    logs: List[str] = []
    log_fn = _make_capture_logger(drone_id, logs)

    try:
        accepted = await runner.inject_failure_once(
            _shells[drone_id], req.component, req.ftype, req.instance_idx,
            log_fn, None,
        )
    except Exception as exc:
        raise HTTPException(500, str(exc))

    return {"accepted": accepted, "component": req.component, "ftype": req.ftype, "logs": logs}


@app.post("/api/drone/{drone_id}/clear")
async def clear_failure_endpoint(drone_id: int, req: FailureRequest) -> Dict:
    if drone_id not in _shells:
        raise HTTPException(404, f"No pxh shell for drone {drone_id} (not connected?)")

    logs: List[str] = []
    log_fn = _make_capture_logger(drone_id, logs)

    try:
        await runner.clear_failure(
            _shells[drone_id], req.component, req.instance_idx, log_fn, None,
        )
    except Exception as exc:
        raise HTTPException(500, str(exc))

    return {"status": "cleared", "component": req.component, "logs": logs}


def _make_capture_logger(drone_id: int, logs: List[str]):
    """Return a sync log function that both captures to list and logs globally."""
    def _fn(msg: str) -> None:
        logs.append(msg)
        _log(f"drone{drone_id}: {msg}")
    return _fn


@app.get("/api/failures/options")
async def failure_options() -> Dict:
    return {
        "components": runner.FAIL_COMPONENTS,
        "types": runner.FAIL_TYPES,
    }


# ---------------------------------------------------------------------------
# Scenario endpoints
# ---------------------------------------------------------------------------

@app.get("/api/scenarios")
async def get_scenarios() -> Dict:
    if not SCENARIOS_FILE.exists():
        return {"scenarios": []}
    return {"scenarios": json.loads(SCENARIOS_FILE.read_text())}


class ScenarioRunRequest(BaseModel):
    scenario_name: str


@app.post("/api/scenario/run")
async def run_scenario_endpoint(req: ScenarioRunRequest) -> Dict:
    if not SCENARIOS_FILE.exists():
        raise HTTPException(404, "scenarios.json not found")

    scenarios = json.loads(SCENARIOS_FILE.read_text())
    scenario = next((s for s in scenarios if s["name"] == req.scenario_name), None)
    if scenario is None:
        raise HTTPException(404, f"Scenario '{req.scenario_name}' not found")

    out_root = _ROOT / "runs_out"
    _log(f"Starting scenario: {req.scenario_name}")
    asyncio.create_task(runner.run_scenario(PX4_DIR, scenario, out_root))
    return {"status": "started", "scenario": req.scenario_name}


# ---------------------------------------------------------------------------
# Telemetry snapshot (polling fallback for clients that don't use WebSocket)
# ---------------------------------------------------------------------------

@app.get("/api/telemetry")
async def get_telemetry() -> Dict:
    return {"drones": _telemetry}


# ---------------------------------------------------------------------------
# Position history (for map trail on initial load)
# ---------------------------------------------------------------------------

@app.get("/api/history/{drone_id}")
async def get_history(drone_id: int) -> Dict:
    if drone_id not in _history:
        raise HTTPException(404, "Drone not found")
    return {"drone_id": drone_id, "history": list(_history[drone_id])}


# ---------------------------------------------------------------------------
# Past run log analysis
# ---------------------------------------------------------------------------

@app.get("/api/runs")
async def list_runs() -> Dict:
    """List all run directories that contain a telemetry.csv."""
    import csv as _csv
    out_root = _ROOT / "runs_out"
    if not out_root.exists():
        return {"runs": []}
    runs = []
    for d in sorted(out_root.iterdir(), reverse=True):
        if not d.is_dir():
            continue
        csv_file = d / "telemetry.csv"
        if csv_file.exists():
            # Count rows for a quick summary
            try:
                with csv_file.open() as f:
                    row_count = sum(1 for _ in f) - 1  # subtract header
            except Exception:
                row_count = 0
            runs.append({"name": d.name, "rows": row_count})
    return {"runs": runs}


@app.get("/api/runs/{run_name}/telemetry")
async def get_run_telemetry(run_name: str) -> Dict:
    """Parse a run's telemetry.csv and return structured data for graphing."""
    import csv as _csv
    # Sanitize run_name to prevent path traversal
    if ".." in run_name or "/" in run_name:
        raise HTTPException(400, "Invalid run name")
    csv_file = _ROOT / "runs_out" / run_name / "telemetry.csv"
    if not csv_file.exists():
        raise HTTPException(404, f"Run '{run_name}' not found")

    rows = []
    mode_changes = []
    prev_mode = None

    with csv_file.open() as f:
        reader = _csv.DictReader(f)
        for row in reader:
            try:
                t = float(row.get("t_s", 0))
                mode = row.get("flight_mode", "")
                rows.append({
                    "t_s":         t,
                    "flight_mode": mode,
                    "battery_pct": float(row.get("battery_pct") or 0),
                    "lat":         float(row.get("lat") or 0),
                    "lon":         float(row.get("lon") or 0),
                    "abs_alt_m":   float(row.get("abs_alt_m") or 0),
                    "in_air":      row.get("in_air", ""),
                    "armed":       row.get("armed", ""),
                })
                if mode != prev_mode:
                    mode_changes.append({"t_s": t, "mode": mode})
                    prev_mode = mode
            except (ValueError, KeyError):
                continue

    return {"run": run_name, "rows": rows, "mode_changes": mode_changes}


# ---------------------------------------------------------------------------
# Scenario editor — save updated scenarios.json
# ---------------------------------------------------------------------------

class ScenarioSaveRequest(BaseModel):
    content: str   # raw JSON string from the editor textarea


@app.post("/api/scenarios/save")
async def save_scenarios(req: ScenarioSaveRequest) -> Dict:
    try:
        parsed = json.loads(req.content)
    except json.JSONDecodeError as exc:
        raise HTTPException(400, f"Invalid JSON: {exc}")
    if not isinstance(parsed, list):
        raise HTTPException(400, "Scenarios must be a JSON array")
    SCENARIOS_FILE.write_text(json.dumps(parsed, indent=2))
    _log(f"Scenarios saved ({len(parsed)} scenario(s))")
    return {"status": "saved", "count": len(parsed)}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    uvicorn.run("dashboard.server:app", host="0.0.0.0", port=8080, reload=False)
