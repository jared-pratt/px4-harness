#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PX4 SITL test runner:
- Connects to a PX4 SITL instance via MAVSDK (UDP 14540+instance)
- Executes time-ordered actions from scenarios.json
- Injects failures via PX4 pxh shell (mavlink_shell.py) with verification snapshots
- Streams telemetry to per-scenario CSV for post-mortem analysis
- Supports background "inject_all_failures" while flying patterns (e.g., drawing "IO")
"""
from stakeholders import RegulatorPolicy, VendorSafetyPolicy, OperatorMissionPlanner
import asyncio, json, time, sys, csv, re, math
from pathlib import Path
from typing import Optional
import pexpect
from concurrent.futures import ThreadPoolExecutor
from grid_atc import upload_geofence_rect, grid_offsets, actions_for_grid

from mavsdk import System
from mavsdk.telemetry import FlightMode

# ===================== Helpers & config =====================

def px4_port_for_instance(instance: int) -> int:
    # PX4 SITL default mapping:
    # instance i -> MAVSDK listen on UDP :14540+i
    return 14540 + int(instance)

def now_hms() -> str:
    return time.strftime('%H:%M:%S')

def is_int_like(val: str) -> bool:
    return re.fullmatch(r"-?\d+", str(val)) is not None

# meters N/E -> degrees lat/lon at a reference latitude
def _meters_to_latlon(d_north_m: float, d_east_m: float, ref_lat_deg: float, ref_lon_deg: float):
    dlat = d_north_m / 111_320.0
    dlon = d_east_m / (111_320.0 * math.cos(math.radians(ref_lat_deg)))
    return (ref_lat_deg + dlat, ref_lon_deg + dlon)

async def _get_home(sysobj):
    # Try home first, then fall back to current position
    try:
        async for h in sysobj.telemetry.home():
            return h
    except Exception:
        pass
    try:
        async for p in sysobj.telemetry.position():
            return p
    except Exception:
        pass
    return None


# ===================== Telemetry logging =====================

async def telemetry_logger(
    sysobj: System,
    csv_path: Path,
    stop_evt: asyncio.Event,
    tag: str,
    log=None,
    regulator: RegulatorPolicy = None,
    vendor: VendorSafetyPolicy = None,
):
    """
    Periodically sample telemetry and:

    - Write a CSV row for every sample.
    - Print a compact "TEL ..." status line.
    - Print higher-level EVENT lines when something interesting changes
      (mode transitions, arming, takeoff/landing, major changes in altitude/speed).
    - Call regulator/vendor policies and print any violations/actions.

    This is the main thing you watch over SSH.
    """
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    f = csv_path.open("w", newline="")
    w = csv.writer(f)
    w.writerow([
        "t_s","tag","flight_mode","in_air","armed",
        "battery_pct","voltage_v","lat","lon","abs_alt_m",
        "health_gyro_cal","health_accel_cal","health_mag_cal",
        "health_global_pos","health_local_pos","health_home_pos"
    ])
    t0 = time.monotonic()

    async def get_flight_mode():
        try:
            async for m in sysobj.telemetry.flight_mode():
                return m
        except Exception:
            return None

    async def get_in_air():
        try:
            async for x in sysobj.telemetry.in_air():
                return x
        except Exception:
            return None

    async def get_armed():
        try:
            async for x in sysobj.telemetry.armed():
                return x
        except Exception:
            return None

    async def get_batt():
        try:
            async for b in sysobj.telemetry.battery():
                return b
        except Exception:
            return None

    async def get_pos():
        try:
            async for p in sysobj.telemetry.position():
                return p
        except Exception:
            return None

    async def get_health():
        try:
            async for h in sysobj.telemetry.health():
                return h
        except Exception:
            return None

    async def get_vel():
        try:
            async for v in sysobj.telemetry.velocity_ned():
                return v
        except Exception:
            return None

    # Remember previous state so we only print extra lines when something changes.
    last_mode = None
    last_in_air = None
    last_armed = None
    last_alt_band = None
    last_speed_band = None

    def band(val: float, edges):
        for e in edges:
            if val < e:
                return f"<{e}"
        return f">={edges[-1]}"

    try:
        while not stop_evt.is_set():
            t = time.monotonic() - t0
            fm = await get_flight_mode()
            ia = await get_in_air()
            ar = await get_armed()
            bt = await get_batt()
            ps = await get_pos()
            hl = await get_health()
            vl = await get_vel()

            # Speed magnitude
            speed = 0.0
            if vl is not None:
                speed = math.sqrt(
                    getattr(vl, "north_m_s", 0.0)**2
                    + getattr(vl, "east_m_s", 0.0)**2
                    + getattr(vl, "down_m_s", 0.0)**2
                )

            # --- Battery: normalize to 0–100% ---
            raw_batt = getattr(bt, "remaining_percent", 0.0) if bt else 0.0
            if raw_batt is None:
                batt_pct = 0.0
            elif raw_batt > 1.5:
                # Some builds report directly in percent (e.g. 93.0 for 93%)
                batt_pct = float(raw_batt)
            else:
                # Others use 0.0–1.0
                batt_pct = float(raw_batt) * 100.0

            volt   = getattr(bt, "voltage_v", 0.0) if bt else 0.0
            lat    = getattr(ps, "latitude_deg", 0.0) if ps else 0.0
            lon    = getattr(ps, "longitude_deg", 0.0) if ps else 0.0
            abs_alt = getattr(ps, "absolute_altitude_m", 0.0) if ps else 0.0
            rel_alt = getattr(ps, "relative_altitude_m", 0.0) if ps else 0.0

            # CSV row for offline analysis
            w.writerow([
                f"{t:.2f}", tag,
                str(fm) if fm else "",
                ia if ia is not None else "",
                ar if ar is not None else "",
                f"{batt_pct:.3f}",
                f"{volt:.3f}",
                f"{lat:.7f}",
                f"{lon:.7f}",
                f"{abs_alt:.3f}",
                getattr(hl, 'is_gyrometer_calibration_ok', None) if hl else None,
                getattr(hl, 'is_accelerometer_calibration_ok', None) if hl else None,
                getattr(hl, 'is_magnetometer_calibration_ok', None) if hl else None,
                getattr(hl, 'is_global_position_ok', None) if hl else None,
                getattr(hl, 'is_local_position_ok', None) if hl else None,
                getattr(hl, 'is_home_position_ok', None) if hl else None,
            ])
            f.flush()

            # Main "status line" – this is what you mostly watch over SSH
            if log:
                log(
                    f"TEL t={t:5.1f}s mode={fm} in_air={ia} armed={ar} "
                    f"batt={batt_pct:6.1f}% v={volt:4.2f}V "
                    f"lat={lat:.5f} lon={lon:.5f} rel_alt={rel_alt:4.1f}m "
                    f"speed≈{speed:4.1f}m/s"
                )

            # --- High-level events (easy to scan) ---
            if fm != last_mode:
                log(f"EVENT flight_mode: {last_mode} -> {fm}")
                last_mode = fm

            if ia != last_in_air:
                log(f"EVENT in_air: {last_in_air} -> {ia}")
                last_in_air = ia

            if ar != last_armed:
                log(f"EVENT armed: {last_armed} -> {ar}")
                last_armed = ar

            alt_band = band(rel_alt, [1.0, 5.0, 20.0, 50.0])
            if alt_band != last_alt_band:
                log(f"EVENT altitude_band: {last_alt_band} -> {alt_band} (rel_alt={rel_alt:.1f}m)")
                last_alt_band = alt_band

            speed_band = band(speed, [0.5, 2.0, 5.0, 8.0, 12.0])
            if speed_band != last_speed_band:
                log(f"EVENT speed_band: {last_speed_band} -> {speed_band} (speed≈{speed:.1f}m/s)")
                last_speed_band = speed_band

            # --- Stakeholder policies ---
            if regulator and ps:
                violations = regulator.check_inflight(lat, lon, rel_alt, speed)
                for msg in violations:
                    if log:
                        log(f"!!! REGULATOR: {msg}")
                    else:
                        print("!!! REGULATOR:", msg)

            if vendor:
                # Give vendor a clean 0–100 percentage
                vendor.check_battery(batt_pct, log or print)

            await asyncio.sleep(0.20)
    finally:
        f.close()



# ===================== Persistent pxh shell =====================

class PxhShell:
    """
    Persistent mavlink_shell.py session so pxh commands run quickly and
    don't block the asyncio loop (they execute in a single thread executor).
    """
    def __init__(self, px4_dir: str, instance: int, timeout_s: float = 20.0):
        self.px4_dir = px4_dir
        self.instance = instance
        self.timeout_s = timeout_s
        self._sh: Optional[pexpect.spawn] = None
        self._lock = asyncio.Lock()
        self._executor = ThreadPoolExecutor(max_workers=1)

    def _ensure_sync(self):
        if self._sh is not None and self._sh.isalive():
            return
        tools = Path(self.px4_dir) / "Tools" / "mavlink_shell.py"
        port = px4_port_for_instance(self.instance)
        self._sh = pexpect.spawn(f"/usr/bin/python3 {tools} {port}",
                                 encoding="utf-8", timeout=self.timeout_s)
        self._sh.expect_exact("pxh>")

    def _run_sync(self, command: str) -> str:
        self._ensure_sync()
        self._sh.sendline(command)
        self._sh.expect("pxh>")
        return self._sh.before  # text printed between command and next prompt

    async def run(self, command: str) -> str:
        async with self._lock:
            loop = asyncio.get_running_loop()
            return await loop.run_in_executor(self._executor, self._run_sync, command)

    async def close(self):
        async with self._lock:
            if self._sh is not None:
                try:
                    self._sh.close(force=True)
                except Exception:
                    pass
                self._sh = None
            self._executor.shutdown(wait=False)


def _cmd_to_fname(cmd: str) -> str:
    return (re.sub(r"[^\w.-]+", "_", cmd)[:64] or "pxh_cmd") + ".txt"

async def pxh_run(shell: PxhShell, command: str, log, out_dir: Optional[Path] = None) -> str:
    out = await shell.run(command)
    # Save full output for traceability
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / _cmd_to_fname(command)).write_text(out)
    # Print full output so you "see it in the terminal"
    log(f"$ {command}\n{out}".rstrip())
    return out


# ===================== Failure injection =====================

# Canonical failure space from your list:
FAIL_COMPONENTS = [
    "gyro", "accel", "mag", "baro", "gps",
    "optical_flow", "vio", "distance_sensor", "airspeed",
    "battery", "motor", "servo", "avoidance",
    "rc_signal", "mavlink_signal",
]
FAIL_TYPES = ["off", "stuck", "garbage", "wrong", "slow", "delayed", "intermittent"]
# Use "ok" to clear/reset a component

async def inject_failure_once(shell: PxhShell, component: str, ftype: str, inst: int, log, out_dir: Path) -> bool:
    """
    Inject one failure; return True if PX4 likely accepted it.
    Also captures commander and status snapshots to prove timing.
    """
    cmd = f"failure {component} {ftype} -i {inst}"
    out = await pxh_run(shell, cmd, log, out_dir)
    accepted = True
    for bad in ["unknown", "invalid", "error", "not supported", "usage:"]:
        if bad in out.lower():
            accepted = False
            break
    # Verification snapshots
    await pxh_run(shell, "commander check", log, out_dir)
    await pxh_run(shell, "listener vehicle_status -n 1", log, out_dir)
    return accepted

async def clear_failure(shell: PxhShell, component: str, inst: int, log, out_dir: Path):
    await pxh_run(shell, f"failure {component} ok -i {inst}", log, out_dir)


# ===================== MAVSDK actions =====================

async def connect_system(instance: int, log) -> System:
    """
    Connect to PX4 SITL for the given instance.

    We use the recommended 'udpin://0.0.0.0:PORT' syntax so you don't see
    the 'Connection using udp:// is deprecated' warning anymore.
    """
    sysobj = System()
    port = px4_port_for_instance(instance)
    await sysobj.connect(system_address=f"udpin://0.0.0.0:{port}")

    # Wait briefly for a heartbeat
    t_deadline = time.time() + 4.0
    got = False
    try:
        async for _ in sysobj.telemetry.health():
            got = True
            break
    except Exception:
        pass
    if not got and time.time() > t_deadline:
        log("warning: no heartbeat yet (continuing)")
    return sysobj





async def do_action(sysobj: System, action: dict, px4_dir: str, instance: int, log, scen_dir: Path, shell: PxhShell):
    cmd = action.get("cmd")
    if not cmd:
        log("action missing 'cmd'")
        return

    if cmd == "wait":
        dt = float(action.get("sec", 1.0))
        await asyncio.sleep(dt)
        log(f"waited {dt}s")

    elif cmd == "arm":
        await sysobj.action.arm()
        log("armed")

    elif cmd == "takeoff":
        alt = float(action.get("alt", 2.5))
        await sysobj.action.set_takeoff_altitude(alt)
        await sysobj.action.takeoff()
        log(f"takeoff to {alt} m")

    elif cmd == "hold":
        await sysobj.action.hold()
        log("hold")

    elif cmd == "land":
        await sysobj.action.land()
        log("land")

    elif cmd == "param_set":
        name = action["name"]
        value = action["value"]
        if isinstance(value, int) or is_int_like(value):
            await sysobj.param.set_param_int(name, int(value))
        else:
            await sysobj.param.set_param_float(name, float(value))
        log(f"param set {name} -> {value}")

    elif cmd == "pxh":
        # run arbitrary pxh command (supports your whole command list)
        pxh_cmd = action["command"]
        await pxh_run(shell, pxh_cmd, log, scen_dir / "pxh")

    elif cmd == "set_speed":
        spd = float(action["mps"])
        try:
            setter = getattr(sysobj.action, "set_maximum_speed", None)
            if setter is not None:
                await setter(spd)
                log(f"set speed {spd} m/s via MAVSDK Action.set_maximum_speed()")
            else:
                # Fallback: use PX4 position controller params (no MIS_SPEED)
                await sysobj.param.set_param_float("MPC_XY_CRUISE", spd)
                await sysobj.param.set_param_float("MPC_XY_VEL_MAX", spd)
                log(f"set speed {spd} m/s via MPC_XY_CRUISE/MPC_XY_VEL_MAX params")
        except Exception as e:
            log(f"set_speed ignored due to error: {e}")

    elif cmd == "goto_offset":
        home = await _get_home(sysobj)
        if not home:
            log("goto_offset: no home/pos yet")
            return
        lat, lon = _meters_to_latlon(float(action["north_m"]), float(action["east_m"]),
                                     home.latitude_deg, home.longitude_deg)
        if "rel_alt_m" in action:
            alt = float(home.absolute_altitude_m) + float(action["rel_alt_m"])
        else:
            alt = float(action.get("alt", float(home.absolute_altitude_m)))
        yaw = float(action.get("yaw", 0.0))
        await sysobj.action.goto_location(lat, lon, alt, yaw)
        log(f"goto_offset N{action['north_m']} E{action['east_m']} -> {lat:.7f},{lon:.7f} alt {alt:.2f} AMSL")

    elif cmd == "goto_abs":
        lat = float(action["lat"]); lon = float(action["lon"])
        home = await _get_home(sysobj)
        if "rel_alt_m" in action and home:
            alt_amsl = home.absolute_altitude_m + float(action["rel_alt_m"])
            source = "rel_alt_m"
        else:
            alt_amsl = float(action.get("alt", 30.0))
            source = "alt(AMSL)"
        yaw = float(action.get("yaw", 0.0))
        await sysobj.action.goto_location(lat, lon, alt_amsl, yaw)
        log(f"goto_abs {lat:.7f},{lon:.7f} alt {alt_amsl:.1f} ({source})")

    elif cmd == "inject_all_failures":
        # Run a background campaign of (component × type) injections.
        start_delay = float(action.get("start_after_s", 10.0))
        interval = float(action.get("interval_s", 0.8))
        clear = bool(action.get("clear_after_each", True))
        inst = int(action.get("instance_index", 0))
        outdir_root = scen_dir / "failures"

        async def _bg():
            await asyncio.sleep(start_delay)
            log(f"BEGIN injecting all failures (inst={inst}, dt={interval}s, clear={clear})")
            for comp in FAIL_COMPONENTS:
                for ftype in FAIL_TYPES:
                    outdir = outdir_root / comp / ftype
                    ok = await inject_failure_once(shell, comp, ftype, inst, log, outdir)
                    if clear:
                        await clear_failure(shell, comp, inst, log, outdir_root / comp / "ok")
                    await asyncio.sleep(interval)
            log("DONE injecting all failures")
        asyncio.create_task(_bg())

    elif cmd == "geofence_rect":
        # Upload a rectangular inclusion geofence centered at HOME
        width_m  = float(action.get("width_m", 400.0))
        height_m = float(action.get("height_m", 400.0))

        home = await _get_home(sysobj)
        if not home:
            log("geofence_rect: no home/pos yet; skipping")
        else:
            await upload_geofence_rect(
                sysobj,
                center_lat=home.latitude_deg,
                center_lon=home.longitude_deg,
                width_m=width_m,
                height_m=height_m,
        )
        log(f"geofence uploaded {width_m}x{height_m} m around home")


    else:
        log(f"UNKNOWN action {cmd}")


# ===================== Scenario engine =====================

async def run_scenario(px4_dir: str, scenario: dict, out_root: Path):
    name     = scenario["name"]
    instance = int(scenario.get("instance", 1))
    actions  = sorted(scenario.get("actions", []), key=lambda a: float(a.get("t", 0.0)))

    # Expand any 'append_grid' macros into concrete goto_offset actions
    expanded = []
    for a in actions:
        if a.get("cmd") == "append_grid":
            nx = int(a.get("nx", 5))
            ny = int(a.get("ny", 5))
            spacing = float(a.get("spacing_m", 25.0))
            alt = float(a.get("alt", 46.0))
            start_t = float(a.get("start_t", 6.0))
            step_t = float(a.get("step_t", 4.0))

            pts = grid_offsets(nx, ny, spacing)
            grid_acts = actions_for_grid(pts, cruise_alt_m=alt, start_t=start_t, step_t=step_t)
            expanded.extend(grid_acts)
            # Optional breadcrumb for your logs:
            expanded.append({"t": max(0.0, start_t - 0.001), "cmd": "pxh", "command": f"echo 'Starting {nx}x{ny} grid @ {spacing} m'"})
        else:
            expanded.append(a)

    actions = sorted(expanded, key=lambda a: float(a.get("t", 0.0)))

    # I/O paths
    scen_dir = out_root / f"{name}_i{instance}"
    scen_dir.mkdir(parents=True, exist_ok=True)
    log_path = scen_dir / "run.log"
    csv_path = scen_dir / "telemetry.csv"

    # logger
    logf = log_path.open("a")
    def log(msg: str):
        line = f"[{now_hms()}] {name}/i{instance}: {msg}"
        print(line, flush=True)
        logf.write(line + "\n"); logf.flush()

    log("connecting...")
    sysobj = await connect_system(instance, log)
    shell = PxhShell(px4_dir, instance)
    log("connected")

    # --- Stakeholder instances for this scenario ---
    # Regulator: 120 m ceiling, 10 m/s, geofence around KSQL core
    #regulator = RegulatorPolicy(max_altitude_m=120.0, max_speed_m_s=10.0)
    #regulator.add_geofence("KSQL core", center=(37.5120, -122.2500), radius_m=1000.0)

    regulator = RegulatorPolicy(max_altitude_m=120.0, max_speed_m_s=10.0)
    # Optional: add a logical no-fly zone somewhere near Baylands later
    # regulator.add_geofence("No-fly demo", center=(37.4128, -121.9998), radius_m=200.0)


    # Vendor: watch battery; wire drone object so you can RTL later
    vendor = VendorSafetyPolicy(drone=sysobj)

    # start telemetry CSV + policy monitoring task
    stop_evt = asyncio.Event()
    tel_task = asyncio.create_task(
        telemetry_logger(
            sysobj,
            csv_path,
            stop_evt,
            tag=name,
            log=log,
            regulator=regulator,
            vendor=vendor,
        )
    )
    # time-stepped actions
    t0 = time.monotonic()
    try:
        idx = 0
        while idx < len(actions):
            now = time.monotonic() - t0
            next_t = float(actions[idx].get("t", 0.0))
            if now < next_t:
                await asyncio.sleep(min(0.1, next_t - now))
                continue
            await do_action(sysobj, actions[idx], px4_dir, instance, log, scen_dir, shell)
            idx += 1

        # Optional expectations
        expect = scenario.get("expect", {})
        if "final_mode" in expect:
            last_mode = None
            try:
                async for fm in sysobj.telemetry.flight_mode():
                    last_mode = fm
                    break
            except Exception:
                pass
            want = expect["final_mode"].upper()
            ok = (last_mode == getattr(FlightMode, want, None))
            log(f"EXPECT final_mode={want} -> {ok} (got {last_mode})")

    finally:
        stop_evt.set()
        try:
            await asyncio.wait_for(tel_task, timeout=2.0)
        except Exception:
            tel_task.cancel()
        await shell.close()
        logf.close()


# ===================== Main =====================

async def main():
    if len(sys.argv) < 3:
        print("Usage: runner.py <PX4_DIR> <scenarios.json> [outdir]")
        sys.exit(1)

    px4_dir   = sys.argv[1]
    scen_path = Path(sys.argv[2])
    out_root  = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("runs_out")

    scenarios = json.loads(scen_path.read_text())

    tasks = []
    for s in scenarios:
        tasks.append(run_scenario(px4_dir, s, out_root))

    results = await asyncio.gather(*tasks, return_exceptions=True)
    for s, res in zip(scenarios, results):
        if isinstance(res, Exception):
            print(f"[{now_hms()}] Scenario {s.get('name')} failed: {res}", file=sys.stderr)

if __name__ == "__main__":
    asyncio.run(main())
