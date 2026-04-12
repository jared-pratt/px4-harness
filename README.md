# PX4 Harness — Multi-Drone SITL Research Platform

Headless PX4 SITL simulation with a custom FastAPI dashboard, a MAVLink WebSocket bridge for ADOS Mission Control, and optional MAVROS/Aerostack2 integration.

---

## Repository Layout

```
px4-harness/
├── scripts/
│   ├── start_sitl.sh            # One-command PX4 launcher (handles cleanup + launch)
│   ├── mavlink_ws_bridge.py     # MAVLink UDP ↔ WebSocket bridge for ADOS
│   ├── test_px4_udp.py          # Diagnostic: verify PX4 UDP telemetry is flowing
│   └── launch_aerostack_mavros.sh
├── dashboard/                   # FastAPI + WebSocket real-time dashboard
│   ├── server.py
│   ├── requirements.txt
│   └── static/                  # index.html, style.css, app.js
├── aerostack2-integration/      # ROS2 launch files, MAVROS config
├── headless_sim/                # Installable headless CLI package
├── runner.py                    # Core MAVSDK scenario runner
├── scenarios.json               # Mission scenario definitions
└── requirements.txt             # Python deps (mavsdk, pexpect)
```

---

## Prerequisites

| Requirement | Version | Notes |
|-------------|---------|-------|
| PX4-Autopilot | any recent | Built from source; expected at `~/ResearchFall2025/Simulators/PX4-Autopilot` |
| Python | 3.10+ | For bridge and dashboard |
| Node.js + npm | 18+ | For ADOS Mission Control only |
| Chrome / Edge | 89+ | Required for WebSocket + WebSerial in ADOS |
| ROS 2 Humble | optional | For MAVROS/Aerostack2 only |

Install Python dependencies:

```bash
pip install -r requirements.txt
pip install -r dashboard/requirements.txt
pip install websockets pymavlink   # for the MAVLink bridge
```

---

## Port Reference

| Drone | PX4 instance | PX4 GCS UDP | Bridge local UDP | WebSocket URL |
|-------|-------------|-------------|-----------------|---------------|
| 1 | 0 | 18570 | 14550 | ws://localhost:5760 |
| 2 | 1 | 18571 | 14551 | ws://localhost:5761 |
| 3 | 2 | 18572 | 14552 | ws://localhost:5762 |
| N | N-1 | 18570+(N-1) | 14550+(N-1) | ws://localhost:5760+(N-1) |

PX4 also exposes MAVSDK/offboard API ports at `14540+instance` (e.g. 14540, 14541).

---

## Quick Start

### Step 1 — Start PX4 SITL

Use the launch script (handles killing stale processes and clearing lock files automatically):

```bash
cd ~/ResearchFall2025/Simulators/px4-harness
bash scripts/start_sitl.sh [LAT] [LON] [ALT_MSL] [NUM_DRONES]
```

Defaults (your saved home location, 2 drones):

```bash
bash scripts/start_sitl.sh
```

Custom location or drone count:

```bash
bash scripts/start_sitl.sh 37.7749 -122.4194 16 3   # San Francisco, 3 drones
```

Wait for each instance to print:

```
INFO  [commander] Ready for takeoff!
```

> **If PX4 fails with "server already running":** stale lock files are present.
> The script clears them automatically, but if you launched PX4 manually, run:
> ```bash
> pkill -9 -f px4; rm -f /tmp/px4_lock-* /tmp/px4-sock-*; sleep 1
> ```

### Step 2 — Start the MAVLink WebSocket Bridge

Bridges PX4's UDP telemetry to WebSocket so ADOS can connect from a browser.

```bash
cd ~/ResearchFall2025/Simulators/px4-harness
python scripts/mavlink_ws_bridge.py
```

You should see `← UDP #1` lines within a second or two, confirming telemetry is flowing from PX4 to the bridge. If you only see outgoing heartbeats and no incoming UDP, PX4 is not ready yet — wait and retry.

### Step 3 — Start ADOS Mission Control

```bash
cd ~/ResearchFall2025/Simulators/ados-mission-control
npm install        # first time only
npm run dev
```

Open **http://localhost:4000** in Chrome or Edge.

> **Remote machine?** Forward the ports over SSH from your local machine:
> ```bash
> ssh -L 4000:localhost:4000 -L 5760:localhost:5760 -L 5761:localhost:5761 user@remote-host
> ```
> All three ports must be forwarded: 4000 (ADOS UI) and one per drone (5760, 5761, …).

### Step 4 — Connect ADOS to Each Drone

For each drone:

1. Click **Connect** in the ADOS top bar
2. Select **WebSocket**
3. Enter the URL for that drone (`ws://localhost:5760` for drone 1, `ws://localhost:5761` for drone 2, etc.)
4. Click **Connect**

Telemetry and the map should appear within a few seconds.

---

## Custom Dashboard (FastAPI)

A lightweight alternative to ADOS — real-time telemetry cards, flight path map, charts, scenario editor, and failure injection.

```bash
cd ~/ResearchFall2025/Simulators/px4-harness
PX4_DIR=~/ResearchFall2025/Simulators/PX4-Autopilot \
  uvicorn dashboard.server:app --host 0.0.0.0 --port 8080
```

Open **http://localhost:8080**.

> Run from the `px4-harness/` directory, not from inside `dashboard/` — the server imports `runner` from the parent package.

### Dashboard Features

| Panel | Description |
|-------|-------------|
| Drone cards | Live mode, arm state, altitude, speed, battery, GPS |
| Flight path map | Leaflet map with per-drone trails |
| Live telemetry | Altitude / speed / battery charts |
| Flight log analysis | Load any `runs_out/` CSV and graph it |
| Scenario editor | Edit `scenarios.json` in-browser and run missions |
| Failure injection | Inject/clear PX4 sensor failures via pxh shell |
| Mission control | Trigger named scenarios directly |
| Console log | Rolling 300-line server log |

---

## Adding More Drones

1. **Bridge** — add an entry to `BRIDGES` in `scripts/mavlink_ws_bridge.py`:
   ```python
   {"label": "drone4", "local_port": 14553, "px4_port": 18573, "ws_port": 5763},
   ```
2. **Launch** — pass the count to the start script:
   ```bash
   bash scripts/start_sitl.sh 40.769028 -111.846333 1300 4
   ```
3. **ADOS** — connect to `ws://localhost:5763`.

---

## MAVROS / Aerostack2 (Optional)

```bash
cd px4-harness
bash scripts/launch_aerostack_mavros.sh
```

Cleans up stale ROS/DDS processes and launches two namespaced MAVROS nodes with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=42`.

MAVROS connects to the PX4 offboard API ports (14540, 14541) — separate from the GCS ports used by the bridge and ADOS.

---

## Diagnostics

**Verify UDP telemetry is flowing from PX4 (before starting bridge):**

```bash
python scripts/test_px4_udp.py
```

Should print `← Got N bytes from ('127.0.0.1', 18570)` within a few seconds.

**Check what's bound to the GCS ports:**

```bash
ss -ulnp | grep '1455'
```

If anything other than the bridge (`python`) owns port 14550/14551, it will steal PX4's telemetry.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `PX4 server already running for instance N` | Run `pkill -9 -f px4; rm -f /tmp/px4_lock-* /tmp/px4-sock-*` |
| Bridge shows no `← UDP` lines | PX4 not ready yet, or wrong instance; wait for "Ready for takeoff!" |
| ADOS connects but no telemetry | Check bridge for `← UDP` lines; restart bridge after PX4 is up |
| `Address already in use` on port 14550 | Another GCS (QGroundControl) is running — close it |
| ADOS page unreachable | If on remote machine, ensure all ports are SSH-forwarded (4000 + 576x) |
| Dashboard `import runner` error | Run `uvicorn` from `px4-harness/` directory, not from `dashboard/` |
