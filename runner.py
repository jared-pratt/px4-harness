#!/usr/bin/env python3
import asyncio, json, time, sys
from pathlib import Path
import pexpect
from mavsdk import System
from mavsdk.telemetry import FlightMode

# --- Port mapping helper ---
def px4_port_for_instance(instance: int) -> int:
    # PX4 SITL sends MAVLink to 127.0.0.1:14540+instance (seen in logs).
    return 14540 + int(instance)

# --- PX4 shell via mavlink_shell.py ---
def px4_shell(px4_dir: str, instance: int) -> pexpect.spawn:
    tools = Path(px4_dir) / "Tools" / "mavlink_shell.py"
    port = px4_port_for_instance(instance)
    sh = pexpect.spawn(f"python3 {tools} -u {port}", encoding="utf-8", timeout=15)
    sh.expect_exact("pxh>")
    return sh

# --- Fault injection mapping (from `failure -h`) ---
FAIL_MAP = {
    "imu_off":     "failure imu off",
    "gyro_off":    "failure gyro off",
    "accel_off":   "failure accel off",
    "mag_off":     "failure mag off",
    "baro_off":    "failure baro off",
    "gps_off":     "failure gps off",
    "battery_low": "failure battery low",
}

async def connect_system(instance: int) -> System:
    sysobj = System()
    port = px4_port_for_instance(instance)
    await sysobj.connect(system_address=f"udp://:{port}")
    # Wait for heartbeat briefly
    start = time.time()
    async for _ in sysobj.telemetry.health():
        if time.time() - start > 2.5:
            break
        break
    return sysobj

async def inject_failure(px4_dir: str, instance: int, key: str, log):
    cmd = FAIL_MAP.get(key)
    if not cmd:
        log(f"unknown failure key: {key}")
        return
    try:
        sh = px4_shell(px4_dir, instance)
        sh.sendline(cmd)
        sh.expect_exact("pxh>")
        sh.sendline("commander check")
        sh.expect_exact("pxh>")
        sh.close(force=True)
        log(f"injected: {cmd}")
    except Exception as e:
        log(f"inject failure error: {e}")

async def do_action(sysobj: System, action: dict, px4_dir: str, instance: int, log):
    cmd = action["cmd"]
    if cmd == "param_set":
        name, value = action["name"], float(action["value"])
        await sysobj.param.set_param_float(name, value)
        log(f"param set {name} {value}")
    elif cmd == "arm":
        await sysobj.action.arm(); log("armed")
    elif cmd == "takeoff":
        alt = float(action.get("alt", 2.5))
        await sysobj.action.set_takeoff_altitude(alt)
        await sysobj.action.takeoff(); log(f"takeoff {alt}m")
    elif cmd == "hold":
        await sysobj.action.hold(); log("hold")
    elif cmd == "land":
        await sysobj.action.land(); log("land")
    elif cmd == "inject_failure":
        await inject_failure(px4_dir, instance, action["failure"], log)
    else:
        log(f"UNKNOWN action {cmd}")

async def run_scenario(px4_dir: str, scenario: dict, outdir: Path):
    name      = scenario["name"]
    instance  = int(scenario.get("instance", 1))
    actions   = sorted(scenario.get("actions", []), key=lambda a: a.get("t", 0))
    outdir.mkdir(parents=True, exist_ok=True)
    logfile = (outdir / f"{name}.log").open("a")

    def log(msg):
        line = f"[{time.strftime('%H:%M:%S')}] {name}/i{instance}: {msg}"
        print(line, flush=True)
        logfile.write(line + "\n"); logfile.flush()

    sysobj = await connect_system(instance)
    log("connected")

    t0 = time.monotonic()
    i = 0
    while i < len(actions):
        now = time.monotonic() - t0
        next_t = float(actions[i].get("t", 0.0))
        if now < next_t:
            await asyncio.sleep(min(0.1, next_t - now))
            continue
        await do_action(sysobj, actions[i], px4_dir, instance, log)
        i += 1

    # Simple expectation checks
    expect = scenario.get("expect", {})
    if "final_mode" in expect:
        last_mode = None
        async for fm in sysobj.telemetry.flight_mode():
            last_mode = fm; break
        want = expect["final_mode"].upper()
        ok = (last_mode == getattr(FlightMode, want, None))
        log(f"expect final_mode={want} -> {ok} (got {last_mode})")

    logfile.close()

async def main():
    if len(sys.argv) < 3:
        print("Usage: runner.py <PX4_DIR> <scenarios.json> [outdir]")
        sys.exit(1)
    px4_dir   = sys.argv[1]
    scen_path = Path(sys.argv[2])
    outdir    = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("runs_out")
    scenarios = json.loads(scen_path.read_text())

    for s in scenarios:
        try:
            await run_scenario(px4_dir, s, outdir)
        except Exception as e:
            print(f"Scenario {s.get('name')} failed: {e}", file=sys.stderr)

if __name__ == "__main__":
    asyncio.run(main())
