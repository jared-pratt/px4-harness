#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PX4 SITL test runner:
- Connects to a PX4 SITL instance via MAVSDK (UDP 14540+instance)
- Executes time-ordered actions from scenarios.json
- Injects failures via PX4 pxh shell (mavlink_shell.py) and logs shell output
- Streams telemetry to per-scenario CSV for post-mortem analysis
"""
import asyncio, json, time, sys, csv, re
from pathlib import Path
import pexpect
import math


from mavsdk import System
from mavsdk.telemetry import FlightMode

# ===================== Helpers & config =====================

def px4_port_for_instance(instance: int) -> int:
    # PX4 SITL default mapping (seen in your logs)
    # instance i -> MAVSDK listen on UDP :14540+i
    return 14540 + int(instance)

def px4_shell(px4_dir: str, instance: int, timeout_s: float = 20.0) -> pexpect.spawn:
    """Open a PX4 pxh shell via mavlink_shell.py (binds to UDP port)"""
    tools = Path(px4_dir) / "Tools" / "mavlink_shell.py"
    port = px4_port_for_instance(instance)
    py_exec = "/usr/bin/python3"  # avoid venv python (pymavlink missing there)
    sh = pexpect.spawn(f"{py_exec} {tools} {port}", encoding="utf-8", timeout=timeout_s)
    sh.expect_exact("pxh>")
    return sh


# Some common “failure” shorthands. You can add more by running `failure -h` in pxh.
FAIL_MAP = {
    "imu_off":       "failure imu off",
    "gyro_off":      "failure gyro off",
    "accel_off":     "failure accel off",
    "mag_off":       "failure mag off",
    "baro_off":      "failure baro off",
    "gps_off":       "failure gps off",
    "battery_low":   "failure battery low",
    # examples you can enable later:
    # "motor_failure": "failure motor 1 stop",
}

def is_int_like(val: str) -> bool:
    try:
        int(val)
        return True
    except Exception:
        return False

def now_hms() -> str:
    return time.strftime('%H:%M:%S')

# meters N/E -> degrees lat/lon at a reference latitude
def _meters_to_latlon(d_north_m: float, d_east_m: float, ref_lat_deg: float, ref_lon_deg: float):
    dlat = d_north_m / 111_320.0
    dlon = d_east_m / (111_320.0 * math.cos(math.radians(ref_lat_deg)))
    return (ref_lat_deg + dlat, ref_lon_deg + dlon)

async def _get_home(sysobj):
    # Try home first, then fall back to current position
    async for h in sysobj.telemetry.home():
        return h
    async for p in sysobj.telemetry.position():
        return p
    return None


# ===================== Telemetry logging =====================

async def telemetry_logger(sysobj: System, csv_path: Path, stop_evt: asyncio.Event, tag: str):
    """
    Writes a CSV with periodic samples:
    t_s, flight_mode, in_air, armed, battery_pct, voltage_v, lat, lon, abs_alt_m,
    health flags (subset)
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

    try:
        while not stop_evt.is_set():
            t = time.monotonic() - t0
            fm = await get_flight_mode()
            ia = await get_in_air()
            ar = await get_armed()
            bt = await get_batt()
            ps = await get_pos()
            hl = await get_health()

            w.writerow([
                f"{t:.2f}", tag,
                str(fm) if fm else "",
                ia if ia is not None else "",
                ar if ar is not None else "",
                f"{getattr(bt, 'remaining_percent', 0.0):.3f}" if bt else "",
                f"{getattr(bt, 'voltage_v', 0.0):.3f}" if bt else "",
                f"{getattr(ps, 'latitude_deg', 0.0):.7f}" if ps else "",
                f"{getattr(ps, 'longitude_deg', 0.0):.7f}" if ps else "",
                f"{getattr(ps, 'absolute_altitude_m', 0.0):.3f}" if ps else "",
                getattr(hl, 'is_gyrometer_calibration_ok', None),
                getattr(hl, 'is_accelerometer_calibration_ok', None),
                getattr(hl, 'is_magnetometer_calibration_ok', None),
                getattr(hl, 'is_global_position_ok', None),
                getattr(hl, 'is_local_position_ok', None),
                getattr(hl, 'is_home_position_ok', None),
            ])
            f.flush()
            await asyncio.sleep(0.20)
    finally:
        f.close()




# ===================== Shell / PXH actions =====================

async def pxh_run(px4_dir: str, instance: int, command: str, log, out_path: Path = None):
    """
    Run arbitrary pxh command (e.g. 'failure gps off', 'listener vehicle_status', etc.)
    Capture output until the next 'pxh>' prompt.
    """
    try:
        sh = px4_shell(px4_dir, instance)
        sh.sendline(command)
        sh.expect("pxh>")
        output = sh.before  # text printed between command and prompt
        sh.close(force=True)
        if out_path:
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(output)
        # Also print the first lines into log (truncated)
        head = "\n".join(output.splitlines()[:20])
        log(f"pxh[{command!r}] ->\n{head}")
    except Exception as e:
        log(f"pxh error: {e}")

async def pxh_inject_failure(px4_dir: str, instance: int, key: str, log, out_dir: Path):
    cmd = FAIL_MAP.get(key)
    if not cmd:
        log(f"unknown failure key: {key}")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"pxh_{key}.txt"

    try:
        sh = px4_shell(px4_dir, instance)
        # 1) inject the failure
        sh.sendline(cmd)
        sh.expect("pxh>")
        fail_out = sh.before

        # 2) show commander status
        sh.sendline("commander check")
        sh.expect("pxh>")
        chk_out = sh.before

        # 3) show vehicle_status once
        sh.sendline("listener vehicle_status -n 1")
        sh.expect("pxh>")
        stat_out = sh.before

        # (optional) IMU snapshot if relevant
        if "imu" in key:
            sh.sendline("listener sensor_combined -n 1")
            sh.expect("pxh>")
            imu_out = sh.before
        else:
            imu_out = ""

        sh.close(force=True)

        blob = (
            f"$ {cmd}\n{fail_out}\n\n"
            f"$ commander check\n{chk_out}\n\n"
            f"$ listener vehicle_status -n 1\n{stat_out}\n\n"
            + (f"$ listener sensor_combined -n 1\n{imu_out}\n" if imu_out else "")
        )
        out_path.write_text(blob)
        # print everything to the console too (so you *see* it live)
        log(f"=== pxh output for {key} ===\n{blob}")

    except Exception as e:
        log(f"pxh error: {e}")


# ===================== MAVSDK actions =====================

async def connect_system(instance: int, log) -> System:
    sysobj = System()
    port = px4_port_for_instance(instance)
    await sysobj.connect(system_address=f"udp://:{port}")

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

async def do_action(sysobj: System, action: dict, px4_dir: str, instance: int, log, out_dir: Path):
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
        if isinstance(value, (int,)) or (isinstance(value, str) and is_int_like(value)):
            await sysobj.param.set_param_int(name, int(value))
        else:
            await sysobj.param.set_param_float(name, float(value))
        log(f"param set {name} -> {value}")

    elif cmd == "pxh":
        # run arbitrary pxh command
        pxh_cmd = action["command"]
        fname = re.sub(r"[^\w.-]+", "_", pxh_cmd)[:64] or "pxh_cmd"
        await pxh_run(px4_dir, instance, pxh_cmd, log, out_dir / f"pxh_{fname}.txt")

    elif cmd == "inject_failure":
        await pxh_inject_failure(px4_dir, instance, action["failure"], log, out_dir)

    elif cmd == "set_speed":
        # Max horizontal speed (m/s). Older MAVSDK may not have set_maximum_speed.
        spd = float(action["mps"])
        try:
            setter = getattr(sysobj.action, "set_maximum_speed", None)
            if setter:
                await setter(spd)
                log(f"set speed {spd} m/s via MAVSDK")
            else:
                # Fallback via PX4 params:
                # MPC_XY_CRUISE / MPC_XY_VEL_MAX are m/s; MIS_SPEED is cm/s for missions
                await sysobj.param.set_param_float("MPC_XY_CRUISE", spd)
                await sysobj.param.set_param_float("MPC_XY_VEL_MAX", spd)
                await sysobj.param.set_param_int("MIS_SPEED", int(spd * 100))
                log(f"set speed via params: MPC_XY_CRUISE/MPC_XY_VEL_MAX={spd} m/s, MIS_SPEED={int(spd*100)} cm/s")
        except Exception as e:
            log(f"set_speed ignored: {e}")


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
        log(f"goto_offset N{action['north_m']} E{action['east_m']} -> {lat:.7f},{lon:.7f} alt {alt:.2f} (AMSL)")


    elif cmd == "goto_abs":
        # Absolute WGS84 destination
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


    else:
        log(f"UNKNOWN action {cmd}")

# ===================== Scenario engine =====================

async def run_scenario(px4_dir: str, scenario: dict, out_root: Path):
    name     = scenario["name"]
    instance = int(scenario.get("instance", 1))
    actions  = sorted(scenario.get("actions", []), key=lambda a: float(a.get("t", 0.0)))

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
    log("connected")

    # start telemetry CSV task
    stop_evt = asyncio.Event()
    tel_task = asyncio.create_task(telemetry_logger(sysobj, csv_path, stop_evt, tag=name))

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
            await do_action(sysobj, actions[idx], px4_dir, instance, log, scen_dir)
            idx += 1

        # Optional expectations
        expect = scenario.get("expect", {})
        if "final_mode" in expect:
            # sample: verify we ended up in LAND after 'land'
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
        logf.close()

async def main():
    if len(sys.argv) < 3:
        print("Usage: runner.py <PX4_DIR> <scenarios.json> [outdir]")
        sys.exit(1)

    px4_dir   = sys.argv[1]
    scen_path = Path(sys.argv[2])
    out_root  = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("runs_out")

    scenarios = json.loads(scen_path.read_text())

    # Run sequentially (simpler to reason about ports and logs).
    for s in scenarios:
        try:
            await run_scenario(px4_dir, s, out_root)
        except Exception as e:
            print(f"[{now_hms()}] Scenario {s.get('name')} failed: {e}", file=sys.stderr)

if __name__ == "__main__":
    asyncio.run(main())
