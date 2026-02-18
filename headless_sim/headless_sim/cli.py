from __future__ import annotations

import argparse
import asyncio
import json
import signal
import sys
import time
from pathlib import Path
from typing import Dict, Any, List, Set

from .sim_manager import SimManager
from .scenario_runner import run_all
from .artifacts import make_run_dir, write_metadata, copy_file, try_collect_px4_ulogs


def _parse_instances(scenario_path: Path) -> List[int]:
    scenarios = json.loads(scenario_path.read_text())
    insts: Set[int] = set()
    for s in scenarios:
        insts.add(int(s.get("instance", 1)))
    return sorted(insts)


def _substitute(cmd: str, *, px4_dir: Path, instance: int) -> str:
    return cmd.format(px4_dir=str(px4_dir), instance=str(instance))


def main() -> None:
    ap = argparse.ArgumentParser(prog="headless-sim")
    sub = ap.add_subparsers(dest="cmd", required=True)

    runp = sub.add_parser("run", help="Start sim processes (optional) and run scenarios headlessly")
    runp.add_argument("--px4-dir", required=True, help="Path to PX4-Autopilot directory")
    runp.add_argument("--scenario", required=True, help="Path to scenarios.json")
    runp.add_argument("--out", default="runs_out_headless", help="Output root folder")
    runp.add_argument("--sequential", action="store_true", help="Run scenarios sequentially instead of concurrently")

    runp.add_argument("--start-processes", action="store_true",
                      help="If set, start PX4/Gazebo processes; otherwise assumes they are already running")
    runp.add_argument("--px4-cmd", default="",
                      help="Command template to start PX4. Supports {px4_dir} and {instance}. "
                           "Example: 'bash -lc \"cd {px4_dir} && make px4_sitl gz_x500\"'")
    runp.add_argument("--gazebo-cmd", default="",
                      help="Command to start Gazebo server headless (optional). Example: 'gz sim -s <world.sdf>'")

    args = ap.parse_args()

    px4_dir = Path(args.px4_dir).expanduser().resolve()
    scenario_path = Path(args.scenario).expanduser().resolve()
    out_root = Path(args.out).expanduser().resolve()

    rp = make_run_dir(out_root)

    (rp.run_dir / "scenario_source").mkdir(parents=True, exist_ok=True)
    copy_file(scenario_path, rp.run_dir / "scenario_source", "scenarios.json")

    meta: Dict[str, Any] = {
        "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "px4_dir": str(px4_dir),
        "scenario_path": str(scenario_path),
        "start_processes": bool(args.start_processes),
        "px4_cmd_template": args.px4_cmd,
        "gazebo_cmd": args.gazebo_cmd,
        "sequential": bool(args.sequential),
    }
    write_metadata(rp.run_dir, meta)

    sm = SimManager(rp.processes_dir)

    def _shutdown(*_: Any) -> None:
        try:
            sm.stop()
        except Exception:
            pass

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        if args.start_processes:
            if args.gazebo_cmd.strip():
                sm.start("gazebo", args.gazebo_cmd)

            if not args.px4_cmd.strip():
                print("ERROR: --start-processes requires --px4-cmd", file=sys.stderr)
                sys.exit(2)

            instances = _parse_instances(scenario_path)
            for inst in instances:
                cmd = _substitute(args.px4_cmd, px4_dir=px4_dir, instance=inst)
                sm.start(f"px4_i{inst}", cmd)

            time.sleep(2.0)

        asyncio.run(run_all(str(px4_dir), scenario_path, rp.scenarios_dir, sequential=args.sequential))
    finally:
        try_collect_px4_ulogs(px4_dir, rp.ulog_dir)
        sm.stop()
        meta["finished_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        write_metadata(rp.run_dir, meta)
        print(f"\nDone. Artifacts in: {rp.run_dir}\n")


if __name__ == "__main__":
    main()
