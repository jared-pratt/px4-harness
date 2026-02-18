from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json
import shutil
import time
from typing import Optional, Dict, Any


def _ts() -> str:
    return time.strftime("%Y-%m-%d_%H-%M-%S")


@dataclass
class RunPaths:
    run_dir: Path
    processes_dir: Path
    scenarios_dir: Path
    ulog_dir: Path


def make_run_dir(out_root: Path) -> RunPaths:
    out_root.mkdir(parents=True, exist_ok=True)
    run_dir = out_root / f"{_ts()}_run"
    run_dir.mkdir(parents=True, exist_ok=True)

    processes_dir = run_dir / "processes"
    scenarios_dir = run_dir / "scenarios"
    ulog_dir = run_dir / "ulog"

    processes_dir.mkdir(parents=True, exist_ok=True)
    scenarios_dir.mkdir(parents=True, exist_ok=True)
    ulog_dir.mkdir(parents=True, exist_ok=True)

    return RunPaths(
        run_dir=run_dir,
        processes_dir=processes_dir,
        scenarios_dir=scenarios_dir,
        ulog_dir=ulog_dir,
    )


def write_metadata(run_dir: Path, meta: Dict[str, Any]) -> None:
    (run_dir / "metadata.json").write_text(json.dumps(meta, indent=2))


def copy_file(src: Path, dst_dir: Path, name: Optional[str] = None) -> None:
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / (name or src.name)
    shutil.copy2(src, dst)


def try_collect_px4_ulogs(px4_dir: Path, dest_ulog_dir: Path) -> None:
    """
    Best-effort uLog collection from common PX4 SITL locations.
    Won't crash if folders differ.
    """
    candidates = [
        px4_dir / "build" / "px4_sitl_default" / "rootfs" / "log",
        px4_dir / "build" / "px4_sitl_default" / "tmp" / "rootfs" / "log",
    ]
    dest_ulog_dir.mkdir(parents=True, exist_ok=True)

    for c in candidates:
        if not c.exists():
            continue
        for ulg in c.rglob("*.ulg"):
            try:
                rel = ulg.relative_to(c)
                out = dest_ulog_dir / rel
                out.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(ulg, out)
            except Exception:
                pass
