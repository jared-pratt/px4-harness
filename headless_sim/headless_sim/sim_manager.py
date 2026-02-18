from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, List


@dataclass
class ManagedProc:
    name: str
    popen: subprocess.Popen
    log_path: Path


class SimManager:
    """
    Starts/stops background processes (PX4 SITL, Gazebo server).
    Uses process groups so Ctrl+C / stop() can kill everything cleanly.
    """
    def __init__(self, processes_dir: Path):
        self.processes_dir = processes_dir
        self.procs: List[ManagedProc] = []

    def start(self, name: str, cmd: str, env: Optional[Dict[str, str]] = None) -> None:
        self.processes_dir.mkdir(parents=True, exist_ok=True)
        log_path = self.processes_dir / f"{name}.log"
        logf = log_path.open("w")

        popen = subprocess.Popen(
            cmd,
            shell=True,
            stdout=logf,
            stderr=subprocess.STDOUT,
            env=(os.environ | (env or {})),
            preexec_fn=os.setsid,  # new process group
            text=True,
        )
        self.procs.append(ManagedProc(name=name, popen=popen, log_path=log_path))

    def stop(self, timeout_s: float = 5.0) -> None:
        # Graceful terminate
        for p in self.procs:
            if p.popen.poll() is None:
                try:
                    os.killpg(os.getpgid(p.popen.pid), signal.SIGTERM)
                except Exception:
                    pass

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if all(p.popen.poll() is not None for p in self.procs):
                return
            time.sleep(0.1)

        # Hard kill
        for p in self.procs:
            if p.popen.poll() is None:
                try:
                    os.killpg(os.getpgid(p.popen.pid), signal.SIGKILL)
                except Exception:
                    pass
