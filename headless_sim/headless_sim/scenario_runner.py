from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any, Dict, List

import runner  # imports your existing repo-root runner.py


async def run_all(px4_dir: str, scenario_path: Path, out_scenarios_dir: Path, sequential: bool = False) -> None:
    scenarios: List[Dict[str, Any]] = json.loads(scenario_path.read_text())

    if sequential:
        for s in scenarios:
            await runner.run_scenario(px4_dir, s, out_scenarios_dir)
        return

    tasks = [runner.run_scenario(px4_dir, s, out_scenarios_dir) for s in scenarios]
    results = await asyncio.gather(*tasks, return_exceptions=True)

    for s, res in zip(scenarios, results):
        if isinstance(res, Exception):
            print(f"[scenario_runner] Scenario {s.get('name')} failed: {res}")
