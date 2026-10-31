"""
Turn an edited plan into files the recorder understands:

    config/spawn_plans/<name>.json                 the vehicles' start poses
    config/experiments/custom/<name>.yaml          the experiment that uses them

The YAML is then recorded like any other experiment:
    python main.py record --config config/experiments/custom/<name>.yaml --name <name>
"""
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from typing import List, Optional

import yaml

from ..backend.actors.spawn_plan import PROJECT_ROOT
from ..core.enums import Role

PLANS_DIR = os.path.join("config", "spawn_plans")
EXPERIMENTS_DIR = os.path.join("config", "experiments", "custom")


@dataclass
class ExportOptions:
    name: str
    extra_traffic: int = 0          # random vehicles added on top of the planned ones
    autopilot: bool = True
    num_frames: int = 10
    save_every: int = 20
    spawn_bbox: Optional[List[float]] = None
    camera_height: float = 70.0


def safe_name(text):
    """File-system friendly name: letters, digits, '_' and '-'."""
    name = re.sub(r"[^A-Za-z0-9_-]+", "_", text.strip()).strip("_")
    if not name:
        raise ValueError("Give the scenario a name (letters, digits, _ or -)")
    return name


def experiment_dict(plan, options, plan_path):
    counts = plan.counts()
    centre = plan.centre() or (0.0, 0.0)
    return {
        "server": {"town": plan.town},
        "scenario": {
            "num_cavs": counts[Role.CAV],
            "num_traffic": counts[Role.TRAFFIC] + options.extra_traffic,
            "spawn_center": [round(centre[0], 1), round(centre[1], 1)],
            "spawn_bbox": options.spawn_bbox,
            "spawn_plan": plan_path.replace(os.sep, "/"),
            "autopilot": options.autopilot,
        },
        "recording": {
            "warmup_ticks": 100 if options.autopilot else 20,
            "num_frames": options.num_frames,
            "save_every": options.save_every,
            "camera": "top",
            "camera_height": options.camera_height,
        },
    }


def export(plan, options, root=PROJECT_ROOT):
    """Write the plan and its experiment YAML; returns (plan_path, yaml_path), relative to `root`."""
    if plan.counts()[Role.CAV] == 0:
        raise ValueError("Place at least one CAV: they carry the LiDARs")
    name = safe_name(options.name)
    plan_rel = os.path.join(PLANS_DIR, name + ".json")
    yaml_rel = os.path.join(EXPERIMENTS_DIR, name + ".yaml")
    plan.save(os.path.join(root, plan_rel))
    os.makedirs(os.path.join(root, EXPERIMENTS_DIR), exist_ok=True)
    with open(os.path.join(root, yaml_rel), "w", encoding="utf-8") as fh:
        fh.write("# Made with the scenario editor (python main.py editor)\n")
        yaml.safe_dump(experiment_dict(plan, options, plan_rel), fh, sort_keys=False)
    return plan_rel, yaml_rel


def record_command(yaml_rel, name):
    return [sys.executable, "main.py", "record", "--config", yaml_rel.replace(os.sep, "/"), "--name", name]


def launch_recording(yaml_rel, name, root=PROJECT_ROOT):
    """Start the recording in its own console window, which stays open to show the result."""
    cmd = record_command(yaml_rel, name)
    if os.name == "nt":
        return subprocess.Popen(["cmd", "/k"] + cmd, cwd=root, creationflags=subprocess.CREATE_NEW_CONSOLE)
    return subprocess.Popen(cmd, cwd=root)
