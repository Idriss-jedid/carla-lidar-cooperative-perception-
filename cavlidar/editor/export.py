"""
Turn an edited plan into files the recorder understands:

    config/spawn_plans/<name>.json                 the vehicles' start poses
    config/experiments/custom/<name>.yaml          the experiment that uses them

The YAML is then recorded like any other experiment:
    python main.py record --config config/experiments/custom/<name>.yaml --name <name> [--process]

Only what the editor shows is written: the scene, the recording, and (when set in the editor) the LiDAR
settings, the seed and the red-light range. The time step is never written: it stays
1 / lidar.rotation_frequency (0.05 s), so every tick is one full LiDAR sweep.
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
    lidar: Optional[dict] = None          # e.g. {"channels": 32, "range": 120.0, ...}; None = default.yaml
    seed: Optional[int] = None            # simulation.seed; None = default.yaml
    ignore_lights: Optional[List[float]] = None   # scenario.ignore_lights_range [min, max] %; None = default.yaml
    process: bool = False                 # record with --process: straight on to the BEV


def safe_name(text):
    """File-system friendly name: letters, digits, '_' and '-'."""
    name = re.sub(r"[^A-Za-z0-9_-]+", "_", text.strip()).strip("_")
    if not name:
        raise ValueError("Give the scenario a name (letters, digits, _ or -)")
    return name


LIDAR_KEYS = ("channels", "range", "points_per_second", "mount_z", "noise_stddev")


def experiment_dict(plan, options, plan_path):
    counts = plan.counts()
    centre = plan.centre() or (0.0, 0.0)
    data = {
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
    if options.ignore_lights is not None:
        data["scenario"]["ignore_lights_range"] = [float(v) for v in options.ignore_lights]
    if options.seed is not None:
        data["simulation"] = {"seed": int(options.seed)}
    if options.lidar:
        unknown = set(options.lidar) - set(LIDAR_KEYS)
        if unknown:
            raise ValueError("Unknown LiDAR settings: {}".format(", ".join(sorted(unknown))))
        data["lidar"] = {k: options.lidar[k] for k in LIDAR_KEYS if k in options.lidar}
    return data


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


def record_command(yaml_rel, name, process=False):
    cmd = [sys.executable, "main.py", "record", "--config", yaml_rel.replace(os.sep, "/"), "--name", name]
    return cmd + ["--process"] if process else cmd


def launch_recording(yaml_rel, name, root=PROJECT_ROOT, process=False):
    """Start the recording (and the processing) in its own console window, which stays open to show the result."""
    cmd = record_command(yaml_rel, name, process)
    if os.name == "nt":
        return subprocess.Popen(["cmd", "/k"] + cmd, cwd=root, creationflags=subprocess.CREATE_NEW_CONSOLE)
    return subprocess.Popen(cmd, cwd=root)
