"""
Recorded runs and their BEV, for the editor's BEV tab (no window, no CARLA: unit-tested).

    runs = list_runs(runs_dir(cfg))              # newest first
    overlay = bev_overlay(runs[0].path, frame)   # occupied BEV cells of every CAV + recorded boxes, world frame
"""
import os
from dataclasses import dataclass, field
from typing import List

import numpy as np
import yaml

from ..backend.actors.spawn_plan import PROJECT_ROOT
from ..backend.geometry.transforms import box_corners_2d
from ..backend.recording import storage
from ..backend.visualization import pipeline


@dataclass
class RunInfo:
    name: str
    path: str
    town: str
    frames: List[int] = field(default_factory=list)
    processed: List[int] = field(default_factory=list)

    @property
    def label(self):
        done = len(self.processed)
        state = "BEV ready" if done == len(self.frames) and done else (
            "{}/{} processed".format(done, len(self.frames)) if done else "not processed")
        return "{}  ({}, {} frames, {})".format(self.name, self.town or "?", len(self.frames), state)


@dataclass
class CavBev:
    name: str
    sensor_pose: dict
    cells: np.ndarray            # (K, 4, 2) world-frame corners of the occupied cells
    non_ground_points: int


@dataclass
class BevOverlay:
    run: str
    frame: int
    town: str
    cavs: List[CavBev]
    vehicles: List[tuple]        # (name, role, (4, 2) world-frame box corners), ground truth of the frame

    def bounds(self, margin=10.0):
        """(x_min, x_max, y_min, y_max) around every cell and vehicle."""
        pts = [c.cells.reshape(-1, 2) for c in self.cavs if len(c.cells)]
        pts += [np.array([[c.sensor_pose["x"], c.sensor_pose["y"]]]) for c in self.cavs]
        pts += [corners for _, _, corners in self.vehicles]
        allp = np.vstack(pts)
        lo, hi = allp.min(axis=0) - margin, allp.max(axis=0) + margin
        return float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1])


def runs_dir(cfg):
    """recording.output_dir, relative to the project folder unless absolute."""
    out = cfg.recording.output_dir
    return os.path.normpath(out if os.path.isabs(out) else os.path.join(PROJECT_ROOT, out))


def _town(run_dir):
    try:
        with open(os.path.join(run_dir, "config.yaml"), encoding="utf-8") as fh:
            return (yaml.safe_load(fh) or {}).get("server", {}).get("town", "")
    except OSError:
        return ""


def run_info(run_dir):
    frames = storage.list_frames(run_dir)
    return RunInfo(os.path.basename(os.path.normpath(run_dir)), run_dir, _town(run_dir), frames,
                   [f for f in frames if pipeline.is_processed(run_dir, f)])


def list_runs(directory):
    """Every run folder (with frames/) in `directory`, newest first (folder names start with date and time)."""
    if not os.path.isdir(directory):
        return []
    names = sorted((d for d in os.listdir(directory) if os.path.isdir(os.path.join(directory, d, "frames"))),
                   reverse=True)
    return [run_info(os.path.join(directory, d)) for d in names]


def bev_overlay(run_dir, frame):
    """The processed BEV of every CAV of one frame, and the frame's vehicle boxes, in the world frame."""
    if not pipeline.is_processed(run_dir, frame):
        raise ValueError("Frame {} of {} is not processed yet".format(frame, run_dir))
    meta = storage.load_json(os.path.join(pipeline.frame_dir(run_dir, frame), "meta.json"))
    pdir = pipeline.processed_dir(run_dir, frame)
    summary = storage.load_json(os.path.join(pdir, "summary.json"))
    half = float(summary["settings"]["bev_half_range_m"])
    cavs = []
    for name in sorted(meta["cavs"]):
        pose = meta["cavs"][name]["sensor_pose"]
        occupancy = np.load(os.path.join(pdir, "{}_bev_occupancy.npy".format(name)))
        cavs.append(CavBev(name, pose, pipeline.bev_cells_world(occupancy, (-half, half, -half, half), pose),
                           int(summary["cavs"][name]["non_ground_points"])))
    vehicles = [(v["name"], v["role"], box_corners_2d(v["pose"], v["extent"], v["center_offset"]))
                for v in meta.get("vehicles", [])]
    return BevOverlay(os.path.basename(os.path.normpath(run_dir)), frame, meta.get("town", ""), cavs, vehicles)


def figure_path(run_dir, frame, cav):
    return os.path.join(pipeline.processed_dir(run_dir, frame), cav + pipeline.FIGURE_SUFFIX)


def make_view(run_dir, frame, which, cav=None):
    """One before / after figure ('world_raw', 'world_filtered', 'world_bev', 'vehicles', or 'cav' with `cav`),
    and vehicles.csv / cavs.csv next to it; returns the figure's path."""
    return pipeline.make_view(run_dir, frame, which, cav)


def make_figure(run_dir, frame, cav):
    """The four-panel figure (raw -> road -> ground removed -> BEV + sectors) of one CAV; returns its path."""
    stages = pipeline.stages_from_processed(run_dir, frame, cav)
    title = "{}, {}, frame {}".format(os.path.basename(os.path.normpath(run_dir)), cav, frame)
    return pipeline.save_figure(stages, title, figure_path(run_dir, frame, cav))
