"""
Validate ground removal on recorded runs against CARLA's ground truth.

Ground truth is used HERE ONLY, to score a ground mask; the ground removal itself
(cavlidar/backend/processing/ground.py) sees nothing but the LiDAR points.

    python tools/eval_ground.py data/runs/<run> [<run> ...] [--frames 3] [--baseline old_ground.py]

Labels (world frame, from the frame's meta.json and the map's OpenDRIVE):
  vehicle point  inside another vehicle's box (in the vehicle's own, possibly pitched
                 frame), more than 0.3 m above the box bottom: its body, not the road
  ground point   not near any vehicle, within 0.3 m of the road / sidewalk height
                 and within 7 m of a lane centre (height from the lane samples)
Scores:
  vehicle loss   % of vehicle points wrongly labelled ground (lower is better)
  ground kept    % of ground points left in the non-ground cloud (lower is better)
"""
import argparse
import glob
import importlib.util
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cavlidar.backend.geometry.transforms import pose_to_matrix, sensor_to_world  # noqa: E402

GROUND_TOL = 0.3       # m between a point and the road / sidewalk height
ROAD_REACH = 7         # m (cells) from a lane centre that still counts as road / sidewalk
BODY_MIN = 0.3         # m above a vehicle's base where its body starts
SLOPE = (15.0, 85.0, -142.0, -128.0)   # Town03 junction 1221, +x leg climbing to z = 8 m


class HeightRaster:
    """Lowest and highest road height in 1 m cells, spread up to ROAD_REACH m from the lanes."""

    def __init__(self, road_map):
        xyz = road_map._xyz
        self.x0, self.y0 = xyz[:, 0].min() - ROAD_REACH - 1, xyz[:, 1].min() - ROAD_REACH - 1
        nx = int(xyz[:, 0].max() - self.x0) + ROAD_REACH + 2
        ny = int(xyz[:, 1].max() - self.y0) + ROAD_REACH + 2
        lo = np.full((nx, ny), np.inf)
        hi = np.full((nx, ny), -np.inf)
        i, j = self._cell(xyz[:, 0], xyz[:, 1])
        np.minimum.at(lo, (i, j), xyz[:, 2])
        np.maximum.at(hi, (i, j), xyz[:, 2])
        for _ in range(ROAD_REACH):  # grow into empty cells from their neighbours
            empty = ~np.isfinite(lo)
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                lo_n = np.roll(np.roll(lo, di, 0), dj, 1)
                hi_n = np.roll(np.roll(hi, di, 0), dj, 1)
                fill = empty & np.isfinite(lo_n)
                lo[fill], hi[fill] = lo_n[fill], hi_n[fill]
                empty &= ~fill
        self.lo, self.hi = lo, hi

    def _cell(self, x, y):
        i = np.clip(np.floor(x - self.x0).astype(int), 0, None)
        j = np.clip(np.floor(y - self.y0).astype(int), 0, None)
        return i, j

    def is_ground(self, x, y, z):
        i, j = self._cell(x, y)
        ok = (i < self.lo.shape[0]) & (j < self.lo.shape[1])
        i, j = np.where(ok, i, 0), np.where(ok, j, 0)
        lo, hi = self.lo[i, j], self.hi[i, j]
        near = np.isfinite(lo) & ok
        return near & ((np.abs(z - lo) <= GROUND_TOL) | (np.abs(z - hi) <= GROUND_TOL)
                       | ((z > lo) & (z < hi)))


def _in_box(world, vehicle, z_min, margin=0.1):
    """Points inside the vehicle's box (its own frame, so pitched on a slope), `z_min` m above its bottom."""
    e, off = vehicle["extent"], vehicle.get("center_offset", [0.0, 0.0, vehicle["extent"][2]])
    xyz1 = np.hstack([world[:, :3], np.ones((len(world), 1))])
    local = xyz1 @ np.linalg.inv(pose_to_matrix(vehicle["pose"])).T
    a, b = local[:, 0] - off[0], local[:, 1] - off[1]
    z = local[:, 2] - (off[2] - e[2])  # height above the box bottom
    return (np.abs(a) <= e[0] + margin) & (np.abs(b) <= e[1] + margin) & (z > z_min) & (z < 2 * e[2] + 0.3)


def load_ground_fn(path=None):
    """patchwise_ground_mask of the package, or of an older copy of ground.py."""
    if path is None:
        from cavlidar.backend.processing.ground import patchwise_ground_mask
        return patchwise_ground_mask
    spec = importlib.util.spec_from_file_location("ground_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.patchwise_ground_mask


def evaluate(run_dir, ground_fn, raster, frames=None):
    """Counts per run; `frames` = how many of the last frames (None = all)."""
    metas = sorted(glob.glob(os.path.join(run_dir, "frames", "*", "meta.json")))
    if frames:
        metas = metas[-frames:]
    c = {"vehicle": 0, "vehicle_lost": 0, "ground": 0, "ground_kept": 0, "slope_ground": 0,
         "slope_ground_kept": 0, "slope_vehicle": 0, "slope_vehicle_lost": 0, "seconds": [], "pairs": {}}
    for meta_path in metas:
        with open(meta_path, encoding="utf-8") as fh:
            meta = json.load(fh)
        folder = os.path.dirname(meta_path)
        for cav, info in sorted(meta["cavs"].items()):
            points = np.load(os.path.join(folder, cav + ".npy"))
            start = time.perf_counter()
            ground = ground_fn(points)
            c["seconds"].append(time.perf_counter() - start)
            world = sensor_to_world(points[:, :3], info["sensor_pose"])
            near_vehicle = np.zeros(len(points), bool)
            body = np.zeros(len(points), bool)
            for v in meta["vehicles"]:
                if v["id"] == info["vehicle_id"]:
                    continue
                near_vehicle |= _in_box(world, v, z_min=-1.0, margin=0.5)
                inside = _in_box(world, v, z_min=BODY_MIN)
                body |= inside
                if inside.sum() >= 50:  # pairs that see each other well (close pairs)
                    key = "{}->{}".format(cav, v["name"])
                    pair = c["pairs"].setdefault(key, [0, 0])
                    pair[0] += int(inside.sum())
                    pair[1] += int((inside & ground).sum())
            is_ground = raster.is_ground(world[:, 0], world[:, 1], world[:, 2]) & ~near_vehicle
            on_slope = ((world[:, 0] >= SLOPE[0]) & (world[:, 0] <= SLOPE[1]) & (world[:, 1] >= SLOPE[2])
                        & (world[:, 1] <= SLOPE[3]) & (world[:, 2] > 0.5))
            c["vehicle"] += int(body.sum())
            c["vehicle_lost"] += int((body & ground).sum())
            c["ground"] += int(is_ground.sum())
            c["ground_kept"] += int((is_ground & ~ground).sum())
            c["slope_ground"] += int((is_ground & on_slope).sum())
            c["slope_ground_kept"] += int((is_ground & on_slope & ~ground).sum())
            c["slope_vehicle"] += int((body & on_slope).sum())
            c["slope_vehicle_lost"] += int((body & on_slope & ground).sum())
    c["scans"] = len(c["seconds"])
    return c


def pct(a, b):
    return 100.0 * a / b if b else float("nan")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--frames", type=int, default=None, help="last N frames of each run (default all)")
    parser.add_argument("--baseline", help="an older ground.py to compare with")
    parser.add_argument("--town", default="Town03")
    parser.add_argument("--json", help="write all counts to this file")
    args = parser.parse_args(argv)

    from cavlidar.api import DEFAULT_CONFIG
    from cavlidar.core.config import load_config
    from cavlidar.editor import settings
    road_map = settings.road_map(load_config(DEFAULT_CONFIG), args.town)
    if road_map is None:
        raise SystemExit("No OpenDRIVE file for " + args.town)
    raster = HeightRaster(road_map)

    methods = [("new", load_ground_fn())]
    if args.baseline:
        methods.insert(0, ("before", load_ground_fn(args.baseline)))
    results = {}
    for run in args.runs:
        for label, fn in methods:
            c = evaluate(run, fn, raster, args.frames)
            results.setdefault(os.path.basename(run), {})[label] = c
            print("{:<40} {:<6} scans {:3d} | vehicle loss {:5.1f}% of {:6d} | ground kept {:5.1f}% of {:7d} | "
                  "slope ground kept {:5.1f}% of {:5d} | slope vehicle loss {:5.1f}% of {:4d} | {:.0f} ms/CAV".format(
                      os.path.basename(run), label, c["scans"], pct(c["vehicle_lost"], c["vehicle"]), c["vehicle"],
                      pct(c["ground_kept"], c["ground"]), c["ground"],
                      pct(c["slope_ground_kept"], c["slope_ground"]), c["slope_ground"],
                      pct(c["slope_vehicle_lost"], c["slope_vehicle"]), c["slope_vehicle"],
                      1000 * float(np.mean(c["seconds"]))))
            for key, (n, lost) in sorted(c["pairs"].items(), key=lambda kv: -kv[1][0])[:4]:
                print("{:>48} {:<22} {:5.1f}% of {:5d}".format("pair", key, pct(lost, n), n))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(results, fh, indent=1)


if __name__ == "__main__":
    main()
