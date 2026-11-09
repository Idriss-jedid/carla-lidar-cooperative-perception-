"""
Two checks of the sensitivity analysis (offline, carla012 env; ground truth used for scoring only).

    python tools/audit_sensitivity.py [--out data/ablation/audit]

1. Sensor-height estimate: does it come from the LiDAR points, or from configuration metadata?
   - the estimator's inputs (estimate_seed_plane takes the points only);
   - its error in millimetres on every scan, against the recorded sensor pose (z of the sensor minus
     z of its own vehicle, used here for scoring only);
   - a perturbation test: every point's z shifted by +0.30 m and -0.50 m. An estimate inferred from
     the points must move by exactly -0.30 / +0.50 m; the config and poses are unchanged.

2. False BEV cells by channel count (16 / 32 / 64, points/s fixed): where they come from.
   For every ground point left in the non-ground cloud: its CAV, its distance, its height above the
   labelled road surface, and why the ground removal kept it, from the 8 m patch it falls in
   (same patch keys as ground.py):
     sparse     patch with < 18 returns: kept on purpose when no neighbour predicts the ground
     no plane   dense patch where no plane was accepted (continuity / lowest-surface rules)
     off plane  plane accepted, point more than 0.18 m from it
   The false-cell count is also redone on a grid shifted by half a cell (0.25 m) to rule out a
   grid-alignment artefact.

3. Causal test of the sparse-patch rule: the same 16 / 32 / 64-channel scans with the patch minimum
   lowered from 18 to 8 returns (an audit setting only, the processing keeps 18). If the 32-channel
   excess comes from sparse patches being kept, it must disappear; vehicle loss shows the cost.
"""
import argparse
import glob
import inspect
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from cavlidar.backend.geometry.transforms import sensor_to_world  # noqa: E402
from cavlidar.backend.processing import ground as ground_module  # noqa: E402
from cavlidar.backend.processing.ego_mask import ego_mask, geometry_for  # noqa: E402
from cavlidar.backend.processing.road_filter import road_mask  # noqa: E402
from eval_ablation import BEV_HALF, BEV_RES, _in_box, bev_cells, false_cells  # noqa: E402
from run_ablation import FACTORS, SCENES, find_run, run_name  # noqa: E402


def audit_scenes():
    """Sensor scenes that have every mount-height and channel run."""
    return [s for s, info in SCENES.items() if info["kind"] == "sensor"
            and all(find_run(run_name(s, f, v)) for f in ("height", "channels") for v in FACTORS[f][1])]

PATCH, MIN_POINTS, THRESHOLD = 8.0, 18, 0.18   # ground.py defaults


def load_scans(run):
    import yaml
    with open(os.path.join(run, "config.yaml"), encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    geometry = geometry_for(cfg["scenario"]["cav_blueprint"], cfg["lidar"])
    for meta_path in sorted(glob.glob(os.path.join(run, "frames", "*", "meta.json"))):
        with open(meta_path, encoding="utf-8") as fh:
            meta = json.load(fh)
        for cav, info in sorted(meta["cavs"].items()):
            pts = np.load(os.path.join(os.path.dirname(meta_path), cav + ".npy"))
            yield cav, int(meta["frame"]), pts, info, meta, geometry


def height_audit(area):
    params = list(inspect.signature(ground_module.estimate_seed_plane).parameters)
    rows = []
    for scene in audit_scenes():
        for value in FACTORS["height"][1]:
            run = find_run(run_name(scene, "height", value))
            errors, shifts, per_cav = [], [], {}
            for k, (cav, frame, pts, info, meta, geometry) in enumerate(load_scans(run)):
                keep = road_mask(pts, info["sensor_pose"], area) & ~ego_mask(pts, geometry)
                p = pts[keep]
                est = -ground_module.estimate_seed_plane(p)[2]
                own = next(v for v in meta["vehicles"] if v["id"] == info["vehicle_id"])
                true = info["sensor_pose"]["z"] - own["pose"]["z"]
                errors.append(1000 * (est - true))
                d = per_cav.setdefault(cav, {"x": own["pose"]["x"], "pitch_deg": own["pose"]["pitch"],
                                             "vertical": [], "mount": []})
                d["vertical"].append(1000 * (est - true))
                d["mount"].append(1000 * (est - value))
                if k < 4:  # perturbation on one sweep of each CAV
                    for dz in (0.30, -0.50):
                        q = p.copy()
                        q[:, 2] += dz
                        shifts.append(1000 * ((-ground_module.estimate_seed_plane(q)[2] - est) + dz))
            rows.append({"scene": scene, "mount_z": value, "scans": len(errors),
                         "error_mm_mean": float(np.mean(errors)), "error_mm_max_abs": float(np.max(np.abs(errors))),
                         "perturbation_residual_mm_max_abs": float(np.max(np.abs(shifts))),
                         "per_cav": {c: {"x": round(d["x"], 1), "pitch_deg": round(d["pitch_deg"], 1),
                                         "error_mm_vs_world_vertical": float(np.mean(d["vertical"])),
                                         "error_mm_vs_mount_z": float(np.mean(d["mount"]))}
                                     for c, d in sorted(per_cav.items())}})
    return {"estimator_inputs": params, "rows": rows}


def patch_status(points, ground):
    """Per point: patch point count and fraction of the patch labelled ground (ground.py's patch keys)."""
    keys = np.floor(points[:, :2] / PATCH).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.ravel()
    ground_per_patch = np.bincount(inverse, weights=ground.astype(float), minlength=len(counts))
    return counts[inverse], (ground_per_patch / counts)[inverse]


def channel_audit(area, raster):
    out = []
    for scene in audit_scenes():
        for value in FACTORS["channels"][1]:
            run = find_run(run_name(scene, "channels", value))
            per_cav, causes, dist, above = {}, {"sparse": 0, "no plane": 0, "off plane": 0}, [], []
            occupied = false_n = false_shift = scans = 0
            sweeps = {}
            for cav, frame, pts, info, meta, geometry in load_scans(run):
                scans += 1
                sweeps[cav] = sweeps.get(cav, 0) + 1
                ego = ego_mask(pts, geometry)
                road = road_mask(pts, info["sensor_pose"], area) & ~ego
                g = np.zeros(len(pts), bool)
                g[road] = ground_module.patchwise_ground_mask(pts[road])
                kept = road & ~g
                world = sensor_to_world(pts[:, :3].astype(np.float64), info["sensor_pose"])
                near_vehicle = np.zeros(len(pts), bool)
                for v in meta["vehicles"]:
                    near_vehicle |= _in_box(world, v, z_min=-1.0, margin=0.5)
                gt_ground = raster.is_ground(world[:, 0], world[:, 1], world[:, 2]) & ~near_vehicle
                cells = bev_cells(pts[kept][:, :2])
                shifted = bev_cells(pts[kept][:, :2] + BEV_RES / 2)
                fc = false_cells(cells, gt_ground[kept])
                per_cav[cav] = per_cav.get(cav, 0) + fc
                false_n += fc
                false_shift += false_cells(shifted, gt_ground[kept])
                occupied += len(np.unique(cells[cells >= 0]))
                left = gt_ground & kept
                n_patch, frac = patch_status(pts[road], g[road])
                idx = np.flatnonzero(road)
                pos = {i: k for k, i in enumerate(idx)}
                for i in np.flatnonzero(left):
                    k = pos[i]
                    if n_patch[k] < MIN_POINTS:
                        causes["sparse"] += 1
                    elif frac[k] == 0:
                        causes["no plane"] += 1
                    else:
                        causes["off plane"] += 1
                dist += np.hypot(pts[left, 0], pts[left, 1]).tolist()
                i, j = raster._cell(world[left, 0], world[left, 1])
                above += (world[left, 2] - raster.lo[i, j]).tolist()
            d = np.array(dist)
            total = sum(causes.values())
            out.append({
                "scene": scene, "channels": value,
                "false_cells_per_scan": false_n / scans, "false_cells_shifted_grid": false_shift / scans,
                "occupied_cells_per_scan": occupied / scans,
                "false_share_of_occupied_pct": 100.0 * false_n / max(1, occupied),
                "false_cells_by_cav": {c: n / sweeps[c] for c, n in sorted(per_cav.items())},
                "ground_left_points_per_scan": total / scans,
                "causes_pct": {c: 100.0 * n / max(1, total) for c, n in causes.items()},
                "distance_m": {"<30": int((d < 30).sum()), "30-60": int(((d >= 30) & (d < 60)).sum()),
                               ">=60": int((d >= 60).sum())},
                "height_above_road_m_median": float(np.median(above)) if above else float("nan"),
            })
    return out


def min_points_test(area, raster, values=(18, 8)):
    out = []
    for scene in audit_scenes():
        for channels in FACTORS["channels"][1]:
            run = find_run(run_name(scene, "channels", channels))
            acc = {m: dict(false=0, ground=0, ground_left=0, body=0, body_lost=0) for m in values}
            scans = 0
            for cav, frame, pts, info, meta, geometry in load_scans(run):
                scans += 1
                road = road_mask(pts, info["sensor_pose"], area) & ~ego_mask(pts, geometry)
                world = sensor_to_world(pts[:, :3].astype(np.float64), info["sensor_pose"])
                near_vehicle = np.zeros(len(pts), bool)
                body = np.zeros(len(pts), bool)
                for v in meta["vehicles"]:
                    near_vehicle |= _in_box(world, v, z_min=-1.0, margin=0.5)
                    if v["id"] != info["vehicle_id"]:
                        body |= _in_box(world, v, z_min=0.3)
                gt_ground = raster.is_ground(world[:, 0], world[:, 1], world[:, 2]) & ~near_vehicle
                for m in values:
                    g = np.zeros(len(pts), bool)
                    g[road] = ground_module.patchwise_ground_mask(pts[road], min_points=m)
                    kept = road & ~g
                    a = acc[m]
                    a["false"] += false_cells(bev_cells(pts[kept][:, :2]), gt_ground[kept])
                    a["ground"] += int(gt_ground.sum())
                    a["ground_left"] += int((gt_ground & kept).sum())
                    a["body"] += int(body.sum())
                    a["body_lost"] += int((body & ~kept).sum())
            for m in values:
                a = acc[m]
                out.append({"scene": scene, "channels": channels, "min_points": m,
                            "false_cells_per_scan": a["false"] / scans,
                            "ground_left_pct": 100.0 * a["ground_left"] / max(1, a["ground"]),
                            "vehicle_loss_pct": 100.0 * a["body_lost"] / max(1, a["body"])})
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default=os.path.join(ROOT, "data", "ablation", "audit"))
    args = parser.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    from cavlidar.api import DEFAULT_CONFIG
    from cavlidar.backend.maps import catalog
    from cavlidar.backend.maps.road_map import RoadMap
    from cavlidar.backend.processing.road_filter import load_drivable_area
    from cavlidar.core.config import load_config
    from cavlidar.editor import settings
    from eval_ground import HeightRaster
    info = catalog.find_map(settings.available_maps(load_config(DEFAULT_CONFIG)), "Town03")
    area = load_drivable_area(info.name, info.xodr_path, settings.MAPS_CACHE)
    raster = HeightRaster(RoadMap.from_file(info.name, info.xodr_path))
    result = {"height": height_audit(area), "channels": channel_audit(area, raster),
              "min_points": min_points_test(area, raster)}
    with open(os.path.join(args.out, "audit.json"), "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=1)
    print(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
