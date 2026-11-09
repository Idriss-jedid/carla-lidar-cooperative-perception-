"""
Score the LiDAR sensitivity benchmark recorded by tools/run_ablation.py (offline, carla012 env).

    python tools/eval_ablation.py                         # every scene -> data/ablation/benchmark/
    python tools/eval_ablation.py --scenes flat slope --ego-mask off --out data/ablation/before
    python tools/eval_ablation.py --check-processing abl_flat2_base abl_flat2_invalid_mount
    python tools/eval_ablation.py --timing-repeats 3      # processing time = fastest of 3 runs per scan
    python tools/eval_ablation.py --check-labels abl_moving1_noise_0  # ground-truth boxes vs moving vehicles

Every scan goes through the project's processing exactly as python -m cavlidar.backend.processing
does it (ego-body mask -> road filter -> ground removal -> 0.5 m BEV -> static sectors;
--check-processing proves it on recorded runs), then is scored against CARLA's ground truth
(used HERE ONLY, never by the processing). Per scan, pooled over the scans of a run:

  points / kept      returns per scan / returns left in the non-ground cloud (what the BEV is built from)
  ego_removed        returns removed by the ego-body mask; ego_removed_vehicle = other vehicles' body
                     points among them (must be 0); own_left = own-body returns still in the cloud
  ground_left        % of ground points (road / sidewalk height, tools/eval_ground.py labels) still in
                     the non-ground cloud                       -> ground removal error
  vehicle_loss       % of other vehicles' body points removed (off-road, ground or ego mask)
  bev_cells          occupied 0.5 m BEV cells (+-60 m) per scan
  observed_cells     0.5 m BEV cells (+-60 m) that receive at least one road return, ground or not:
                     how much of the road the sensor observes (BEV coverage)
  bev_false_cells    occupied cells whose points are ALL labelled ground (cells that should be empty)
  coverage           % of other vehicles with their centre in the BEV window and >= 10 body points
                     kept (vehicle visibility); split near (< 30 m) / far (30-60 m)
  quality            mean sector quality S~ (4 static sectors from pi/4, 0.2 m downsampling)
  *_50               the same restricted to the common 0-50 m region around the sensor (range)
  est_height         sensor height above the ground from the ground removal's seed plane
  ms                 ego mask + road filter + ground removal + BEV, median per scan
  *_slope            ground_left / vehicle_loss on the sloped leg (x 15-85 m, z > 0.5 m)
Per frame, all CAVs of the frame together (cooperative view, union of their non-ground clouds in
the world, disc of 60 m around junction 1221; every vehicle is a target, CAVs included):
  coop_coverage      % of vehicles with >= 10 body points kept by the CAVs in total
  coop_cells / coop_false_cells   occupied / all-ground 0.5 m world cells of the union
  coop_observed_cells             world cells of the disc observed by at least one CAV (road returns)
Normalised by the observed road (derived from the run means, i.e. pooled over the run's frames):
  false_per_1000_observed         1000 x bev_false_cells / observed_cells (per CAV)
  coop_false_per_1000_observed    1000 x coop_false_cells / coop_observed_cells: more CAVs observe more
                                  road, so their raw union false cells grow even at a constant error rate

A run whose LiDAR mount lies inside the car body (EgoGeometry.mount_valid False) is scored but
EXCLUDED from every table, aggregate and plot. Scenes of the same terrain are aggregated as
mean +- sample standard deviation across scenes, and as the paired difference to each scene's own
baseline. Scored runs are cached in <out>/runs/ (--rescore to redo them).
"""
import argparse
import glob
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from cavlidar.backend.geometry.transforms import sensor_to_world  # noqa: E402
from cavlidar.backend.processing.bev_grid import bev_from_points  # noqa: E402
from cavlidar.backend.processing.ego_mask import DEFAULT_MARGIN, ego_mask, geometry_for  # noqa: E402
from cavlidar.backend.processing.sectors import sector_metadata, static_sectors  # noqa: E402
from cavlidar.backend.processing.ground import estimate_seed_plane, patchwise_ground_mask  # noqa: E402
from cavlidar.backend.processing.road_filter import load_drivable_area, road_mask  # noqa: E402
from eval_ground import SLOPE, HeightRaster, _in_box  # noqa: E402
from run_ablation import (BASELINE, CHECKS, FACTORS, POPULATION_BASELINE, POPULATION_FACTORS,  # noqa: E402
                          SCENES, find_run, run_name)

BEV_RES, BEV_HALF = 0.5, 60.0
MIN_BODY_POINTS = 10      # a vehicle counts as represented with at least this many kept body points
NEAR = 30.0               # m, near / far split of the coverage
COMMON = 50.0             # m, region every range setting covers
SECTORS = static_sectors(4, np.pi / 4)
COOP_CENTRE, COOP_RADIUS = (2.9, -136.5), 60.0
EVAL_VERSION = 3          # bump when a metric definition changes: cached runs are rescored
FACTOR_ORDER = list(FACTORS) + list(POPULATION_FACTORS)


def bev_cells(points_xy):
    """Flat BEV cell index of each point (same layout as bev_from_points), -1 outside the grid."""
    n = int(round(2 * BEV_HALF / BEV_RES))
    col = np.floor((points_xy[:, 1] + BEV_HALF) / BEV_RES).astype(np.int64)
    row = n - 1 - np.floor((points_xy[:, 0] + BEV_HALF) / BEV_RES).astype(np.int64)
    inside = (row >= 0) & (row < n) & (col >= 0) & (col < n)
    return np.where(inside, row * n + col, -1)


def false_cells(cells, is_ground):
    """Number of occupied cells whose points are all ground (cells = flat indices, -1 ignored)."""
    keep = cells >= 0
    if not keep.any():
        return 0
    c, g = cells[keep], is_ground[keep]
    occupied = np.unique(c)
    with_other = np.unique(c[~g])
    return int(len(np.setdiff1d(occupied, with_other)))


def coverage(sensor_pose, vehicles, own_id, body_kept_by_vehicle):
    """{near: (covered, total), far: (covered, total)} for the other vehicles in the BEV window."""
    m = np.linalg.inv(_pose_matrix(sensor_pose))
    out = {"near": [0, 0], "far": [0, 0], "within50": [0, 0]}
    for v in vehicles:
        if v["id"] == own_id:
            continue
        p = v["pose"]
        local = m @ np.array([p["x"], p["y"], p["z"], 1.0])
        if abs(local[0]) >= BEV_HALF or abs(local[1]) >= BEV_HALF:
            continue
        dist = np.hypot(local[0], local[1])
        covered = int(body_kept_by_vehicle.get(v["id"], 0) >= MIN_BODY_POINTS)
        key = "near" if dist < NEAR else "far"
        out[key][1] += 1
        out[key][0] += covered
        if dist < COMMON:
            out["within50"][1] += 1
            out["within50"][0] += covered
    return out


def _pose_matrix(pose):
    from cavlidar.backend.geometry.transforms import pose_to_matrix
    return pose_to_matrix(pose)


def process_scan(points, sensor_pose, area, geometry=None, margin=DEFAULT_MARGIN):
    """The project's processing of one scan (no ground truth): ego, road, ground, kept masks, BEV, stage ms."""
    t0 = time.perf_counter()
    ego = ego_mask(points, geometry, margin) if geometry is not None else np.zeros(len(points), bool)
    road = road_mask(points, sensor_pose, area) & ~ego
    t1 = time.perf_counter()
    ground = np.zeros(len(points), bool)
    ground[road] = patchwise_ground_mask(points[road])
    t2 = time.perf_counter()
    kept = road & ~ground
    bev = bev_from_points(points[kept], BEV_RES, (-BEV_HALF, BEV_HALF, -BEV_HALF, BEV_HALF))
    t3 = time.perf_counter()
    return ego, road, ground, kept, bev, (1000 * (t1 - t0), 1000 * (t2 - t1), 1000 * (t3 - t2))


def score_scan(points, sensor_pose, own_id, vehicles, area, raster, geometry=None, margin=DEFAULT_MARGIN,
               repeats=1, lidar_range=120.0):
    times = []
    for _ in range(max(1, repeats)):   # identical output each time; the fastest run is the least disturbed
        ego, road, ground, kept, bev, t = process_scan(points, sensor_pose, area, geometry, margin)
        times.append(t)
    ms_road, ms_ground, ms_bev = np.min(np.array(times), axis=0)
    seed = estimate_seed_plane(points[road])
    quality = [s["quality"] for s in sector_metadata(points, points[kept], SECTORS, sensor_pose, lidar_range)]

    world = sensor_to_world(points[:, :3].astype(np.float64), sensor_pose)
    near_vehicle = np.zeros(len(points), bool)
    body = np.zeros(len(points), bool)
    own = np.zeros(len(points), bool)
    body_kept = {}
    for v in vehicles:
        if v["id"] == own_id:
            own |= _in_box(world, v, z_min=-1.0, margin=0.2)
            continue
        near_vehicle |= _in_box(world, v, z_min=-1.0, margin=0.5)
        inside = _in_box(world, v, z_min=0.3)
        body |= inside
        body_kept[v["id"]] = int((inside & kept).sum())
    gt_ground = raster.is_ground(world[:, 0], world[:, 1], world[:, 2]) & ~near_vehicle & ~own
    on_slope = ((world[:, 0] >= SLOPE[0]) & (world[:, 0] <= SLOPE[1]) & (world[:, 1] >= SLOPE[2])
                & (world[:, 1] <= SLOPE[3]) & (world[:, 2] > 0.5))
    cells = bev_cells(points[kept][:, :2])
    road_cells = bev_cells(points[road][:, :2])
    cov = coverage(sensor_pose, vehicles, own_id, body_kept)
    in50 = np.hypot(points[:, 0], points[:, 1]) < COMMON
    kept_cells_50 = bev_cells(points[kept & in50][:, :2])
    own_kept_cells = np.unique(cells[own[kept] & (cells >= 0)])
    return {
        "points": int(len(points)), "kept": int(kept.sum()),
        "ground": int(gt_ground.sum()), "ground_left": int((gt_ground & kept).sum()),
        "body": int(body.sum()), "body_lost": int((body & ~kept).sum()),
        "ground_slope": int((gt_ground & on_slope).sum()), "ground_left_slope": int((gt_ground & on_slope & kept).sum()),
        "body_slope": int((body & on_slope).sum()), "body_lost_slope": int((body & on_slope & ~kept).sum()),
        "bev_cells": int(bev.occupancy.sum()), "bev_false_cells": false_cells(cells, gt_ground[kept]),
        "observed_cells": int(len(np.unique(road_cells[road_cells >= 0]))),
        "own_hits": int(own.sum()), "own_left": int((own & kept).sum()), "ego_cells": int(len(own_kept_cells)),
        "ego_removed": int(ego.sum()), "ego_removed_vehicle": int((ego & body).sum()),
        "ground_50": int((gt_ground & in50).sum()), "ground_left_50": int((gt_ground & in50 & kept).sum()),
        "body_50": int((body & in50).sum()), "body_lost_50": int((body & in50 & ~kept).sum()),
        "bev_false_cells_50": false_cells(kept_cells_50, gt_ground[kept & in50]),
        "cov50": cov["within50"][0], "tot50": cov["within50"][1],
        "quality_sum": float(sum(quality)),
        "near_cov": cov["near"][0], "near_tot": cov["near"][1], "far_cov": cov["far"][0], "far_tot": cov["far"][1],
        "est_height": float(-seed[2]) if seed is not None else float("nan"),
        "true_height": float(sensor_pose["z"] - _ground_under(vehicles, own_id)),
        "ms_road": float(ms_road), "ms_ground": float(ms_ground), "ms_bev": float(ms_bev),
        # cooperative view of the frame (dropped before caching)
        "_body_kept": body_kept, "_kept_xy": world[kept][:, :2], "_kept_ground": gt_ground[kept],
        "_road_xy": world[road][:, :2],
    }


def _ground_under(vehicles, own_id):
    """z of the road under the CAV: its box bottom (actor location z for CARLA vehicles)."""
    v = next((v for v in vehicles if v["id"] == own_id), None)
    return v["pose"]["z"] if v else 0.0


def coop_frame(frame_scans, vehicles):
    """Union of the frame's CAV clouds: vehicles seen by the CAVs together, and world BEV cells."""
    kept_by = {}
    for s in frame_scans:
        for vid, n in s["_body_kept"].items():
            kept_by[vid] = kept_by.get(vid, 0) + n
    targets = [v for v in vehicles
               if np.hypot(v["pose"]["x"] - COOP_CENTRE[0], v["pose"]["y"] - COOP_CENTRE[1]) < COOP_RADIUS]
    covered = sum(int(kept_by.get(v["id"], 0) >= MIN_BODY_POINTS) for v in targets)
    xy = np.concatenate([s["_kept_xy"] for s in frame_scans])
    g = np.concatenate([s["_kept_ground"] for s in frame_scans])
    inside = np.hypot(xy[:, 0] - COOP_CENTRE[0], xy[:, 1] - COOP_CENTRE[1]) < COOP_RADIUS
    cells = _world_cells(xy[inside])
    occupied = np.unique(cells)
    real = np.unique(cells[~g[inside]])
    road = np.concatenate([s["_road_xy"] for s in frame_scans])
    road = road[np.hypot(road[:, 0] - COOP_CENTRE[0], road[:, 1] - COOP_CENTRE[1]) < COOP_RADIUS]
    return {"coop_targets": len(targets), "coop_covered": covered, "coop_cells": int(len(occupied)),
            "coop_false_cells": int(len(np.setdiff1d(occupied, real))),
            "coop_observed_cells": int(len(np.unique(_world_cells(road))))}


def _world_cells(xy):
    ij = np.floor(xy / BEV_RES).astype(np.int64) + 10 ** 6
    return ij[:, 0] * 4 * 10 ** 6 + ij[:, 1]


def run_geometry(run):
    import yaml
    with open(os.path.join(run, "config.yaml"), encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    return cfg, geometry_for(cfg["scenario"]["cav_blueprint"], cfg["lidar"])


def score_run(run, area, raster, use_ego_mask=True, margin=DEFAULT_MARGIN, repeats=1):
    cfg, geometry = run_geometry(run)
    lidar_range = float(cfg["lidar"]["range"])
    scans, coop = [], []
    for meta_path in sorted(glob.glob(os.path.join(run, "frames", "*", "meta.json"))):
        with open(meta_path, encoding="utf-8") as fh:
            meta = json.load(fh)
        folder = os.path.dirname(meta_path)
        frame_scans = []
        for cav, info in sorted(meta["cavs"].items()):
            pts = np.load(os.path.join(folder, cav + ".npy"))
            s = score_scan(pts, info["sensor_pose"], info["vehicle_id"], meta["vehicles"], area, raster,
                           geometry if use_ego_mask else None, margin, repeats, lidar_range)
            s.update(cav=cav, frame=int(meta["frame"]))
            frame_scans.append(s)
        coop.append(coop_frame(frame_scans, meta["vehicles"]))
        for s in frame_scans:
            scans.append({k: v for k, v in s.items() if not k.startswith("_")})
    return scans, coop, geometry


def pct(a, b):
    return 100.0 * a / b if b else float("nan")


def summarise(scans, coop=None):
    tot = {k: sum(s[k] for s in scans) for k in scans[0] if isinstance(scans[0][k], int) and k != "frame"}
    pts = np.array([s["points"] for s in scans], float)
    ms = np.array([s["ms_road"] + s["ms_ground"] + s["ms_bev"] for s in scans])
    out = {
        "scans": len(scans),
        "points": float(pts.mean()), "points_std": float(pts.std()),
        "kept": tot["kept"] / len(scans),
        "ground_left": pct(tot["ground_left"], tot["ground"]),
        "vehicle_loss": pct(tot["body_lost"], tot["body"]),
        "ground_left_slope": pct(tot["ground_left_slope"], tot["ground_slope"]),
        "vehicle_loss_slope": pct(tot["body_lost_slope"], tot["body_slope"]),
        "body_points": tot["body"] / len(scans),
        "bev_cells": tot["bev_cells"] / len(scans), "bev_false_cells": tot["bev_false_cells"] / len(scans),
        "observed_cells": tot["observed_cells"] / len(scans),
        "coverage": pct(tot["near_cov"] + tot["far_cov"], tot["near_tot"] + tot["far_tot"]),
        "coverage_near": pct(tot["near_cov"], tot["near_tot"]), "coverage_far": pct(tot["far_cov"], tot["far_tot"]),
        "vehicles_in_window": (tot["near_tot"] + tot["far_tot"]) / len(scans),
        "own_hits": tot["own_hits"] / len(scans), "own_left": tot["own_left"] / len(scans),
        "ego_cells": tot["ego_cells"] / len(scans), "ego_removed": tot["ego_removed"] / len(scans),
        "ego_removed_vehicle": tot["ego_removed_vehicle"],
        "quality": float(np.mean([s["quality_sum"] for s in scans])) / len(SECTORS),
        "ground_left_50": pct(tot["ground_left_50"], tot["ground_50"]),
        "vehicle_loss_50": pct(tot["body_lost_50"], tot["body_50"]),
        "bev_false_cells_50": tot["bev_false_cells_50"] / len(scans),
        "coverage_50": pct(tot["cov50"], tot["tot50"]),
        "height_error_cm": 100 * float(np.nanmean([abs(s["est_height"] - s["true_height"]) for s in scans])),
        "est_height": float(np.nanmean([s["est_height"] for s in scans])),
        "ms": float(np.median(ms)), "ms_p25": float(np.percentile(ms, 25)), "ms_p75": float(np.percentile(ms, 75)),
        "ms_ground": float(np.median([s["ms_ground"] for s in scans])),
    }
    if coop:
        out.update({
            "coop_coverage": pct(sum(c["coop_covered"] for c in coop), sum(c["coop_targets"] for c in coop)),
            "coop_targets": float(np.mean([c["coop_targets"] for c in coop])),
            "coop_cells": float(np.mean([c["coop_cells"] for c in coop])),
            "coop_false_cells": float(np.mean([c["coop_false_cells"] for c in coop])),
            "coop_observed_cells": float(np.mean([c["coop_observed_cells"] for c in coop])),
        })
    return out


def per_cav(scans):
    """Summary of each CAV's own scans: compares the same CAV across population settings."""
    return {cav: summarise([s for s in scans if s["cav"] == cav]) for cav in sorted({s["cav"] for s in scans})}


# --------------------------------------------------------------------------------------------------------------
# scoring with a cache

def add_derived(row):
    """False cells per 1000 observed road cells (per CAV and for the union), from the run means."""
    if row.get("observed_cells"):
        row["false_per_1000_observed"] = 1000.0 * row["bev_false_cells"] / row["observed_cells"]
    if row.get("coop_observed_cells"):
        row["coop_false_per_1000_observed"] = 1000.0 * row["coop_false_cells"] / row["coop_observed_cells"]
    return row


def score_cached(name, run, ctx, args):
    path = os.path.join(args.out, "runs", name + ".json")
    if not args.rescore and os.path.exists(path):
        with open(path, encoding="utf-8") as fh:
            cached = json.load(fh)
        if cached.get("run") == os.path.basename(run) and cached.get("eval_version") == EVAL_VERSION \
                and cached.get("ego_mask") == args.ego_mask and cached.get("timing_repeats") == args.timing_repeats:
            return add_derived(cached)
    scans, coop, geometry = score_run(run, ctx["area"], ctx["raster"], args.ego_mask == "on",
                                      repeats=args.timing_repeats)
    row = summarise(scans, coop)
    row.update(run=os.path.basename(run), eval_version=EVAL_VERSION, ego_mask=args.ego_mask,
               timing_repeats=args.timing_repeats, mount_valid=bool(geometry.mount_valid),
               roof_clearance_m=round(float(geometry.roof_clearance), 3), per_cav=per_cav(scans))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(row, fh, indent=1)
    print("[scored] {} ({} scans{})".format(name, len(scans), "" if geometry.mount_valid else ", INVALID MOUNT"),
          flush=True)
    return add_derived(row)


def collect(scenes, ctx, args):
    """results[scene] = {terrain, kind, factors: {factor: [rows]}, checks: {check: row}}; excluded runs."""
    results, excluded = {}, []
    for scene in scenes:
        info = SCENES[scene]
        population = info["kind"] == "population"
        table, base = (POPULATION_FACTORS, POPULATION_BASELINE) if population else (FACTORS, BASELINE)
        entry = {"terrain": info["terrain"], "kind": info["kind"], "factors": {}, "checks": {}}
        for factor, (key, values) in table.items():
            rows = []
            for value in values:
                name = run_name(scene, factor, value)
                run = find_run(name)
                if run is None:
                    continue
                row = dict(score_cached(name, run, ctx, args), value=value, baseline=value == base[factor], name=name)
                if not row["mount_valid"]:
                    excluded.append({"name": name, "reason": "LiDAR mount inside the car body"})
                    continue
                rows.append(row)
            if len(rows) > 1 or (rows and len(values) == 1):   # a factor needs its baseline and one other value
                entry["factors"][factor] = rows
        for check in CHECKS:
            name = run_name(scene, check, None)
            run = find_run(name)
            if run is not None:
                row = dict(score_cached(name, run, ctx, args), name=name)
                entry["checks"][check] = row
                if not row["mount_valid"]:
                    excluded.append({"name": name, "reason": "LiDAR mount inside the car body"})
        results[scene] = entry
    return results, excluded


# --------------------------------------------------------------------------------------------------------------
# aggregation across scenes

AGG_METRICS = ["points", "kept", "ego_removed", "ground_left", "vehicle_loss", "bev_cells", "bev_false_cells",
               "observed_cells", "coop_observed_cells",
               "coverage", "coverage_near", "coverage_far", "quality", "ms", "ground_left_50", "vehicle_loss_50",
               "bev_false_cells_50", "coverage_50", "ground_left_slope", "vehicle_loss_slope", "coop_coverage",
               "coop_cells", "coop_false_cells", "false_per_1000_observed", "coop_false_per_1000_observed"]


def _sd(values):
    return float(np.std(values, ddof=1)) if len(values) > 1 else float("nan")


def aggregate(results):
    """{group: {scenes, factors: {factor: [{value, n, scenes, mean, sd, diff_mean, diff_sd, per_scene}]}}}."""
    groups = {}
    for scene, entry in results.items():
        groups.setdefault(entry["terrain"], []).append(scene)
    out = {}
    for group, scenes in groups.items():
        factors = {}
        for factor in sorted({f for s in scenes for f in results[s]["factors"]}, key=FACTOR_ORDER.index):
            values = sorted({r["value"] for s in scenes for r in results[s]["factors"].get(factor, [])})
            rows = []
            for value in values:
                per_scene, diffs, is_base = {}, {}, True
                for s in scenes:
                    rows_s = results[s]["factors"].get(factor, [])
                    r = next((x for x in rows_s if x["value"] == value), None)
                    b = next((x for x in rows_s if x["baseline"]), None)
                    if r is None or b is None:
                        continue
                    is_base &= r["baseline"]
                    per_scene[s] = {m: r[m] for m in AGG_METRICS if m in r}
                    diffs[s] = {m: r[m] - b[m] for m in AGG_METRICS if m in r and m in b}
                if not per_scene:
                    continue
                metrics = [m for m in AGG_METRICS if all(m in p for p in per_scene.values())]
                rows.append({
                    "value": value, "n": len(per_scene), "scenes": sorted(per_scene), "baseline": is_base,
                    "mean": {m: float(np.nanmean([p[m] for p in per_scene.values()])) for m in metrics},
                    "sd": {m: _sd([p[m] for p in per_scene.values()]) for m in metrics},
                    "diff_mean": {m: float(np.nanmean([d[m] for d in diffs.values()])) for m in metrics},
                    "diff_sd": {m: _sd([d[m] for d in diffs.values()]) for m in metrics},
                    # scenes whose change has the sign of the mean change (consistency across scenes)
                    "diff_same_sign": {m: int(sum(np.sign(d[m]) == np.sign(np.nanmean([x[m] for x in diffs.values()]))
                                                  and d[m] != 0 for d in diffs.values())) for m in metrics},
                    "diffs": diffs,
                    "per_scene": per_scene,
                })
            factors[factor] = rows
        out[group] = {"scenes": scenes, "factors": factors}
    return out


# --------------------------------------------------------------------------------------------------------------
# processing equivalence check

def check_processing(names, ctx, out):
    """Run the project's processing CLI on recorded runs and compare its non-ground output with process_scan."""
    from cavlidar.backend.processing.__main__ import process_frame
    report = {}
    for name in names:
        run = find_run(name)
        if run is None:
            print("[missing] {}".format(name))
            continue
        _, geometry = run_geometry(run)
        same = total = 0
        flags = set()
        for meta_path in sorted(glob.glob(os.path.join(run, "frames", "*", "meta.json"))):
            with open(meta_path, encoding="utf-8") as fh:
                meta = json.load(fh)
            out_dir = process_frame(run, frame=int(meta["frame"]), preview=False)
            with open(os.path.join(str(out_dir), "summary.json"), encoding="utf-8") as fh:
                summary = json.load(fh)
            for cav, info in sorted(meta["cavs"].items()):
                pts = np.load(os.path.join(os.path.dirname(meta_path), cav + ".npy"))
                kept = process_scan(pts, info["sensor_pose"], ctx["area"], geometry)[3]
                cli = np.load(os.path.join(str(out_dir), cav + "_nonground.npy"))
                same += int(cli.shape == pts[kept].shape and np.array_equal(cli, pts[kept]))
                total += 1
                flags.add(bool(summary["cavs"][cav]["ego_geometry"]["mount_valid"]))
        report[name] = {"run": os.path.basename(run), "scans": total, "identical_nonground": same,
                        "cli_mount_valid": sorted(flags), "eval_mount_valid": bool(geometry.mount_valid)}
        print("[processing] {}: {} of {} scans identical to the CLI, mount_valid CLI {} / eval {}".format(
            name, same, total, sorted(flags), geometry.mount_valid))
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "processing_check.json"), "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    return report


def label_views(run, min_points=50):
    """
    Ground-truth box vs returns for every (frame, CAV, other vehicle with >= min_points returns) of a
    recorded run, without labels: returns are attributed by the box (tools/label_alignment.near_box,
    1.5 m beyond its ends), the offset is read at the end of the box that faces the sensor
    (label_alignment.face_offset, 2 % quantile against stray returns). The estimator of the first
    version (median of the shifts holding the most returns, 2.5 m search) is kept per view as tie_shift.
    """
    import label_alignment as la
    rows = []
    for meta_path in sorted(glob.glob(os.path.join(run, "frames", "*", "meta.json"))):
        with open(meta_path, encoding="utf-8") as fh:
            meta = json.load(fh)
        by_id = {v["id"]: v for v in meta["vehicles"]}
        for cav, info in sorted(meta["cavs"].items()):
            obs = by_id[info["vehicle_id"]]
            pts = np.load(os.path.join(os.path.dirname(meta_path), cav + ".npy"))
            world = sensor_to_world(pts[:, :3].astype(np.float64), info["sensor_pose"])
            s = info["sensor_pose"]
            others = [v for v in meta["vehicles"] if v["id"] != obs["id"]]
            for v in others:
                near = la.near_box(world, v, others)
                if near.sum() < min_points:
                    continue
                view = la.view_of((s["x"], s["y"], s["z"]), v)
                a = la.box_frame(world[near], v)[0]
                old = la.near_box(world, v, others, along=2.5, across=2.5)
                speed = v.get("speed_mps", 0.0)
                rows.append({"frame": meta["frame"], "cav": cav, "id": v["id"], "model": v.get("type_id"),
                             "view": view, "points": int(near.sum()), "speed": speed,
                             "offset": la.face_offset(a, v["extent"][0], view, q=0.02),
                             "tie_shift": la.tie_shift(world[old], v)[0] if old.sum() else float("nan"),
                             "observer_speed": obs.get("speed_mps", 0.0)})
    return rows


def check_labels(names, out, min_points=50):
    """
    Are the ground-truth boxes where the LiDAR sees the vehicles? Per run and viewing side (sensor behind,
    beside or ahead of the vehicle): median offset at the visible end for moving (>= 1 m/s) and stationary
    vehicles, the share within 0.15 m, the offset a box one tick late would give (-median speed x 0.05 s),
    and the first estimator's median (biased: about -1 m from behind and +1 m from ahead whatever the
    alignment, tools/label_alignment.py). The reference for "aligned" is the stationary sampling gap of
    the semantic-LiDAR diagnostic (tools/diagnose_label_alignment.py): ~+0.03 m rear, ~-0.03 m front.
    """
    import label_alignment as la
    os.makedirs(out, exist_ok=True)
    report = {}
    for name in names:
        run = find_run(name)
        if run is None:
            print("[missing] {}".format(name))
            continue
        rows = label_views(run, min_points)
        entry = {"run": os.path.basename(run), "views": len(rows), "by_view": {}}
        for state, test in (("moving", lambda r: r["speed"] >= 1.0), ("stationary", lambda r: r["speed"] < 0.1)):
            for view in la.VIEWS:
                sel = [r for r in rows if test(r) and r["view"] == view]
                if not sel:
                    continue
                off = np.array([r["offset"] for r in sel])
                ties = np.array([r["tie_shift"] for r in sel])
                speed = float(np.median([r["speed"] for r in sel]))
                entry["by_view"]["{} {}".format(state, view)] = {
                    "views": len(sel), "speed_median": speed,
                    "offset_median": float(np.median(off)),
                    "offset_p05_p95": [float(x) for x in np.percentile(off, [5, 95])],
                    "share_within_0.15m": float(np.mean(np.abs(off) <= 0.15)),
                    "one_tick_late_would_give": -0.05 * speed,
                    "first_estimator_median": float(np.nanmedian(ties))}
        moving = [r for r in rows if r["speed"] >= 1.0]
        entry["moving_views"] = len(moving)
        if moving:
            off = np.array([r["offset"] for r in moving])
            entry["moving_share_within_0.15m"] = float(np.mean(np.abs(off) <= 0.15))
            entry["moving_first_estimator_share_within_0.15m"] = float(
                np.mean(np.abs([r["tie_shift"] for r in moving]) <= 0.15))
        report[name] = entry
        print("[labels] {}: {} views ({} moving); {}".format(name, len(rows), len(moving), "; ".join(
            "{} n={} offset {:+.2f} m (first estimator {:+.2f}, one tick late {:+.2f})".format(
                k, x["views"], x["offset_median"], x["first_estimator_median"], x["one_tick_late_would_give"])
            for k, x in entry["by_view"].items() if k.startswith("moving"))))
        with open(os.path.join(out, "labels_views_{}.json".format(name)), "w", encoding="utf-8") as fh:
            json.dump(rows, fh)
    with open(os.path.join(out, "labels_check.json"), "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    return report


# --------------------------------------------------------------------------------------------------------------
# report

COLUMNS = [("points", "points/scan", "{:.0f}"), ("kept", "kept/scan", "{:.0f}"),
           ("ego_removed", "ego removed", "{:.0f}"), ("ground_left", "ground left %", "{:.2f}"),
           ("vehicle_loss", "vehicle loss %", "{:.2f}"), ("bev_cells", "BEV cells", "{:.0f}"),
           ("bev_false_cells", "false cells", "{:.1f}"), ("observed_cells", "observed cells", "{:.0f}"),
           ("coverage", "coverage %", "{:.1f}"), ("coverage_far", "far cov. %", "{:.1f}"), ("est_height", "est. height m", "{:.2f}"),
           ("own_left", "own pts left", "{:.0f}"), ("quality", "mean S~", "{:.4f}"), ("ms", "ms/scan", "{:.0f}")]
RANGE_COLUMNS = [("ground_left_50", "ground left % <50 m", "{:.2f}"),
                 ("vehicle_loss_50", "vehicle loss % <50 m", "{:.2f}"),
                 ("bev_false_cells_50", "false cells <50 m", "{:.1f}"),
                 ("coverage_50", "coverage % <50 m", "{:.1f}")]
SLOPE_COLUMNS = [("ground_left_slope", "slope ground left %", "{:.2f}"),
                 ("vehicle_loss_slope", "slope vehicle loss %", "{:.2f}")]
COOP_COLUMNS = [("coop_coverage", "coop coverage %", "{:.1f}"), ("coop_observed_cells", "coop observed", "{:.0f}"),
                ("coop_cells", "coop cells", "{:.0f}"), ("coop_false_cells", "coop false cells", "{:.1f}"),
                ("coop_false_per_1000_observed", "coop false / 1000 observed", "{:.1f}")]
EFFECT_COLUMNS = [("kept", "kept/scan", "{:.0f}"), ("ground_left", "ground left %", "{:.2f}"),
                  ("vehicle_loss", "vehicle loss %", "{:.2f}"), ("bev_false_cells", "false cells", "{:.1f}"),
                  ("observed_cells", "observed cells", "{:.0f}"),
                  ("false_per_1000_observed", "false / 1000 observed", "{:.1f}"), ("coverage", "coverage %", "{:.1f}"),
                  ("ms", "ms/scan", "{:.0f}")]
COOP_EFFECT_COLUMNS = [("coop_coverage", "coop coverage %", "{:.1f}"),
                       ("coop_observed_cells", "coop observed", "{:.0f}"),
                       ("coop_false_cells", "coop false cells", "{:.1f}"),
                       ("coop_false_per_1000_observed", "coop false / 1000 observed", "{:.1f}")]
AGG_COLUMNS = [("points", "points/scan", "{:.0f}"), ("kept", "kept/scan", "{:.0f}"),
               ("ground_left", "ground left %", "{:.2f}"), ("vehicle_loss", "vehicle loss %", "{:.2f}"),
               ("bev_false_cells", "false cells", "{:.1f}"), ("observed_cells", "observed cells", "{:.0f}"),
               ("coverage", "coverage %", "{:.1f}"), ("ms", "ms/scan", "{:.0f}")]


def _label(factor, value, baseline):
    text = "{:g}M".format(value / 1e6) if factor == "pps" else "{:g}".format(value)
    return text + (" (base)" if baseline else "")


def report_lines(results, agg, excluded, args):
    lines = ["# LiDAR sensitivity benchmark (ego mask {})".format(args.ego_mask), "",
             "One factor changed per run, parked scenes; sensor baseline = 32 ch, 120 m, 1 000 000 pts/s, 20 Hz,",
             "noise 0, mount 2.4 m, drop-off 0.45. Ground truth used for scoring only. Aggregates: mean +- sample sd",
             "across scenes (n = scenes); delta = mean paired difference to each scene's own baseline.", ""]
    for group, g in agg.items():
        lines += ["## {} scenes: {}".format(group, ", ".join(g["scenes"])), ""]
        cols = AGG_COLUMNS + (COOP_COLUMNS if group == "junction" else [])
        for factor, rows in g["factors"].items():
            extra = RANGE_COLUMNS if factor == "range" else []
            c = [x for x in cols + extra if all(x[0] in r["mean"] for r in rows)]
            lines += ["### {} — {}".format(group, factor), "",
                      "| value | n | " + " | ".join(h for _, h, _ in c) + " |", "|" + "---|" * (len(c) + 2)]
            for r in rows:
                cells = []
                for k, _, f in c:
                    cell = f.format(r["mean"][k])
                    if r["n"] > 1:
                        cell += " ± " + f.format(r["sd"][k])
                    if not r["baseline"]:
                        cell += " (Δ {}{})".format("+" if r["diff_mean"][k] >= 0 else "", f.format(r["diff_mean"][k]))
                    cells.append(cell)
                lines.append("| {} | {} | {} |".format(_label(factor, r["value"], r["baseline"]), r["n"],
                                                      " | ".join(cells)))
            lines.append("")
    lines += ["## Effects: paired change to each scene's own baseline", "",
              "mean change ± sd across scenes [scenes with a change of the same sign / scenes]", ""]
    for group, g in agg.items():
        cols = EFFECT_COLUMNS + (COOP_EFFECT_COLUMNS if group == "junction" else [])
        lines += ["### {} ({} scenes)".format(group, len(g["scenes"])), "",
                  "| factor | value | " + " | ".join(h for _, h, _ in cols) + " |", "|" + "---|" * (len(cols) + 2)]
        for factor, rows in g["factors"].items():
            for r in rows:
                if r["baseline"]:
                    continue
                cells = []
                for k, _, f in cols:
                    if k not in r["diff_mean"]:
                        cells.append("")
                        continue
                    sd = " ± " + f.format(r["diff_sd"][k]) if r["n"] > 1 else ""
                    cells.append("{}{}{} [{}/{}]".format("+" if r["diff_mean"][k] >= 0 else "",
                                                         f.format(r["diff_mean"][k]), sd,
                                                         r["diff_same_sign"][k], r["n"]))
                lines.append("| {} | {} | {} |".format(factor, _label(factor, r["value"], False), " | ".join(cells)))
        lines.append("")
    lines += ["## Per scene", ""]
    for scene, entry in results.items():
        for factor, rows in entry["factors"].items():
            cols = COLUMNS + (SLOPE_COLUMNS if entry["terrain"] == "slope" else []) + \
                (RANGE_COLUMNS if factor == "range" else []) + (COOP_COLUMNS if entry["kind"] == "population" else [])
            lines += ["### {} — {}".format(scene, factor), "",
                      "| value | " + " | ".join(h for _, h, _ in cols) + " |", "|" + "---|" * (len(cols) + 1)]
            for r in rows:
                lines.append("| {} | {} |".format(_label(factor, r["value"], r["baseline"]),
                                                  " | ".join(f.format(r[k]) for k, _, f in cols)))
            lines.append("")
    checks = [(s, c, r) for s, e in results.items() for c, r in e["checks"].items()]
    if checks or excluded:
        lines += ["## Protocol checks", ""]
        for s, c, r in checks:
            lines.append("- {} {}: {}, mount valid {}, points/scan {:.0f}, ground left {:.2f} %, "
                         "false cells {:.1f}".format(s, c, r["run"], r["mount_valid"], r["points"], r["ground_left"],
                                                     r["bev_false_cells"]))
        for e in excluded:
            lines.append("- excluded: {} ({})".format(e["name"], e["reason"]))
        lines.append("")
    return lines


PLOT_ROWS = [("kept", "kept points / scan"), ("ground_left", "ground left in cloud (%)"),
             ("vehicle_loss", "vehicle points lost (%)"), ("bev_false_cells", "BEV false cells / scan"),
             ("coverage", "vehicles represented (%)")]
TERRAIN_STYLE = {"flat": ("#2a78d6", "o"), "slope": ("#eb6834", "s"), "junction": ("#2a78d6", "o"),
                 "moving": ("#2a78d6", "o")}
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
PERCENT_METRICS = {"ground_left", "vehicle_loss", "coverage", "coop_coverage", "coverage_near", "coverage_far"}
UNITS = {"noise": "noise (m)", "height": "mount height (m)", "channels": "channels", "range": "range (m)",
         "dropoff": "drop-off rate", "pps": "points/s (M)", "cavs": "CAVs", "traffic": "traffic vehicles"}


def plot_results(agg, path, factors, rows_spec, groups):
    """Small multiples: one column per factor, one row per metric; per group the mean across scenes
    (bars = sd) as a thick line and each scene as a thin line."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    factors = [f for f in factors if any(agg.get(g, {}).get("factors", {}).get(f) for g in groups)]
    if not factors:
        return
    fig, axes = plt.subplots(len(rows_spec), len(factors), figsize=(3.2 * len(factors), 2.3 * len(rows_spec)),
                             squeeze=False, facecolor=SURFACE)
    for c, factor in enumerate(factors):
        scale = 1e-6 if factor == "pps" else 1.0
        for r, (metric, label) in enumerate(rows_spec):
            ax = axes[r][c]
            ax.set_facecolor(SURFACE)
            for group in groups:
                rows = [x for x in agg.get(group, {}).get("factors", {}).get(factor, []) if metric in x["mean"]]
                if not rows:
                    continue
                colour, marker = TERRAIN_STYLE[group]
                xs = [x["value"] * scale for x in rows]
                base = next((x["value"] * scale for x in rows if x["baseline"]), None)
                if base is not None:
                    ax.axvline(base, color=GRID, lw=1.5, ls="--", zorder=0)
                for scene in agg[group]["scenes"]:
                    pts = [(x["value"] * scale, x["per_scene"][scene][metric]) for x in rows if scene in x["per_scene"]]
                    if len(pts) > 1:
                        ax.plot(*zip(*pts), color=colour, lw=1, alpha=0.3, zorder=2)
                n = max(x["n"] for x in rows)
                ys = [x["mean"][metric] for x in rows]
                sds = [x["sd"][metric] if x["n"] > 1 else 0.0 for x in rows]
                top = 100.0 if metric in PERCENT_METRICS else float("inf")   # a share cannot exceed 100 %
                ax.errorbar(xs, ys, yerr=[sds, [min(sd, top - y) for y, sd in zip(ys, sds)]],
                            color=colour, lw=2, marker=marker, ms=6, mec=SURFACE, mew=1.2, capsize=2.5,
                            label="{} (up to {} scenes)".format(group, n), zorder=3)
                ax.set_xticks(xs)
                ax.set_xticklabels(["{:g}".format(v) for v in xs], fontsize=8, color=INK2)
            ax.tick_params(axis="y", labelsize=8, colors=INK2)
            ax.grid(axis="y", color=GRID, lw=0.8)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            for side in ("left", "bottom"):
                ax.spines[side].set_color(GRID)
            if c == 0:
                ax.set_ylabel(label, fontsize=8.5, color=INK)
            if r == 0:
                ax.set_title(factor, fontsize=10, color=INK)
            if r == len(rows_spec) - 1:
                ax.set_xlabel(UNITS.get(factor, factor), fontsize=8.5, color=INK2)
    handles, labels = [], []
    for ax in axes.ravel():
        for h, lab in zip(*ax.get_legend_handles_labels()):
            if lab not in labels:
                handles.append(h)
                labels.append(lab)
    fig.legend(handles, labels, loc="upper center", ncol=max(1, len(labels)), fontsize=9, frameon=False,
               bbox_to_anchor=(0.5, 1.0))
    note = ("thick line = mean across scenes (bars = sd), thin lines = single scenes, "
            "dashed = baseline; one factor changed per run")
    if len(factors) < 4:
        note = note.replace("single scenes, ", "single scenes,\n")
    fig.text(0.5, 0.965 if len(factors) >= 4 else 0.955, note, ha="center", va="top" if len(factors) < 4 else "baseline",
             fontsize=8.5, color=INK2)
    fig.tight_layout(rect=(0, 0, 1, 0.95 if len(factors) >= 4 else 0.91))
    fig.savefig(path, dpi=110, facecolor=SURFACE)
    plt.close(fig)


def load_context():
    from cavlidar.api import DEFAULT_CONFIG
    from cavlidar.backend.maps import catalog
    from cavlidar.backend.maps.road_map import RoadMap
    from cavlidar.core.config import load_config
    from cavlidar.editor import settings
    info = catalog.find_map(settings.available_maps(load_config(DEFAULT_CONFIG)), "Town03")
    return {"area": load_drivable_area(info.name, info.xodr_path, settings.MAPS_CACHE),
            "raster": HeightRaster(RoadMap.from_file(info.name, info.xodr_path))}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenes", nargs="+", default=list(SCENES), choices=list(SCENES))
    parser.add_argument("--out", default=os.path.join(ROOT, "data", "ablation", "benchmark"))
    parser.add_argument("--ego-mask", choices=["on", "off"], default="on")
    parser.add_argument("--timing-repeats", type=int, default=1,
                        help="process every scan N times and keep the fastest (timing only; outputs are identical)")
    parser.add_argument("--rescore", action="store_true", help="ignore the cached scores")
    parser.add_argument("--check-processing", nargs="+", metavar="RUN", default=None,
                        help="compare the processing CLI with this evaluation on these run names, then stop")
    parser.add_argument("--check-labels", nargs="+", metavar="RUN", default=None,
                        help="check that the ground-truth boxes match the moving vehicles in these runs, then stop")
    args = parser.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    if args.check_labels:
        check_labels(args.check_labels, args.out)
        return
    ctx = load_context()
    if args.check_processing:
        check_processing(args.check_processing, ctx, args.out)
        return

    results, excluded = collect(args.scenes, ctx, args)
    agg = aggregate(results)
    with open(os.path.join(args.out, "results.json"), "w", encoding="utf-8") as fh:
        json.dump({"settings": {"ego_mask": args.ego_mask, "timing_repeats": args.timing_repeats,
                                "eval_version": EVAL_VERSION, "scenes": {s: SCENES[s] for s in args.scenes}},
                   "scenes": results, "aggregate": agg, "excluded": excluded}, fh, indent=1)
    lines = report_lines(results, agg, excluded, args)
    with open(os.path.join(args.out, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    plot_results(agg, os.path.join(args.out, "sensitivity.png"), list(FACTORS), PLOT_ROWS, ["flat", "slope"])
    plot_results(agg, os.path.join(args.out, "population.png"), list(POPULATION_FACTORS),
                 [("coop_coverage", "vehicles seen by the CAVs (%)"), ("coop_observed_cells", "road cells observed"),
                  ("coop_false_cells", "union false cells"),
                  ("coop_false_per_1000_observed", "union false cells\nper 1000 observed"),
                  ("coverage", "per-CAV coverage (%)")], ["junction"])
    plot_results(agg, os.path.join(args.out, "moving.png"), list(FACTORS), PLOT_ROWS, ["moving"])
    print("\n".join(lines))


if __name__ == "__main__":
    main()
