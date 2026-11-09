"""
Record the LiDAR sensitivity benchmark (one factor changed per run) on fixed, parked scenes.

    python tools/run_ablation.py --scenes flat2 slope2 --factors dropoff --checks repeat invalid_mount
    python tools/run_ablation.py --scenes pop1 --factors cavs traffic
    python tools/run_ablation.py --scenes flat2 --check-only        # geometry checks, no recording
    python tools/run_ablation.py --list                              # what would be recorded
    python tools/run_ablation.py --scenes flat2 --factors height --only abl_flat2_height_1.6

Needs CARLA running Town03 (carla012 env). Scenes are listed in config/experiments/ablation/scenes.json
(tools/make_sensitivity_scenes.py adds them). Each run = scene YAML + the overrides of ONE factor
value, e.g. scene_flat2.yaml + lidar.noise_stddev=0.05 -> data/runs/<date>_abl_flat2_noise_0.05/.
The value equal to the baseline is recorded once (abl_<scene>_base) and shared by every factor;
existing runs are never re-recorded.

Sensor factors (scenes of kind "sensor", parked; and "moving", driven by the Traffic Manager): the vehicles
stay identical (for moving scenes: follow identical paths, compared frame by frame), only the LiDAR changes.
Population factors (scenes of kind "population", all Tesla Model 3):
  cavs     4 / 6 / 8 / 10 CAVs among the SAME 34 parked vehicles: only which ones carry a LiDAR changes
  traffic  4 CAVs + 20 / 30 / 40 traffic vehicles, nested: this one changes the scene and its occlusions
Moving scenes (kind "moving", Traffic Manager): only the baseline and the checks are recorded here;
their sensor factors need tools/record_moving_sensors.py, because moving traffic diverges between runs.
Protocol checks (--checks):
  repeat         the baseline recorded a second time: the scans must be identical
  invalid_mount  LiDAR at 1.2 m, inside the car body: must be flagged by the processing and
                 excluded from every comparison

After recording, every run of a scene is compared with its baseline (same vehicles and models,
largest position / rotation difference, and no vehicle moving between the sweeps of a run).
The result is saved in data/ablation/checks/<scene>.json; a failed check stops the script.
"""
import argparse
import glob
import json
import os
import sys
from collections import OrderedDict

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

REGISTRY = os.path.join(ROOT, "config", "experiments", "ablation", "scenes.json")
CHECK_DIR = os.path.join(ROOT, "data", "ablation", "checks")


def load_scenes():
    with open(REGISTRY, encoding="utf-8") as fh:
        return json.load(fh, object_pairs_hook=OrderedDict)


SCENES = load_scenes()
BASELINE = {"noise": 0.0, "height": 2.4, "channels": 32, "range": 120.0, "dropoff": 0.45, "pps": 1000000}
FACTORS = {  # sensor factors: (config key, values)
    "noise": ("lidar.noise_stddev", [0.0, 0.02, 0.05, 0.10]),
    "height": ("lidar.mount_z", [1.6, 1.9, 2.4, 2.8]),
    "channels": ("lidar.channels", [16, 32, 64]),          # points/s kept at 1 000 000
    "range": ("lidar.range", [50.0, 80.0, 120.0]),
    "dropoff": ("lidar.dropoff_general_rate", [0.0, 0.2, 0.3, 0.45, 0.6]),
    "pps": ("lidar.points_per_second", [500000, 1000000, 2000000]),
}
POPULATION_BASELINE = {"cavs": 4, "traffic": 30}
POPULATION_FACTORS = {
    "cavs": ("scenario.num_cavs", [4, 6, 8, 10]),
    "traffic": ("scenario.num_traffic", [20, 30, 40]),
}
CHECKS = {"repeat": [], "invalid_mount": ["lidar.mount_z=1.2"]}
POSITION_TOL = 0.01   # m: largest accepted position difference between runs of a parked scene
ANGLE_TOL = 0.1       # deg


def scenes_of(kind):
    return [name for name, info in SCENES.items() if info["kind"] == kind]


def factors_for(scene):
    return POPULATION_FACTORS if SCENES[scene]["kind"] == "population" else FACTORS


def baseline_for(scene):
    return POPULATION_BASELINE if SCENES[scene]["kind"] == "population" else BASELINE


def fmt(value):
    return "{:d}".format(int(value)) if float(value).is_integer() else "{:g}".format(value)


def run_name(scene, factor, value):
    if factor in CHECKS:
        return "abl_{}_{}".format(scene, factor)
    if scene in SCENES and SCENES[scene]["kind"] == "moving":   # one simulation per factor, own baseline
        return "abl_{}_{}_{}".format(scene, factor, fmt(value))
    base = POPULATION_BASELINE if factor in POPULATION_FACTORS else BASELINE
    if value == base[factor]:
        return "abl_{}_base".format(scene)
    return "abl_{}_{}_{}".format(scene, factor, fmt(value))


def overrides(scene, factor, value):
    if factor in CHECKS:
        return list(CHECKS[factor])
    if value == baseline_for(scene)[factor]:
        return []
    if factor in POPULATION_FACTORS:
        variant = SCENES[scene]["variants"]["{}{}".format(factor[0], value)]
        return ["scenario.spawn_plan={}".format(variant["spawn_plan"]),
                "scenario.num_cavs={}".format(variant["num_cavs"]),
                "scenario.num_traffic={}".format(variant["num_traffic"])]
    return ["{}={}".format(FACTORS[factor][0], value)]


def planned_runs(scene, factors, checks):
    todo = OrderedDict()
    table = factors_for(scene)
    for factor in factors:
        if factor not in table:
            continue
        for value in table[factor][1]:
            todo[run_name(scene, factor, value)] = overrides(scene, factor, value)
    for check in checks:
        todo[run_name(scene, check, None)] = overrides(scene, check, None)
    return todo


def find_run(name):
    runs = sorted(glob.glob(os.path.join(ROOT, "data", "runs", "*_" + name)))
    return runs[-1] if runs and glob.glob(os.path.join(runs[-1], "frames", "*", "meta.json")) else None


def _identity(run):
    """Vehicle name -> stable key: the pool index of population plans, else the name itself."""
    import yaml
    with open(os.path.join(run, "config.yaml"), encoding="utf-8") as fh:
        plan_path = yaml.safe_load(fh)["scenario"].get("spawn_plan")
    keys = {}
    if plan_path:
        full = plan_path if os.path.isabs(plan_path) else os.path.join(ROOT, plan_path)
        with open(full, encoding="utf-8") as fh:
            for v in json.load(fh)["vehicles"]:
                if "pool_index" in v:
                    keys["{}_{}".format(v["role"], v["index"])] = "pool_{}".format(v["pool_index"])
    return keys


def vehicle_states(run):
    """[{key: (type_id, xyz, rpy)}] for every recorded frame of a run."""
    keys = _identity(run)
    frames = []
    for meta_path in sorted(glob.glob(os.path.join(run, "frames", "*", "meta.json"))):
        with open(meta_path, encoding="utf-8") as fh:
            meta = json.load(fh)
        frames.append({keys.get(v["name"], v["name"]): (
            v["type_id"], np.array([v["pose"][k] for k in ("x", "y", "z")]),
            np.array([v["pose"][k] for k in ("roll", "pitch", "yaw")])) for v in meta["vehicles"]})
    return frames


def _angle_diff(a, b):
    return np.abs((a - b + 180.0) % 360.0 - 180.0)


def scans(run):
    for meta_path in sorted(glob.glob(os.path.join(run, "frames", "*", "meta.json"))):
        with open(meta_path, encoding="utf-8") as fh:
            meta = json.load(fh)
        for cav in sorted(meta["cavs"]):
            yield cav, np.load(os.path.join(os.path.dirname(meta_path), cav + ".npy"))


def check_scene(scene, names):
    """Compare every run of a scene with its baseline; returns the report (raises on failure)."""
    base_run = find_run("abl_{}_base".format(scene))
    base_frames = vehicle_states(base_run)
    base = base_frames[0]
    population = SCENES[scene]["kind"] == "population"
    moving = SCENES[scene]["kind"] == "moving"
    report = {"scene": scene, "baseline": os.path.basename(base_run), "runs": {}, "ok": True}
    for name in names:
        run = find_run(name)
        if run is None:
            continue
        frames = vehicle_states(run)
        first = frames[0]
        common = sorted(set(first) & set(base))
        problems = []
        import yaml
        with open(os.path.join(run, "config.yaml"), encoding="utf-8") as fh:
            sc = yaml.safe_load(fh)["scenario"]
        if len(first) != sc["num_cavs"] + sc["num_traffic"]:
            problems.append("{} vehicles spawned, {} planned".format(len(first), sc["num_cavs"] + sc["num_traffic"]))
        if not population and set(first) != set(base):
            problems.append("not the same vehicles as the baseline")
        if any(first[k][0] != base[k][0] for k in common):
            problems.append("different vehicle models")
        pairs = list(zip(frames, base_frames)) if moving else [(first, base)]   # moving: frame by frame
        if moving and len(frames) != len(base_frames):
            problems.append("{} frames, baseline {}".format(len(frames), len(base_frames)))
        pos = max(float(np.linalg.norm(f[k][1] - b[k][1])) for f, b in pairs for k in common)
        ang = max(float(_angle_diff(f[k][2], b[k][2]).max()) for f, b in pairs for k in common)
        moved = max(float(np.linalg.norm(f[k][1] - first[k][1])) for f in frames[1:] for k in first) \
            if len(frames) > 1 else 0.0
        if pos > POSITION_TOL or ang > ANGLE_TOL:
            problems.append("positions differ from the baseline")
        if moved > POSITION_TOL and not moving:
            problems.append("vehicles move between sweeps")
        entry = {"run": os.path.basename(run), "vehicles": len(first), "common_with_baseline": len(common),
                 "max_position_diff_m": pos, "max_rotation_diff_deg": ang, "max_motion_within_run_m": moved,
                 "problems": problems}
        if name.endswith("_repeat"):
            pairs = list(zip(scans(base_run), scans(run)))
            entry["identical_scans"] = sum(int(a[0] == b[0] and np.array_equal(a[1], b[1])) for a, b in pairs)
            entry["scans"] = len(pairs)
            if entry["identical_scans"] != len(pairs):
                problems.append("repeat differs from the baseline in {} of {} scans".format(
                    len(pairs) - entry["identical_scans"], len(pairs)))
        report["runs"][name] = entry
        report["ok"] &= not problems
    os.makedirs(CHECK_DIR, exist_ok=True)
    with open(os.path.join(CHECK_DIR, scene + ".json"), "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    worst_pos = max(e["max_position_diff_m"] for e in report["runs"].values())
    worst_ang = max(e["max_rotation_diff_deg"] for e in report["runs"].values())
    worst_move = max(e["max_motion_within_run_m"] for e in report["runs"].values())
    print("[check] scene {}: {} runs, largest position difference {:.4f} m, rotation {:.4f} deg, "
          "motion within a run {:.4f} m".format(scene, len(report["runs"]), worst_pos, worst_ang, worst_move))
    for name, e in report["runs"].items():
        if "identical_scans" in e:
            print("[check] {}: {} of {} scans identical to the baseline".format(name, e["identical_scans"], e["scans"]))
        for p in e["problems"]:
            print("[FAIL] {}: {}".format(name, p))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenes", nargs="+", default=list(SCENES), choices=list(SCENES))
    parser.add_argument("--factors", nargs="*", default=list(FACTORS) + list(POPULATION_FACTORS),
                        choices=list(FACTORS) + list(POPULATION_FACTORS))
    parser.add_argument("--checks", nargs="*", default=[], choices=list(CHECKS))
    parser.add_argument("--only", nargs="+", default=None, metavar="RUN",
                        help="record only these run names (e.g. abl_flat2_height_1.6); the baseline is always included")
    parser.add_argument("--check-only", action="store_true", help="only compare the existing runs")
    parser.add_argument("--list", action="store_true", help="print the runs and their overrides, record nothing")
    args = parser.parse_args(argv)
    failed = []
    for scene in args.scenes:
        todo = planned_runs(scene, [] if SCENES[scene]["kind"] == "moving" else args.factors, args.checks)
        if SCENES[scene]["kind"] == "moving" and args.factors:
            print("[note] {}: moving traffic is not reproducible between runs; record its sensor factors with "
                  "tools/record_moving_sensors.py (all values of a factor in one simulation)".format(scene))
        todo.setdefault("abl_{}_base".format(scene), [])
        todo.move_to_end("abl_{}_base".format(scene), last=False)
        if args.only:
            todo = OrderedDict((k, v) for k, v in todo.items() if k in args.only or k.endswith("_base"))
        for name, override in todo.items():
            exists = find_run(name) is not None
            if args.list:
                print("{} {:40s} {}".format("[exists]" if exists else "[todo]  ", name, override or "(baseline)"))
                continue
            if exists or args.check_only:
                continue
            from cavlidar import api
            cfg = api.load_config(os.path.join(ROOT, SCENES[scene]["config"]), overrides=override)
            print("[record] {} {}".format(name, override or "(baseline)"), flush=True)
            api.record_lidar(cfg, name=name)
        if not args.list and find_run("abl_{}_base".format(scene)):
            if not check_scene(scene, list(todo))["ok"]:
                failed.append(scene)
    if failed:
        raise SystemExit("geometry check failed for: {}".format(", ".join(failed)))


if __name__ == "__main__":
    main()
