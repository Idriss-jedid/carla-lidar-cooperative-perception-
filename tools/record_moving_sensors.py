"""
Record the sensor factors of the MOVING-traffic scenes: every value of one factor in ONE simulation.

    python tools/record_moving_sensors.py --scenes moving1 moving2            # every sensor factor
    python tools/record_moving_sensors.py --scenes moving1 --factors dropoff

Why: with moving traffic CARLA 0.9.12 is not reproducible between runs (two runs of the same
configuration diverge by up to 3.6 m, data/ablation/checks/moving*.json), so recording one run per
sensor value would mix sensor effects with different traffic. Here each CAV carries one LiDAR per
value of the factor (ray-cast LiDARs do not see each other), all fed by the same ticks: every
variant sees exactly the same vehicles at the same instants, and the factor's baseline value is
recorded in the same simulation. Every variant is saved as an ordinary run folder
(data/runs/<date>_abl_<scene>_<factor>_<value>/, same frames/ and meta.json layout as the recorder,
with the variant's own config.yaml), so tools/eval_ablation.py scores it like any other run.
Different factors are different simulations (different traffic), each with its own baseline.
A simulation counts as moving traffic only if at least 50 % of the recorded vehicle states move faster
than 1 m/s (fixed before looking at any metric); otherwise, e.g. when the traffic waits at red lights
(the light phase depends on the server clock), the factor is recorded again, at most 3 attempts.
Rejected recordings stay on disk; the newest recording of a name is the one evaluated.
Uses the project's SimulationSession and Lidar unchanged; needs CARLA running Town03 (carla012 env).
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from run_ablation import BASELINE, CHECK_DIR, FACTORS, SCENES, find_run, run_name  # noqa: E402

MIN_MOVING_SHARE = 0.5   # share of vehicle states faster than 1 m/s for a valid moving-traffic simulation
MAX_ATTEMPTS = 3


def record_factor(scene, factor, force=False):
    from cavlidar import api
    from cavlidar.backend.actors import ground_truth
    from cavlidar.backend.actors.lidar import Lidar
    from cavlidar.backend.recording import storage
    from cavlidar.backend.simulator.session import SimulationSession
    from cavlidar.core import config as _config
    from cavlidar.core.constants import CAV_CLOUD_NAME, CONFIG_FILE, META_FILE, SPAWN_FILE

    key, values = FACTORS[factor]
    names = {v: run_name(scene, factor, v) for v in values}
    if not force and all(find_run(n) for n in names.values()):
        print("[skip] {} {}: every value exists".format(scene, factor))
        return None
    config_path = os.path.join(ROOT, SCENES[scene]["config"])
    cfgs = {v: api.load_config(config_path, overrides=[] if v == BASELINE[factor] else ["{}={}".format(key, v)])
            for v in values}
    base_cfg = cfgs[BASELINE[factor]]
    dirs = {}
    for v in values:
        dirs[v] = storage.create_run_dir(base_cfg.recording.output_dir, names[v])
        _config.save_config(cfgs[v], os.path.join(dirs[v], CONFIG_FILE))
    print("[record] {} {}: {} LiDAR settings in one simulation".format(scene, factor, len(values)), flush=True)

    rec = base_cfg.recording
    with SimulationSession(base_cfg) as session:
        session.spawn_fleet()
        groups = {v: [Lidar(session.world, vehicle, cfgs[v].lidar, CAV_CLOUD_NAME.format(i))
                      for i, vehicle in enumerate(session.cavs)] for v in values}
        session.lidars = [lidar for v in values for lidar in groups[v]]   # destroyed by the session
        spawn = ground_truth.collect_vehicles(session.world, session.role_names())
        for v in values:
            storage.save_json(os.path.join(dirs[v], SPAWN_FILE), spawn)
        for _ in range(rec.warmup_ticks):
            frame = session.tick()
            for lidar in session.lidars:
                lidar.get(frame)
            session.update_camera()
        session.start_routes()
        saved, tick = 0, 0
        while saved < rec.num_frames:
            frame = session.tick()
            scans = {v: [lidar.get(frame) for lidar in groups[v]] for v in values}
            session.update_camera()
            tick += 1
            if (tick - 1) % rec.save_every:
                continue
            vehicles = ground_truth.collect_vehicles(session.world, session.role_names())
            for v in values:
                directory = storage.frame_dir(dirs[v], frame)
                for scan in scans[v]:
                    storage.save_points(directory, scan.name, scan.points, rec.formats)
                meta = {"frame": frame, "timestamp": scans[v][0].timestamp, "town": base_cfg.server.town,
                        "cavs": {scan.name: {"vehicle_id": lidar.vehicle.id, "sensor_pose": scan.sensor_pose,
                                             "mount_z": cfgs[v].lidar.mount_z, "num_points": int(len(scan.points))}
                                 for scan, lidar in zip(scans[v], groups[v])},
                        "vehicles": vehicles}
                storage.save_json(os.path.join(directory, META_FILE), meta)
            saved += 1
            speeds = [x["speed_mps"] * 3.6 for x in vehicles if x["role"] == "cav"]
            print("  frame {} saved ({}/{}), CAV speeds km/h: {}".format(
                frame, saved, rec.num_frames, ", ".join("{:.0f}".format(s) for s in speeds)), flush=True)
    return dirs


def check_factor(scene, factor):
    """Every value of a factor must hold the same vehicles at the same frames (one simulation)."""
    key, values = FACTORS[factor]
    runs = {v: find_run(run_name(scene, factor, v)) for v in values}
    metas = {}
    for v, run in runs.items():
        frames = sorted(os.listdir(os.path.join(run, "frames")))
        metas[v] = [json.load(open(os.path.join(run, "frames", f, "meta.json"), encoding="utf-8")) for f in frames]
    base = metas[BASELINE[factor]]
    worst, sensor_xy = 0.0, 0.0
    for v, frames in metas.items():
        if [m["frame"] for m in frames] != [m["frame"] for m in base]:
            raise SystemExit("{} {} {}: different frames".format(scene, factor, v))
        for m, b in zip(frames, base):
            pos = {x["name"]: np.array([x["pose"][k] for k in ("x", "y", "z")]) for x in m["vehicles"]}
            ref = {x["name"]: np.array([x["pose"][k] for k in ("x", "y", "z")]) for x in b["vehicles"]}
            worst = max(worst, max(float(np.linalg.norm(pos[k] - ref[k])) for k in ref))
            for cav in m["cavs"]:
                p, q = m["cavs"][cav]["sensor_pose"], b["cavs"][cav]["sensor_pose"]
                sensor_xy = max(sensor_xy, float(np.hypot(p["x"] - q["x"], p["y"] - q["y"])))
    speeds = [x["speed_mps"] for m in base for x in m["vehicles"]]
    report = {"scene": scene, "factor": factor, "runs": {str(v): os.path.basename(r) for v, r in runs.items()},
              "frames": [m["frame"] for m in base], "max_vehicle_position_diff_m": worst,
              "max_sensor_xy_diff_m": sensor_xy, "vehicles": len(base[0]["vehicles"]),
              "moving_vehicles_share": float(np.mean(np.array(speeds) > 1.0)),
              "median_speed_moving_mps": float(np.median([s for s in speeds if s > 1.0])) if any(
                  s > 1.0 for s in speeds) else 0.0}
    report["valid_moving"] = report["moving_vehicles_share"] >= MIN_MOVING_SHARE
    os.makedirs(CHECK_DIR, exist_ok=True)
    with open(os.path.join(CHECK_DIR, "{}_{}.json".format(scene, factor)), "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    print("[check] {} {}: {} values, vehicles identical across values (max diff {:.4f} m), sensor x/y diff "
          "{:.4f} m; {:.0f} % of vehicle states moving, median {:.1f} m/s".format(
              scene, factor, len(values), worst, sensor_xy, 100 * report["moving_vehicles_share"],
              report["median_speed_moving_mps"]))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    moving = [s for s, info in SCENES.items() if info["kind"] == "moving"]
    parser.add_argument("--scenes", nargs="+", default=moving, choices=moving)
    parser.add_argument("--factors", nargs="+", default=list(FACTORS), choices=list(FACTORS))
    args = parser.parse_args(argv)
    for scene in args.scenes:
        for factor in args.factors:
            record_factor(scene, factor)
            report = check_factor(scene, factor)
            attempts = 1
            while not report["valid_moving"] and attempts < MAX_ATTEMPTS:
                print("[retry] {} {}: only {:.0f} % of vehicle states moving".format(
                    scene, factor, 100 * report["moving_vehicles_share"]), flush=True)
                record_factor(scene, factor, force=True)
                report = check_factor(scene, factor)
                attempts += 1
            report["attempts"] = attempts
            with open(os.path.join(CHECK_DIR, "{}_{}.json".format(scene, factor)), "w", encoding="utf-8") as fh:
                json.dump(report, fh, indent=1)
            if not report["valid_moving"]:
                print("[FAIL] {} {}: traffic still not moving after {} attempts".format(scene, factor, attempts))


if __name__ == "__main__":
    main()
