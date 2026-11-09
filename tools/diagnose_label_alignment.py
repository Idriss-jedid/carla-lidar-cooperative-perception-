"""
Are the ground-truth boxes of MOVING vehicles where the LiDAR sees them? A controlled CARLA diagnostic.

    python tools/diagnose_label_alignment.py record [--cases straight traffic2 ...]   # CARLA running Town03
    python tools/diagnose_label_alignment.py analyse [--cases ...]

record: short simulations, every tick saved to data/diagnostics/label_alignment/<case>/ (an existing case
is never overwritten). Each observer carries the project's ray-cast LiDAR (Lidar class, the case's config)
and a semantic LiDAR with the same rays (sensor.lidar.ray_cast_semantic: every return carries the id of the
actor it hit, so no box is needed to tell which returns belong to which vehicle). Per tick:
  - the frame world.tick() returned, the WorldSnapshot (frame, simulation time, delta) right after the tick
    and again after every sensor was read, and every snapshot the client received (world.on_tick);
  - every vehicle's ActorSnapshot (transform, velocity, angular velocity, acceleration) and the recorder's
    ground truth (ground_truth.collect_vehicles, i.e. actor.get_transform(), what meta.json stores);
  - every measurement's frame, timestamp and sensor transform, the sensor actors' and their parents' snapshots,
    the share of ray-cast returns found bit for bit among the semantic returns (same rays, same instant);
  - semantic returns on vehicles (x, y, z, actor id) every tick, the ray-cast cloud every second tick.
Cases (fixed placements, constant-velocity targets, see CASES):
  straight       road 28 (flat, straight): targets at 0 .. 18 m/s seen from behind, beside and ahead, 4 models
  straight_dt10  the same with a 0.1 s step (LiDAR at 10 Hz): does an offset scale with the step?
  observer       a LiDAR driving at 15 m/s past parked vehicles: is the sensor pose of the right instant?
  slope          road 18 (12 deg): targets up and down the slope, observers at the top, bottom and below
  traffic1/2     the benchmark's moving1 / moving2 scenes (Traffic Manager, random models, turns)
  e12            the Milestone 3 e12 configuration (Traffic Manager, figure-2 plan)
The traffic cases are new simulations of those configurations (moving traffic is not reproducible), not
replays of the benchmark recordings. As in the benchmark, a traffic recording counts only if >= 50 % of
the vehicle states move faster than 1 m/s (else, e.g. traffic waiting at red lights, it is recorded again,
at most 3 attempts; rejected recordings are kept as <case>_rejected_<n>/).

analyse: frame/time consistency, recorder ground truth vs ActorSnapshot, sensor pose vs parent pose, ray-cast
vs semantic rays, then for every (tick t, observer, vehicle with >= MIN_POINTS semantic returns) the offset
between the returns and the box of ticks t-2 .. t+2 at the visible end (tools/label_alignment.py), against
speed, heading, yaw rate, pitch, acceleration, observer speed and the step. Writes
results/label_alignment/diagnostic.json, views_<case>.csv and figures.

Uses the project's SimulationSession, Lidar and ground_truth unchanged (carla012 env).
"""
import argparse
import csv
import glob
import json
import os
import queue
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

import label_alignment as la  # noqa: E402

OUT_DATA = os.path.join(ROOT, "data", "diagnostics", "label_alignment")
OUT_RESULTS = os.path.join(ROOT, "results", "label_alignment")
SETTLE_TICKS = 20       # vehicles drop onto the road with the brakes on
PREROLL_TICKS = 10      # constant velocity reached before the first saved tick
MIN_POINTS = 30         # semantic returns on a vehicle for a view to count
MIN_MOVING_SHARE = 0.5  # traffic cases: share of vehicle states faster than 1 m/s (rule of the benchmark)
MAX_ATTEMPTS = 3        # rejected traffic recordings are kept as <case>_rejected_<n>/
SEMANTIC_BLUEPRINT = "sensor.lidar.ray_cast_semantic"
VEHICLE_TAG = 10        # CityScapes tag 'Vehicles' in CARLA 0.9.12
SEM_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("cos", "<f4"), ("idx", "<u4"), ("tag", "<u4")])
TESLA, BMW, LINCOLN, MERCEDES = ("vehicle.tesla.model3", "vehicle.bmw.grandtourer", "vehicle.lincoln.mkz_2017",
                                 "vehicle.mercedes.coupe_2020")
# Road 28: lanes -1 (x 5.1) and -2 (x 8.6) run towards -y, lanes 2 (x -1.9) and 3 (x -5.4) towards +y.
# Same-lane targets: the faster one ahead, so nobody catches up. (x, y) are snapped to the lane centre.
STRAIGHT = [
    ("obs_north", TESLA, 0, (-1.9, -62), True), ("obs_mid", TESLA, 0, (-1.9, -90), True),
    ("obs_south", TESLA, 0, (-1.9, -116), True),
    ("tesla_18", TESLA, 18, (5.1, -64), False), ("tesla_13", TESLA, 13, (5.1, -52), False),
    ("tesla_5", TESLA, 5, (5.1, -46), False),
    ("bmw_9", BMW, 9, (8.6, -72), False), ("lincoln_9", LINCOLN, 9, (8.6, -60), False),
    ("mercedes_9", MERCEDES, 9, (8.6, -48), False),
    ("tesla_9_up", TESLA, 9, (-5.4, -100), False), ("tesla_2_up", TESLA, 2, (-5.4, -118), False),
    ("tesla_0", TESLA, 0, (-1.9, -76), False), ("bmw_0", BMW, 0, (-1.9, -104), False),
    ("lincoln_0", LINCOLN, 0, (-5.4, -58), False), ("mercedes_0", MERCEDES, 0, (-1.9, -50), False),
]
OBSERVER = [
    ("obs_moving_15", TESLA, 15, (5.1, -48), True), ("obs_parked", TESLA, 0, (-1.9, -90), True),
    ("tesla_0_a", TESLA, 0, (8.6, -70), False), ("bmw_0", BMW, 0, (8.6, -85), False),
    ("lincoln_0", LINCOLN, 0, (-1.9, -65), False), ("mercedes_0", MERCEDES, 0, (-1.9, -110), False),
    ("tesla_0_b", TESLA, 0, (-5.4, -80), False), ("tesla_0_ahead", TESLA, 0, (5.1, -112), False),
    ("tesla_9_up", TESLA, 9, (-5.4, -118), False),
]
# Road 18 (x 21 .. 71, 0 -> 8 m): lane 1 (y -133.8) uphill towards +x, lane -1 (y -137.3) downhill towards -x.
SLOPE = [
    ("obs_top", TESLA, 0, (68, -133.8), True), ("obs_bottom", TESLA, 0, (22, -137.3), True),
    ("obs_below", TESLA, 0, (12, -135.5), True),
    ("up_7", TESLA, 7, (30, -133.8), False), ("up_3", TESLA, 3.5, (22, -133.8), False),
    ("down_7", TESLA, 7, (62, -137.3), False), ("down_3", TESLA, 3.5, (70, -137.3), False),
    ("parked_up", TESLA, 0, (60, -133.8), False), ("parked_down", TESLA, 0, (32, -137.3), False),
]
CASES = {
    "straight": {"actors": STRAIGHT, "ticks": 60},
    "straight_dt10": {"actors": STRAIGHT, "ticks": 30, "overrides": ["simulation.fixed_delta_seconds=0.1",
                                                                     "lidar.rotation_frequency=10.0"]},
    "observer": {"actors": OBSERVER, "ticks": 60},
    "slope": {"actors": SLOPE, "ticks": 50},
    "traffic1": {"config": "config/experiments/ablation/scene_moving1.yaml", "ticks": 60},
    "traffic2": {"config": "config/experiments/ablation/scene_moving2.yaml", "ticks": 60},
    "e12": {"config": "config/experiments/scenario/e12_figure2_driving.yaml", "ticks": 60},
}
BASE_CONFIG = "config/experiments/ablation/scene_moving2.yaml"   # server, simulation and LiDAR of the benchmark


# --------------------------------------------------------------------------------------------------------------
# recording

class SemanticLidar:
    """A semantic ray-cast LiDAR with the rays of a LidarConfig, read frame by frame like the project's Lidar."""

    def __init__(self, world, vehicle, cfg, name):
        import carla
        bp = world.get_blueprint_library().find(SEMANTIC_BLUEPRINT)
        for key in ("channels", "range", "points_per_second", "rotation_frequency", "upper_fov", "lower_fov"):
            bp.set_attribute(key, str(getattr(cfg, key)))
        mount = carla.Transform(carla.Location(x=cfg.mount_x, y=cfg.mount_y, z=cfg.mount_z))
        self.name, self.vehicle = name, vehicle
        self.actor = world.spawn_actor(bp, mount, attach_to=vehicle)
        self._queue = queue.Queue()
        self.actor.listen(self._queue.put)

    def get(self, frame, timeout=30.0):
        from cavlidar.backend.geometry.transforms import transform_to_dict
        while True:
            m = self._queue.get(timeout=timeout)
            if m.frame == frame:
                data = np.frombuffer(m.raw_data, dtype=SEM_DTYPE).copy()
                return m.frame, m.timestamp, transform_to_dict(m.transform), data
            if m.frame > frame:
                raise RuntimeError("{}: expected frame {}, got {}".format(self.name, frame, m.frame))

    def destroy(self):
        if self.actor is not None:
            self.actor.stop()
            self.actor.destroy()
            self.actor = None


def _vec(v):
    return [v.x, v.y, v.z]


def _actor_state(snap, actor_id):
    from cavlidar.backend.geometry.transforms import transform_to_dict
    s = snap.find(actor_id)
    if s is None:
        return None
    return {"transform": transform_to_dict(s.get_transform()), "velocity": _vec(s.get_velocity()),
            "angular_velocity": _vec(s.get_angular_velocity()), "acceleration": _vec(s.get_acceleration())}


def capture(session, frame, tick, observers, received, case_dir, dt):
    """One tick record (the recorder's order: tick, read the LiDARs, collect the ground truth)."""
    from cavlidar.backend.actors import ground_truth
    world = session.world
    snap = world.get_snapshot()
    snap_rec = {"frame": snap.frame, "elapsed": snap.timestamp.elapsed_seconds,
                "delta": snap.timestamp.delta_seconds, "platform": snap.timestamp.platform_timestamp}
    vehicle_ids = [v.id for v in session.cavs + session.traffic]
    actors = {str(i): _actor_state(snap, i) for i in vehicle_ids}
    sensors = {}
    for name, (lidar, sem) in observers.items():
        scan = lidar.get(frame)
        s_frame, s_time, s_pose, data = sem.get(frame)
        vehicle_hits = data[data["tag"] == VEHICLE_TAG]
        np.savez(os.path.join(case_dir, "points", "{:03d}_{}_sem.npz".format(tick, name)),
                 xyz=np.stack([vehicle_hits["x"], vehicle_hits["y"], vehicle_hits["z"]], axis=1),
                 idx=vehicle_hits["idx"])
        if tick % 2 == 0:
            np.save(os.path.join(case_dir, "points", "{:03d}_{}_ray.npy".format(tick, name)), scan.points)
        sem_xyz = np.stack([data["x"], data["y"], data["z"]], axis=1)
        sensors[name] = {
            "vehicle_id": lidar.vehicle.id,
            "ray": {"frame": scan.frame, "timestamp": scan.timestamp, "transform": scan.sensor_pose,
                    "points": int(len(scan.points))},
            "sem": {"frame": s_frame, "timestamp": s_time, "transform": s_pose, "points": int(len(data)),
                    "vehicle_points": int(len(vehicle_hits))},
            "ray_actor": (_actor_state(snap, lidar.actor.id) or {}).get("transform"),
            "sem_actor": (_actor_state(snap, sem.actor.id) or {}).get("transform"),
            "ray_in_sem": la.subset_share(scan.points[:, :3], sem_xyz),
            "ray_in_sem_1mm": la.subset_share(scan.points[:, :3], sem_xyz, tol=0.001),
        }
    gt = ground_truth.collect_vehicles(world, session.role_names())   # what the recorder writes to meta.json
    after = world.get_snapshot()
    return {"tick": tick, "frame": frame, "dt": dt, "snapshot": snap_rec,
            "snapshot_after_read": {"frame": after.frame, "elapsed": after.timestamp.elapsed_seconds},
            "received": received, "actors": actors, "recorder_gt": gt, "sensors": sensors}


def _spawn_placed(session, actors):
    """Spawn the case's vehicles on their lane centres (brakes on). Returns {name: (actor, speed, observer)}."""
    import carla
    from cavlidar.backend.actors import vehicles
    world = session.world
    carla_map = world.get_map()
    lib = world.get_blueprint_library()
    placed = {}
    for name, blueprint, speed, (x, y), observer in actors:
        wp = carla_map.get_waypoint(carla.Location(x=x, y=y, z=0.0), project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        tf = wp.transform
        tf = carla.Transform(carla.Location(x=tf.location.x, y=tf.location.y, z=tf.location.z + vehicles.SPAWN_LIFT),
                             carla.Rotation(pitch=tf.rotation.pitch, yaw=tf.rotation.yaw, roll=0.0))
        bp = lib.find(blueprint)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", name)
        actor = world.try_spawn_actor(bp, tf)
        if actor is None:
            raise RuntimeError("{} could not be spawned at ({:.1f}, {:.1f})".format(name, x, y))
        (session.cavs if observer else session.traffic).append(actor)
        placed[name] = (actor, float(speed), observer)
    vehicles.park([a for a, _, _ in placed.values()])
    return placed


def moving_share(ticks):
    speeds = [np.linalg.norm(a["velocity"]) for t in ticks for a in t["actors"].values() if a is not None]
    return float(np.mean(np.array(speeds) > 1.0)) if speeds else 0.0


def record_case(case, force=False):
    """Record a case; a traffic case with too little motion is kept aside and recorded again."""
    spec = CASES[case]
    case_dir = os.path.join(OUT_DATA, case)
    if os.path.exists(os.path.join(case_dir, "ticks.json")) and not force:
        print("[skip] {}: already recorded".format(case))
        return
    for attempt in range(1, MAX_ATTEMPTS + 1):
        _record_once(case, attempt)
        if "config" not in spec:
            return
        _, ticks = load_case(case)
        share = moving_share(ticks)
        if share >= MIN_MOVING_SHARE:
            print("[valid] {}: {:.0%} of the vehicle states move".format(case, share))
            return
        n = 1
        while os.path.exists(os.path.join(OUT_DATA, "{}_rejected_{}".format(case, n))):
            n += 1
        os.rename(case_dir, os.path.join(OUT_DATA, "{}_rejected_{}".format(case, n)))
        print("[rejected] {}: only {:.0%} of the vehicle states move (kept as {}_rejected_{})".format(
            case, share, case, n))
    print("[failed] {}: no valid recording in {} attempts".format(case, MAX_ATTEMPTS))


def _record_once(case, attempt=1):
    import carla
    from cavlidar import api
    from cavlidar.backend.actors.lidar import Lidar
    from cavlidar.backend.simulator.session import SimulationSession
    spec = CASES[case]
    case_dir = os.path.join(OUT_DATA, case)
    os.makedirs(os.path.join(case_dir, "points"), exist_ok=True)
    cfg = api.load_config(os.path.join(ROOT, spec.get("config", BASE_CONFIG)), overrides=spec.get("overrides", []))
    dt = cfg.simulation.fixed_delta_seconds
    received = []
    with SimulationSession(cfg) as session:
        world = session.world
        names = {}
        if "actors" in spec:
            placed = _spawn_placed(session, spec["actors"])
            names = {a.id: n for n, (a, _, _) in placed.items()}
            for _ in range(SETTLE_TICKS):
                session.tick()
            observer_vehicles = [(n, a) for n, (a, _, obs) in placed.items() if obs]
        else:
            session.spawn_fleet()
            observer_vehicles = [(session.role_names()[v.id], v) for v in session.cavs]
        observers = {}
        for name, vehicle in observer_vehicles:
            lidar = Lidar(world, vehicle, cfg.lidar, name)
            session.lidars.append(lidar)
            sem = SemanticLidar(world, vehicle, cfg.lidar, name)
            session.lidars.append(sem)                     # destroyed with the session
            observers[name] = (lidar, sem)
        if "actors" in spec:
            for name, (actor, speed, _) in placed.items():
                if speed > 0:
                    actor.apply_control(carla.VehicleControl())
                    actor.enable_constant_velocity(carla.Vector3D(speed, 0.0, 0.0))
            warm = PREROLL_TICKS
        else:
            warm = cfg.recording.warmup_ticks
        for _ in range(warm):
            frame = session.tick()
            for lidar, sem in observers.values():
                lidar.get(frame)
                sem.get(frame)
            session.update_camera()
        session.start_routes()
        world.on_tick(lambda s: received.append((s.frame, s.timestamp.elapsed_seconds)))
        ticks = []
        for tick in range(spec["ticks"]):
            frame = session.tick()
            rec = capture(session, frame, tick, observers, [], case_dir, dt)
            session.update_camera()
            ticks.append(rec)
            # every snapshot received since the previous tick, collected just before the next tick
            rec["received"] = list(received)
            del received[:]
        role_names = session.role_names()
        static = {}
        for v in session.cavs + session.traffic:
            bb = v.bounding_box
            static[str(v.id)] = {"name": names.get(v.id, role_names.get(v.id)), "type_id": v.type_id,
                                 "extent": [bb.extent.x, bb.extent.y, bb.extent.z],
                                 "center_offset": [bb.location.x, bb.location.y, bb.location.z],
                                 "box_rotation": [bb.rotation.roll, bb.rotation.pitch, bb.rotation.yaw]}
        settings = world.get_settings()
        physics = {"substepping": settings.substepping, "max_substep_delta_time": settings.max_substep_delta_time,
                   "max_substeps": settings.max_substeps}
        info = {"case": case, "attempt": attempt, "dt": dt, "ticks": spec["ticks"], "config": spec.get("config", BASE_CONFIG),
                "overrides": spec.get("overrides", []), "seed": cfg.simulation.seed, "physics": physics,
                "lidar": {k: getattr(cfg.lidar, k) for k in ("channels", "range", "points_per_second",
                                                               "rotation_frequency", "mount_z",
                                                               "dropoff_general_rate", "noise_stddev")},
                "speeds": {n: s for n, (_, s, _) in placed.items()} if "actors" in spec else {},
                "observers": {n: l.vehicle.id for n, (l, _) in observers.items()},
                "vehicles": static,
                "server_version": session.client.get_server_version()}
    with open(os.path.join(case_dir, "case.json"), "w", encoding="utf-8") as fh:
        json.dump(info, fh, indent=1)
    with open(os.path.join(case_dir, "ticks.json"), "w", encoding="utf-8") as fh:
        json.dump(ticks, fh)
    print("[recorded] {}: {} ticks, {} observers, {} vehicles".format(case, len(ticks), len(observers), len(static)))


# --------------------------------------------------------------------------------------------------------------
# analysis

def load_case(case):
    d = os.path.join(OUT_DATA, case)
    with open(os.path.join(d, "case.json"), encoding="utf-8") as fh:
        info = json.load(fh)
    with open(os.path.join(d, "ticks.json"), encoding="utf-8") as fh:
        ticks = json.load(fh)
    return info, ticks


def consistency(info, ticks):
    """Frame/time checks, recorder ground truth vs ActorSnapshot, sensor transforms vs parents, ray vs semantic."""
    out = la.frame_checks(ticks)
    out["snapshot_changed_while_reading"] = sum(t["snapshot_after_read"]["frame"] != t["frame"] for t in ticks)
    gt_pos, gt_ang, gt_speed = 0.0, 0.0, 0.0
    sensor_pos, sensor_ang, ray_sem_pose, mount_pos, mount_ang = 0.0, 0.0, 0.0, 0.0, 0.0
    ray_in_sem = []
    mount = (0.0, 0.0, info["lidar"]["mount_z"])
    for t in ticks:
        for v in t["recorder_gt"]:
            s = t["actors"][str(v["id"])]
            dp, da = la.pose_difference(v["pose"], s["transform"])
            gt_pos, gt_ang = max(gt_pos, dp), max(gt_ang, da)
            gt_speed = max(gt_speed, abs(v["speed_mps"] - float(np.linalg.norm(s["velocity"]))))
        for s in t["sensors"].values():
            if s["ray_actor"] is not None:
                dp, da = la.pose_difference(s["ray"]["transform"], s["ray_actor"])
                sensor_pos, sensor_ang = max(sensor_pos, dp), max(sensor_ang, da)
            ray_sem_pose = max(ray_sem_pose, la.pose_difference(s["ray"]["transform"], s["sem"]["transform"])[0])
            parent = la.compose(t["actors"][str(s["vehicle_id"])]["transform"], mount)
            dp, da = la.pose_difference(s["ray"]["transform"], parent)
            mount_pos, mount_ang = max(mount_pos, dp), max(mount_ang, da)
            ray_in_sem.append((s["ray_in_sem"], s["ray_in_sem_1mm"]))
    out.update({"recorder_gt_vs_snapshot_max_m": gt_pos, "recorder_gt_vs_snapshot_max_deg": gt_ang,
                "recorder_speed_vs_snapshot_max_mps": gt_speed,
                "sensor_transform_vs_sensor_snapshot_max_m": sensor_pos,
                "sensor_transform_vs_sensor_snapshot_max_deg": sensor_ang,
                "ray_vs_semantic_sensor_transform_max_m": ray_sem_pose,
                "sensor_transform_vs_parent_snapshot_plus_mount_max_m": mount_pos,
                "sensor_transform_vs_parent_snapshot_plus_mount_max_deg": mount_ang,
                "ray_returns_in_semantic_exact_min": float(min(x for x, _ in ray_in_sem)),
                "ray_returns_in_semantic_exact_mean": float(np.mean([x for x, _ in ray_in_sem])),
                "ray_returns_in_semantic_1mm_min": float(min(x for _, x in ray_in_sem))})
    return out


def _vehicle(info, state, vid):
    v = info["vehicles"][str(vid)]
    return {"id": int(vid), "pose": state["transform"], "extent": v["extent"], "center_offset": v["center_offset"]}


def views(case, info, ticks, ks=(-2, -1, 0, 1, 2)):
    """One row per (tick, observer, vehicle with >= MIN_POINTS semantic returns), offsets for boxes of t+k."""
    from cavlidar.backend.geometry.transforms import sensor_to_world
    d = os.path.join(OUT_DATA, case, "points")
    rows = []
    dt = info["dt"]
    for i, t in enumerate(ticks):
        if i + min(ks) < 0 or i + max(ks) >= len(ticks):
            continue
        for name, s in t["sensors"].items():
            pts = np.load(os.path.join(d, "{:03d}_{}_sem.npz".format(t["tick"], name)))
            pose = s["sem"]["transform"]
            world = sensor_to_world(pts["xyz"].astype(np.float64), pose)
            sensor_xyz = (pose["x"], pose["y"], pose["z"])
            obs_state = t["actors"][str(s["vehicle_id"])]
            ray = None
            if t["tick"] % 2 == 0:
                rp = np.load(os.path.join(d, "{:03d}_{}_ray.npy".format(t["tick"], name)))
                ray = sensor_to_world(rp[:, :3].astype(np.float64), s["ray"]["transform"])
            gt_by_id = {v["id"]: v for v in t["recorder_gt"]}
            for vid in np.unique(pts["idx"]):
                vid = int(vid)
                if vid == s["vehicle_id"] or str(vid) not in info["vehicles"]:
                    continue
                mine = world[pts["idx"] == vid]
                if len(mine) < MIN_POINTS:
                    continue
                state = t["actors"][str(vid)]
                box = _vehicle(info, state, vid)
                e = box["extent"]
                view = la.view_of(sensor_xyz, box)
                yaw = state["transform"]["yaw"]
                row = {"case": case, "dt": dt, "tick": t["tick"], "observer": name, "id": vid,
                       "name": info["vehicles"][str(vid)]["name"], "model": info["vehicles"][str(vid)]["type_id"],
                       "view": view, "points": int(len(mine)),
                       "distance": float(np.hypot(state["transform"]["x"] - pose["x"], state["transform"]["y"] - pose["y"])),
                       "speed": float(np.linalg.norm(state["velocity"])),
                       "v_along": la.along_speed(state["velocity"], yaw),
                       "a_along": la.along_speed(state["acceleration"], yaw),
                       "yaw_rate": float(state["angular_velocity"][2]),
                       "pitch": float((state["transform"]["pitch"] + 180.0) % 360.0 - 180.0),
                       "observer_speed": float(np.linalg.norm(obs_state["velocity"])),
                       "observer_v_along": la.along_speed(obs_state["velocity"], yaw)}
                for k in ks:
                    bk = _vehicle(info, ticks[i + k]["actors"][str(vid)], vid)
                    a, b, h = la.box_frame(mine, bk)
                    row["offset_k{:+d}".format(k)] = la.face_offset(a, e[0], view)
                    row["inside_k{:+d}".format(k)] = float(np.mean((np.abs(a) <= e[0] + 0.05) &
                                                                   (np.abs(b) <= e[1] + 0.05)))
                a, _, _ = la.box_frame(mine, box)
                row["offset_rear"] = float(a.min() + e[0])
                row["offset_front"] = float(a.max() - e[0])
                if ray is not None and vid in gt_by_id:
                    # the same measurement on the recorder's ray-cast LiDAR, returns attributed by the
                    # recorder's box (no labels), as eval_ablation --check-labels does on recorded runs
                    gt = gt_by_id[vid]
                    others = [o for o in t["recorder_gt"] if o["id"] not in (vid, s["vehicle_id"])]
                    near = la.near_box(ray, gt, others)
                    if near.sum() >= MIN_POINTS:
                        ra = la.box_frame(ray[near], gt)[0]
                        row["ray_offset"] = la.face_offset(ra, gt["extent"][0], view, q=0.02)
                        row["ray_tie_shift"] = la.tie_shift(ray[near], gt)[0]
                rows.append(row)
    return rows


def _fit(rows, k=0, regressor="v_along"):
    """
    offset_k = tau * regressor + c_view (one intercept per view, so the sampling gap is not read as a
    lag): least squares, 95 % interval from a bootstrap over vehicles (views of one vehicle are not
    independent). regressor: the target's speed along its heading, or the observer's speed projected on it.
    """
    if not rows:
        return None
    y = np.array([r["offset_k{:+d}".format(k)] for r in rows])
    v = np.array([r[regressor] for r in rows])
    if np.ptp(v) < 1.0:
        return None
    views = [w for w in la.VIEWS if any(r["view"] == w for r in rows)]
    X = np.column_stack([v] + [[1.0 if r["view"] == w else 0.0 for r in rows] for w in views])
    coef = np.linalg.lstsq(X, y, rcond=None)[0]
    groups = sorted({(r["case"], r["id"]) for r in rows})
    index = {g: [i for i, r in enumerate(rows) if (r["case"], r["id"]) == g] for g in groups}
    rng = np.random.default_rng(0)
    boot = []
    for _ in range(500):
        pick = [i for g in rng.choice(len(groups), len(groups)) for i in index[groups[g]]]
        if np.ptp(X[pick, 0]) < 1.0:
            continue
        boot.append(np.linalg.lstsq(X[pick], y[pick], rcond=None)[0][0])
    lo, hi = np.percentile(boot, [2.5, 97.5])
    return {"tau_s": float(coef[0]), "tau_ci95_s": [float(lo), float(hi)], "views": len(rows),
            "vehicles": len(groups), "intercepts_m": {w: float(c) for w, c in zip(views, coef[1:])}}


def stationary_gaps(rows):
    """Median offset of stationary vehicles per view and per (model, view): the sampling gap at each end."""
    still = [r for r in rows if r["speed"] < 0.1]
    by_view = {w: _median([r for r in still if r["view"] == w], "offset_k+0") for w in la.VIEWS}
    by_model = {}
    for r in still:
        by_model.setdefault((r["model"], r["view"]), []).append(r["offset_k+0"])
    return by_view, {key: float(np.median(v)) for key, v in by_model.items() if len(v) >= 5}


def per_model(rows):
    """Offset to the box of tick t per model and view, stationary vs moving (>= 1 m/s along the heading)."""
    out = {}
    for r in rows:
        state = "stationary" if r["speed"] < 0.1 else ("moving" if abs(r["v_along"]) >= 1 else None)
        if state:
            out.setdefault(r["model"], {}).setdefault(state, {}).setdefault(r["view"], []).append(r["offset_k+0"])
    return {m: {st: {w: {"views": len(v), "median": float(np.median(v)), "p05": float(np.percentile(v, 5)),
                         "p95": float(np.percentile(v, 95))} for w, v in d2.items()} for st, d2 in d.items()}
            for m, d in sorted(out.items())}


def _median(rows, key):
    vals = [r[key] for r in rows if key in r and np.isfinite(r[key])]
    return float(np.median(vals)) if vals else float("nan")


def summarise_views(rows, dt, gaps):
    """
    Medians per motion class and view, offsets to the boxes of t-2 .. t+2, implied time offsets, lag fits.
    gaps: stationary_gaps(); a moving view's time offset and best box tick are taken relative to the
    sampling gap of stationary vehicles of the same model and view (of the same view if the model never stood).
    """
    by_view, by_model = gaps
    classes = {
        "stationary (< 0.1 m/s)": lambda r: r["speed"] < 0.1,
        "slow (0.1 - 5 m/s)": lambda r: 0.1 <= r["speed"] < 5,
        "medium (5 - 12 m/s)": lambda r: 5 <= r["speed"] < 12,
        "fast (>= 12 m/s)": lambda r: r["speed"] >= 12,
        "turning (|yaw rate| >= 5 deg/s)": lambda r: abs(r["yaw_rate"]) >= 5 and r["speed"] >= 1,
        "on the slope (|pitch| >= 2 deg)": lambda r: abs(r["pitch"]) >= 2,
        "accelerating / braking (|a| >= 1 m/s2)": lambda r: abs(r["a_along"]) >= 1 and r["speed"] >= 1,
        "stationary, seen by a moving observer (>= 5 m/s)": lambda r: r["speed"] < 0.1 and r["observer_speed"] >= 5,
    }
    out = {}
    for label, test in classes.items():
        sel = [r for r in rows if test(r)]
        if not sel:
            continue
        entry = {"views": len(sel), "vehicles": len({(r["case"], r["id"]) for r in sel}),
                 "speed_median": _median(sel, "speed")}
        for w in la.VIEWS:
            sw = [r for r in sel if r["view"] == w]
            if sw:
                entry[w] = {"views": len(sw), **{"offset_k{:+d}".format(k): _median(sw, "offset_k{:+d}".format(k))
                                                for k in (-2, -1, 0, 1, 2)},
                            "inside_k+0": _median(sw, "inside_k+0")}
        moving = [r for r in sel if abs(r["v_along"]) >= 1]
        if moving:
            gap = [by_model.get((r["model"], r["view"]), by_view[r["view"]]) for r in moving]
            entry["time_offset_s_median"] = float(np.median([(r["offset_k+0"] - g) / r["v_along"]
                                                             for r, g in zip(moving, gap)]))
            best = []
            for r, g in zip(moving, gap):
                errs = {k: abs(r["offset_k{:+d}".format(k)] - g) for k in (-2, -1, 0, 1, 2)}
                best.append(min(errs, key=errs.get))
            entry["best_box_tick"] = {"t{:+d}".format(k): best.count(k) for k in (-2, -1, 0, 1, 2)}
        out[label] = entry
    still = [r for r in rows if r["speed"] < 0.1]
    return {"classes": out, "lag_fit": _fit(rows),   # stationary views anchor the per-view intercepts
            "observer_fit": _fit(still, regressor="observer_v_along"),
            "one_tick_late_would_give_tau_s": -dt}


def analyse(cases):
    os.makedirs(OUT_RESULTS, exist_ok=True)
    report = {"cases": {}}
    loaded = {}
    for case in cases:
        if not os.path.exists(os.path.join(OUT_DATA, case, "ticks.json")):
            print("[missing] {}".format(case))
            continue
        info, ticks = load_case(case)
        loaded[case] = (info, ticks, views(case, info, ticks))
    all_rows = [r for _, _, rows in loaded.values() for r in rows]
    gaps = stationary_gaps(all_rows)
    for case, (info, ticks, rows) in loaded.items():
        cons = consistency(info, ticks)
        rotations = [v["box_rotation"] for v in info["vehicles"].values()]
        cons["box_rotation_max_deg"] = float(np.max(np.abs(rotations))) if rotations else 0.0
        report["cases"][case] = {"dt": info["dt"], "physics": info["physics"], "server": info["server_version"],
                                 "attempt": info.get("attempt", 1), "moving_share": moving_share(ticks),
                                 "consistency": cons, **summarise_views(rows, info["dt"], gaps)}
        with open(os.path.join(OUT_RESULTS, "views_{}.csv".format(case)), "w", newline="", encoding="utf-8") as fh:
            keys = sorted({k for r in rows for k in r})
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        fit = report["cases"][case]["lag_fit"]
        print("[{}] {} views; frames {}; GT vs snapshot {:.2e} m; ray in semantic {:.4f}; tau {}".format(
            case, len(rows), {k: v for k, v in cons.items() if isinstance(v, int) and k != "ticks" and v},
            cons["recorder_gt_vs_snapshot_max_m"], cons["ray_returns_in_semantic_1mm_min"],
            "{:+.4f} s [{:+.4f}, {:+.4f}]".format(fit["tau_s"], *fit["tau_ci95_s"]) if fit else "-"))
    # pooled tables over the 0.05 s cases (offsets to t +- 1 scale with the step); every step has its own fit below
    report["all_cases"] = summarise_views([r for r in all_rows if r["dt"] == 0.05], 0.05, gaps)
    report["all_cases"]["lag_fit_by_step"] = {
        "{:g}".format(dt): _fit([r for r in all_rows if r["dt"] == dt]) for dt in sorted({r["dt"] for r in all_rows})}
    report["stationary_gap_by_view_m"] = gaps[0]
    report["per_model"] = per_model(all_rows)
    ray = [r for r in all_rows if "ray_offset" in r]
    if ray:
        diff = np.array([r["ray_offset"] - r["offset_k+0"] for r in ray])
        report["ray_cast_vs_semantic"] = {
            "views": len(ray), "offset_difference_median_m": float(np.median(diff)),
            "offset_difference_p05_p95_m": [float(x) for x in np.percentile(diff, [5, 95])],
            "tie_shift_by_view_median_m": {w: _median([r for r in ray if r["view"] == w], "ray_tie_shift")
                                           for w in la.VIEWS},
            "face_offset_by_view_median_m": {w: _median([r for r in ray if r["view"] == w], "ray_offset")
                                             for w in la.VIEWS}}
    with open(os.path.join(OUT_RESULTS, "diagnostic.json"), "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    return report, all_rows


# --------------------------------------------------------------------------------------------------------------
# figures

BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"     # categorical slots 1-3 (validated all-pairs)
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
VIEW_STYLE = {"rear": (BLUE, "sensor behind (rear end)"), "side": (AQUA, "sensor beside"),
              "front": (ORANGE, "sensor ahead (front end)")}


def _style(ax):
    ax.set_facecolor(SURFACE)
    ax.tick_params(labelsize=8, colors=INK2)
    ax.grid(color=GRID, lw=0.7)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)


def _read_views(case):
    with open(os.path.join(OUT_RESULTS, "views_{}.csv".format(case)), encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k, v in r.items():
            try:
                r[k] = float(v)
            except (TypeError, ValueError):
                pass
    return rows


def figure_ticks(report, path):
    """Offsets to the boxes of t-1, t, t+1 vs speed (0.05 s cases), and the fitted time offset per case."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = [r for c in report["cases"] for r in _read_views(c) if r["dt"] == 0.05]
    fig = plt.figure(figsize=(11.5, 7.2), facecolor=SURFACE)
    grid = fig.add_gridspec(2, 3, height_ratios=[1.25, 1.0], hspace=0.42, wspace=0.22)
    vmin, vmax = min(r["v_along"] for r in rows) - 1, max(r["v_along"] for r in rows) + 1
    for i, k in enumerate((-1, 0, 1)):
        ax = fig.add_subplot(grid[0, i])
        _style(ax)
        xs = np.linspace(vmin, vmax, 2)
        ax.plot(xs, -k * 0.05 * xs, color=INK2, lw=1.2, ls="--", zorder=1,
                label="expected if box and returns share the instant" if i == 0 else None)
        for view, (colour, label) in VIEW_STYLE.items():
            sel = [r for r in rows if r["view"] == view]
            ax.scatter([r["v_along"] for r in sel], [r["offset_k{:+d}".format(k)] for r in sel], s=7, color=colour,
                       alpha=0.35, lw=0, zorder=2, label=label if i == 0 else None)
        ax.set_ylim(-1.15, 1.15)
        ax.set_xlim(vmin, vmax)
        ax.set_title({-1: "box of tick t-1", 0: "box of tick t (what meta.json stores)", 1: "box of tick t+1"}[k],
                     fontsize=10, color=INK)
        ax.set_xlabel("vehicle speed along its heading (m/s)", fontsize=8.5, color=INK2)
        if i == 0:
            ax.set_ylabel("returns - box at the visible end (m)\n< 0: returns behind the box", fontsize=8.5, color=INK)
    leg = fig.legend(loc="upper center", ncol=4, fontsize=8.5, frameon=False, bbox_to_anchor=(0.5, 0.995),
                     markerscale=2.5)
    for handle in leg.legendHandles:
        handle.set_alpha(1.0)
    ax = fig.add_subplot(grid[1, :])
    _style(ax)
    cases = list(report["cases"])
    for j, case in enumerate(cases):
        c = report["cases"][case]
        fit = c["lag_fit"]
        ax.plot([-c["dt"]], [j], marker="x", color=ORANGE, ms=8, mew=2, zorder=3,
                label="a box one tick late would give" if j == 0 else None)
        if fit:
            ax.errorbar([fit["tau_s"]], [j], xerr=[[fit["tau_s"] - fit["tau_ci95_s"][0]],
                                                   [fit["tau_ci95_s"][1] - fit["tau_s"]]],
                        color=BLUE, marker="o", ms=6, mec=SURFACE, capsize=3, lw=1.8, zorder=4,
                        label="measured (95 % interval, bootstrap over vehicles)" if j == 0 else None)
            ax.text(0.106, j, "{:+.4f} s  ({} views, {} vehicles)".format(fit["tau_s"], fit["views"], fit["vehicles"]),
                    va="center", fontsize=8, color=INK2)
    ax.axvline(0, color=INK2, lw=1)
    ax.set_yticks(range(len(cases)))
    ax.set_yticklabels(["{} (step {:g} s)".format(c, report["cases"][c]["dt"]) for c in cases], fontsize=8.5,
                       color=INK)
    ax.set_xlim(-0.115, 0.2)
    ax.set_xticks([-0.1, -0.05, 0, 0.05, 0.1])
    ax.invert_yaxis()
    ax.set_xlabel("time offset of the box: fitted offset = tau x speed + one constant per viewing side (s)",
                  fontsize=8.5, color=INK2)
    ax.legend(loc="upper center", fontsize=8.5, frameon=False, ncol=2, bbox_to_anchor=(0.5, -0.17))
    fig.savefig(path, dpi=110, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)


def _corners(vehicle):
    e, off = vehicle["extent"], vehicle.get("center_offset", [0, 0, 0])
    from cavlidar.backend.geometry.transforms import pose_to_matrix
    m = pose_to_matrix(vehicle["pose"])
    local = np.array([[sx * e[0] + off[0], sy * e[1] + off[1], off[2], 1.0]
                      for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1))])
    return (local @ m.T)[:, :3]


def _overlay(ax, points, frame_box, boxes, title, note):
    """points / boxes drawn in frame_box's frame (x along its heading, y to its left)."""
    _style(ax)
    a, b, h = la.box_frame(points, frame_box)
    keep = h > 0.2
    ax.scatter(a[keep], -b[keep], s=5, color=INK, lw=0, zorder=4, label="LiDAR returns")
    for vehicle, colour, ls, label in boxes:
        ca, cb, _ = la.box_frame(_corners(vehicle), frame_box)
        ax.plot(ca, -cb, color=colour, ls=ls, lw=1.8, zorder=3, label=label)
    ax.set_aspect("equal")
    ax.set_xlim(-4.6, 4.6)
    ax.set_ylim(-2.3, 2.3)
    ax.set_title(title, fontsize=9.5, color=INK)
    ax.text(0.02, 0.03, note, transform=ax.transAxes, fontsize=7.8, color=INK2, va="bottom")
    ax.set_xlabel("along the heading (m), driving to the right", fontsize=8, color=INK2)
    ax.legend(loc="upper left", fontsize=7.2, frameon=False, ncol=2)


def figure_overlays(report, labels_dir, path):
    """Top views in the vehicle's box frame: diagnostic boxes of t-1, t, t+1; recorded runs before/after."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from cavlidar.backend.geometry.transforms import sensor_to_world
    from run_ablation import find_run
    fig, axes = plt.subplots(2, 2, figsize=(11.5, 6.4), facecolor=SURFACE)
    info, ticks = load_case("straight")
    rows = [r for r in _read_views("straight") if r["name"] == "tesla_18"]
    for ax, view in zip(axes[0], ("rear", "front")):
        cand = [r for r in rows if r["view"] == view and r["points"] >= 80]
        if not cand:
            ax.set_visible(False)
            continue
        r = max(cand, key=lambda x: x["points"])
        i = int(r["tick"])
        obs = r["observer"]
        pts = np.load(os.path.join(OUT_DATA, "straight", "points", "{:03d}_{}_sem.npz".format(i, obs)))
        world = sensor_to_world(pts["xyz"].astype(np.float64), ticks[i]["sensors"][obs]["sem"]["transform"])
        vid = int(r["id"])
        mine = world[pts["idx"] == vid]
        box = {k: _vehicle(info, ticks[i + k]["actors"][str(vid)], vid) for k in (-1, 0, 1)}
        _overlay(ax, mine, box[0], [(box[-1], AQUA, ":", "box of t-1"), (box[0], BLUE, "-", "box of t (recorded)"),
                                    (box[1], ORANGE, "--", "box of t+1")],
                 "diagnostic: Tesla at {:.1f} m/s, sensor {}".format(r["speed"], "behind" if view == "rear" else "ahead"),
                 "semantic returns of this vehicle only; offset to box t {:+.2f} m, one tick = {:.2f} m".format(
                     r["offset_k+0"], r["speed"] * info["dt"]))
    for ax, (name, view) in zip(axes[1], (("abl_moving2_noise_0", "rear"), ("e12_figure2_driving", "front"))):
        views_path = os.path.join(labels_dir, "labels_views_{}.json".format(name))
        run = find_run(name)
        if not os.path.exists(views_path) or run is None:
            ax.set_visible(False)
            continue
        with open(views_path, encoding="utf-8") as fh:
            lv = [x for x in json.load(fh) if x["view"] == view and x["speed"] >= 1.0]
        r = max(lv, key=lambda x: abs(x["tie_shift"]))
        frame_dir = os.path.join(run, "frames", "{:06d}".format(r["frame"]))
        with open(os.path.join(frame_dir, "meta.json"), encoding="utf-8") as fh:
            meta = json.load(fh)
        vehicle = next(v for v in meta["vehicles"] if v["id"] == r["id"])
        info_c = meta["cavs"][r["cav"]]
        world = sensor_to_world(np.load(os.path.join(frame_dir, r["cav"] + ".npy"))[:, :3].astype(np.float64),
                                info_c["sensor_pose"])
        others = [v for v in meta["vehicles"] if v["id"] != info_c["vehicle_id"]]
        near = la.near_box(world, vehicle, others)
        h = np.radians(vehicle["pose"]["yaw"])
        shifted = dict(vehicle, pose=dict(vehicle["pose"], x=vehicle["pose"]["x"] + r["tie_shift"] * np.cos(h),
                                          y=vehicle["pose"]["y"] + r["tie_shift"] * np.sin(h)))
        _overlay(ax, world[near], vehicle, [(vehicle, BLUE, "-", "recorded box"),
                                            (shifted, ORANGE, "--", "box moved by the first check")],
                 "benchmark run {}: {} at {:.1f} m/s, sensor {}".format(
                     name.replace("_figure2_driving", ""), vehicle["type_id"][8:], r["speed"],
                     "behind" if view == "rear" else "ahead"),
                 "ray-cast returns near the box; first check {:+.2f} m, visible end {:+.2f} m".format(
                     r["tie_shift"], r["offset"]))
    fig.tight_layout()
    fig.savefig(path, dpi=110, facecolor=SURFACE)
    plt.close(fig)


def figure_before_after(labels_dir, path):
    """Moving vehicles of the benchmark recordings: first estimator vs visible-end offset, by viewing side."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = []
    for p in sorted(glob.glob(os.path.join(labels_dir, "labels_views_*.json"))):
        name = os.path.basename(p)[len("labels_views_"):-5]
        if name.endswith("_camera"):          # same simulation as diag_targets_moving with a camera: duplicate views
            continue
        with open(p, encoding="utf-8") as fh:
            rows += [dict(x, run=name) for x in json.load(fh) if x["speed"] >= 1.0]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.3), sharey=True, facecolor=SURFACE)
    rng = np.random.default_rng(0)
    for ax, key, title in ((axes[0], "tie_shift", "before: first check (shift holding the most returns)"),
                           (axes[1], "offset", "after: offset at the visible end of the box")):
        _style(ax)
        for j, view in enumerate(la.VIEWS):
            colour, label = VIEW_STYLE[view]
            sel = [r for r in rows if r["view"] == view and np.isfinite(r[key])]
            if not sel:
                continue
            ys = np.array([r[key] for r in sel])
            ax.scatter(j + rng.uniform(-0.28, 0.28, len(ys)), ys, s=9, color=colour, alpha=0.45, lw=0, zorder=2)
            med = float(np.median(ys))
            ax.plot([j - 0.36, j + 0.36], [med, med], color=INK, lw=2.2, zorder=3)
            ax.text(j + 0.38, med, "{:+.2f}".format(med), va="center", fontsize=8.5, color=INK)
            lag = -0.05 * float(np.median([r["speed"] for r in sel]))
            ax.plot([j - 0.36, j + 0.36], [lag, lag], color=ORANGE, lw=1.4, ls="--", zorder=3,
                    label="a box one tick late (median speed x 0.05 s)" if j == 0 else None)
            ax.text(j, 1.02, "n = {}".format(len(sel)), ha="center", fontsize=8, color=INK2,
                    transform=ax.get_xaxis_transform())
        ax.axhline(0, color=INK2, lw=1, zorder=1)
        ax.set_xticks(range(3))
        ax.set_xticklabels([VIEW_STYLE[v][1] for v in la.VIEWS], fontsize=8.5, color=INK)
        ax.set_title(title, fontsize=10, color=INK, pad=16)
        ax.set_ylim(-2.05, 2.05)
    axes[0].set_ylabel("estimated box offset along the heading (m)\n< 0: returns behind the box", fontsize=8.5,
                       color=INK)
    axes[0].legend(loc="lower left", fontsize=8, frameon=False)
    fig.text(0.5, -0.02, "moving vehicles (>= 1 m/s) in {} benchmark recordings (moving1/2, e12, label diagnostics); "
             "black bar = median".format(len({r["run"] for r in rows})), ha="center", fontsize=8.5, color=INK2)
    fig.tight_layout()
    fig.savefig(path, dpi=110, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)


def figures(labels_dir):
    with open(os.path.join(OUT_RESULTS, "diagnostic.json"), encoding="utf-8") as fh:
        report = json.load(fh)
    figure_ticks(report, os.path.join(OUT_RESULTS, "alignment_ticks.png"))
    figure_overlays(report, labels_dir, os.path.join(OUT_RESULTS, "alignment_overlays.png"))
    figure_before_after(labels_dir, os.path.join(OUT_RESULTS, "alignment_before_after.png"))
    print("[figures] {}".format(OUT_RESULTS))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("command", choices=["record", "analyse", "figures"])
    p.add_argument("--cases", nargs="+", default=list(CASES), choices=list(CASES))
    p.add_argument("--force", action="store_true", help="record a case again (replaces its diagnostic data)")
    p.add_argument("--labels-dir", default=os.path.join(ROOT, "data", "ablation", "labels"),
                   help="output folder of eval_ablation.py --check-labels (per-view files), for the figures")
    args = p.parse_args(argv)
    if args.command == "record":
        for case in args.cases:
            record_case(case, args.force)
    elif args.command == "analyse":
        analyse(args.cases)
    else:
        figures(args.labels_dir)


if __name__ == "__main__":
    main()
