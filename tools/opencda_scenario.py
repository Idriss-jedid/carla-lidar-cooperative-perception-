"""
The paper's scenarios with OpenCDA, the tool the paper used, recorded in this repository's run format.

    conda activate opencda                      # Python 3.7 + OpenCDA's requirements + carla 0.9.12
    python tools/opencda_scenario.py --cavs 4 --seed 1 --frames 10
    python main.py process data/runs/<date>_<time>_opencda_s1_4cav       # then the usual processing (cav env)

The paper: "System performance was evaluated over a total of 228 simulated driving scenarios generated in CARLA's
Town 3 using the OpenCDA framework. Each scenario corresponds to a random timestamp at a four-leg intersection, with
both CAVs and non-CAVs randomly initialized" (57 per CAV count 4, 6, 8, 10; 20-40 other vehicles; Table 1).

From the seed this script draws one such scenario around junction 1221 (the paper's Figure 2 junction):
- CAVs on driving lanes 12-60 m from the junction centre, each with a destination about 120 m further along the
  lanes (through the junction), driven by OpenCDA's behaviour agent with max speed 20-50 km/h and tailgate speed
  30-60 km/h (Table 1);
- 20-40 other vehicles at random in the paper's spawn area (x -100..100, y -200..200), driven by CARLA's Traffic
  Manager through OpenCDA, each with a speed offset of -20..+20 % and a red-light disregard of 0..100 % (Table 1);
- the paper's LiDAR (32 channels, 120 m, 1 000 000 points/s, 20 Hz) on every CAV, mounted by OpenCDA (x -0.5 m,
  z 1.9 m on a Lincoln MKZ 2017);
then runs it in synchronous mode and saves, after a warm-up, one frame every --every ticks: each CAV's sweep of that
tick and every vehicle's pose and box of the same tick, as data/runs/<run>/frames/<frame>/cavN.npy + meta.json.

CARLA 0.9.12 must already run Town03: the world is not reloaded (load_world crashed CARLA on a 4 GB GPU). OpenCDA's
LiDAR and camera windows are disabled. Not reproduced: the paper's "random yaw -180..180" (OpenCDA spawns vehicles
along their lane, as here) and its random choice of timestamps (frames are saved at a fixed interval).
"""
import argparse
import json
import math
import os
import random
import sys
import time

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OPENCDA = os.environ.get("OPENCDA_ROOT", os.path.join(os.path.dirname(ROOT), "external", "OpenCDA"))
CONFIG = os.path.join(ROOT, "config", "opencda", "paper_town03.yaml")
JUNCTION = (2.9, -136.5)          # Town03 junction 1221, the paper's Figure 2
BBOX = (-100.0, 100.0, -200.0, 200.0)
CAV_BLUEPRINT = "vehicle.lincoln.mkz_2017"
LIDAR_MOUNT = {"mount_x": -0.5, "mount_y": 0.0, "mount_z": 1.9}   # OpenCDA's LidarSensor


def quiet_windows():
    """OpenCDA shows the LiDAR in an Open3D window and cameras with OpenCV: neither may open on the user's screen."""
    import cv2

    import opencda.core.sensing.perception.perception_manager as pm

    class NoWindow:
        def destroy_window(self):
            pass

    cv2.imshow = lambda *a, **k: None
    cv2.waitKey = lambda *a, **k: -1
    cv2.destroyAllWindows = lambda *a, **k: None
    pm.o3d_visualizer_init = lambda *a, **k: NoWindow()
    pm.o3d_visualizer_show = lambda *a, **k: None


def pose_dict(t):
    return {"x": t.location.x, "y": t.location.y, "z": t.location.z,
            "roll": t.rotation.roll, "pitch": t.rotation.pitch, "yaw": t.rotation.yaw}


def route_end(wp, rng, steps=60):
    """The location `steps` x 2 m further along the lanes from `wp`, with a random branch at every split."""
    for _ in range(steps):
        nxt = wp.next(2.0)
        if not nxt:
            break
        wp = rng.choice(nxt)
    return wp.transform.location


def draw_cavs(carla_map, count, rng):
    """[(spawn waypoint, destination location)]: CAVs around the junction, each heading on ~120 m of lanes."""
    import carla
    lanes = [w for w in carla_map.generate_waypoints(2.0)
             if w.lane_type == carla.LaneType.Driving and not w.is_junction
             and 12.0 <= math.hypot(w.transform.location.x - JUNCTION[0], w.transform.location.y - JUNCTION[1]) <= 60]
    rng.shuffle(lanes)
    chosen = []
    for w in lanes:
        loc = w.transform.location
        if all(loc.distance(c.transform.location) >= 12.0 for c, _ in chosen):
            goal = route_end(w, rng)
            if goal.distance(loc) > 40:
                chosen.append((w, goal))
        if len(chosen) == count:
            break
    if len(chosen) < count:
        raise RuntimeError("Only {} of {} CAV positions found around the junction".format(len(chosen), count))
    return chosen


def build_params(args, carla_map, rng):
    from omegaconf import OmegaConf
    from opencda.scenario_testing.utils.yaml_utils import add_current_time

    params = OmegaConf.merge(
        OmegaConf.load(os.path.join(OPENCDA, "opencda", "scenario_testing", "config_yaml", "default.yaml")),
        OmegaConf.load(CONFIG))
    params.world.seed = args.seed
    params.carla_traffic_manager.range[0][6] = args.traffic
    cavs, behaviour = [], []
    for i, (wp, goal) in enumerate(draw_cavs(carla_map, args.cavs, rng)):
        t = wp.transform
        max_speed = round(rng.uniform(20, 50), 1)
        tailgate = round(rng.uniform(max(30, max_speed), 60), 1)
        behaviour.append({"cav": "cav_{}".format(i), "max_speed_kmh": max_speed, "tailgate_speed_kmh": tailgate})
        cavs.append({"name": "cav{}".format(i),
                     "spawn_position": [t.location.x, t.location.y, t.location.z + 0.3,
                                        t.rotation.roll, t.rotation.yaw, t.rotation.pitch],
                     "destination": [goal.x, goal.y, goal.z],
                     "v2x": {"communication_range": 120},
                     "behavior": {"max_speed": max_speed, "tailgate_speed": tailgate}})
    params.scenario.single_cav_list = cavs
    return add_current_time(params), behaviour


def wait_for_sweeps(cavs, frame, timeout=10.0):
    """Every CAV's LiDAR data of `frame` (OpenCDA keeps only the latest measurement, so wait for it)."""
    end = time.time() + timeout
    while True:
        frames = [c.perception_manager.lidar.frame for c in cavs]
        if all(f == frame for f in frames):
            return [np.array(c.perception_manager.lidar.data, dtype=np.float32) for c in cavs]
        if any(f > frame for f in frames) or time.time() > end:
            raise RuntimeError("LiDAR frames {} do not match the tick {}".format(frames, frame))
        time.sleep(0.002)


def ground_truth(world, names):
    out = []
    for a in world.get_actors().filter("vehicle.*"):
        bb, v = a.bounding_box, a.get_velocity()
        name = names.get(a.id, "other")
        out.append({"id": a.id, "type_id": a.type_id, "role": name.split("_")[0], "name": name,
                    "pose": pose_dict(a.get_transform()),
                    "extent": [bb.extent.x, bb.extent.y, bb.extent.z],
                    "center_offset": [bb.location.x, bb.location.y, bb.location.z],
                    "speed_mps": math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)})
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description="The paper's scenarios with OpenCDA, in this repository's format")
    parser.add_argument("--cavs", type=int, default=4, help="4, 6, 8 or 10 in the paper")
    parser.add_argument("--traffic", type=int, default=None, help="other vehicles; default: 20-40 at random")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--every", type=int, default=20, help="ticks between saved frames (20 = 1 per second)")
    parser.add_argument("--warmup", type=int, default=100, help="ticks before the first frame")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--out", default=os.path.join(ROOT, "data", "runs"))
    args = parser.parse_args(argv)
    rng = random.Random(args.seed)
    if args.traffic is None:
        args.traffic = rng.randint(20, 40)

    sys.path.insert(0, OPENCDA)
    import carla

    quiet_windows()
    import opencda.scenario_testing.utils.sim_api as sim_api
    from opencda.core.common.cav_world import CavWorld

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    town = os.path.basename(client.get_world().get_map().name)
    if town != "Town03":
        sys.exit("CARLA runs {}; start it on Town03 (this script never reloads the world)".format(town))
    params, behaviour = build_params(args, client.get_world().get_map(), rng)
    random.seed(args.seed)
    np.random.seed(args.seed)

    name = "{}_opencda_s{}_{}cav".format(time.strftime("%Y-%m-%d_%H%M%S"), args.seed, args.cavs)
    run_dir = os.path.join(args.out, name)
    os.makedirs(os.path.join(run_dir, "frames"))
    manager, cavs, traffic, tm = None, [], [], None
    try:
        manager = sim_api.ScenarioManager(params, False, "0.9.12", town=None, cav_world=CavWorld(False))
        world = manager.world
        cavs = manager.create_vehicle_manager(application=["single"], data_dump=False)
        tm, traffic = manager.create_traffic_carla()
        tm.set_random_device_seed(args.seed)
        tm_settings = []
        for v in traffic:     # the paper's ranges instead of OpenCDA's +-30 % around one value
            speed, lights = rng.uniform(-20, 20), rng.uniform(0, 100)
            tm.vehicle_percentage_speed_difference(v, speed)
            tm.ignore_lights_percentage(v, lights)
            tm_settings.append({"id": v.id, "speed_difference_pct": round(speed, 1),
                                "ignore_lights_pct": round(lights, 1)})
        names = {c.vehicle.id: "cav_{}".format(i) for i, c in enumerate(cavs)}
        names.update({v.id: "traffic_{}".format(i) for i, v in enumerate(traffic)})
        for i, s in enumerate(tm_settings):
            s["name"] = "traffic_{}".format(i)
        with open(os.path.join(run_dir, "spawn.json"), "w") as fh:
            json.dump(ground_truth(world, names), fh, indent=1)
        lidar = params.vehicle_base.sensing.perception.lidar
        config = {
            "server": {"town": town, "host": args.host, "port": args.port},
            "simulation": {"fixed_delta_seconds": 0.05, "seed": args.seed},
            "scenario": {"source": "OpenCDA (tools/opencda_scenario.py)", "num_cavs": len(cavs),
                         "num_traffic": len(traffic), "cav_blueprint": CAV_BLUEPRINT,
                         "junction_centre": list(JUNCTION), "spawn_bbox": list(BBOX)},
            "lidar": dict({k: lidar[k] for k in ("channels", "range", "points_per_second", "rotation_frequency",
                                                 "upper_fov", "lower_fov", "dropoff_general_rate",
                                                 "dropoff_intensity_limit", "dropoff_zero_intensity",
                                                 "noise_stddev")}, **LIDAR_MOUNT),
            "recording": {"warmup_ticks": args.warmup, "save_every": args.every, "num_frames": args.frames},
            "opencda": {"cav_behaviour": behaviour, "traffic_manager": tm_settings,
                        "destinations": [list(c["destination"]) for c in params.scenario.single_cav_list]},
        }
        with open(os.path.join(run_dir, "config.yaml"), "w") as fh:
            yaml.safe_dump(json.loads(json.dumps(config)), fh, sort_keys=False)
        print("Scenario: {} CAVs, {} other vehicles; run {}".format(len(cavs), len(traffic), run_dir), flush=True)

        tick, saved, reroutes = 0, 0, []
        carla_map = world.get_map()
        while saved < args.frames:
            manager.tick()
            frame = world.get_snapshot().frame
            for i, cav in enumerate(cavs):
                cav.update_info()
                try:
                    control = cav.run_step()
                except SystemExit:
                    # OpenCDA's agent ends the whole program when a CAV reaches its destination; here the CAV
                    # gets a new one ~120 m further along its lanes, so it keeps driving as in continuous traffic
                    here = cav.vehicle.get_location()
                    goal = route_end(carla_map.get_waypoint(here), rng)
                    cav.set_destination(here, goal, clean=True)
                    reroutes.append({"tick": tick, "cav": "cav_{}".format(i), "destination": [goal.x, goal.y, goal.z]})
                    print("cav_{} reached its destination at tick {}: new one {:.0f} m ahead".format(
                        i, tick, goal.distance(here)), flush=True)
                    control = cav.run_step()
                cav.vehicle.apply_control(control)
            tick += 1
            if tick <= args.warmup or (tick - args.warmup - 1) % args.every:
                continue
            sweeps = wait_for_sweeps(cavs, frame)
            folder = os.path.join(run_dir, "frames", "{:06d}".format(frame))
            os.makedirs(folder)
            meta = {"frame": frame, "timestamp": world.get_snapshot().timestamp.elapsed_seconds, "town": town,
                    "cavs": {}, "vehicles": ground_truth(world, names)}
            for i, (cav, points) in enumerate(zip(cavs, sweeps)):
                np.save(os.path.join(folder, "cav{}.npy".format(i)), points)
                meta["cavs"]["cav{}".format(i)] = {
                    "vehicle_id": cav.vehicle.id,
                    "sensor_pose": pose_dict(cav.perception_manager.lidar.sensor.get_transform()),
                    "mount_z": LIDAR_MOUNT["mount_z"], "num_points": int(len(points))}
            with open(os.path.join(folder, "meta.json"), "w") as fh:
                json.dump(meta, fh, indent=1)
            saved += 1
            speeds = [3.6 * v["speed_mps"] for v in meta["vehicles"] if v["name"].startswith("cav")]
            print("frame {} ({}/{}): points {}, CAV speeds {} km/h".format(
                frame, saved, args.frames, [len(p) for p in sweeps], [round(s) for s in speeds]), flush=True)
        if reroutes:
            with open(os.path.join(run_dir, "reroutes.json"), "w") as fh:
                json.dump(reroutes, fh, indent=1)
    finally:
        cleanup(cavs, traffic, tm, manager)
    print("Saved {} frames in {}".format(args.frames, run_dir), flush=True)


def cleanup(cavs, traffic, tm, manager):
    """Remove every actor and give CARLA back its asynchronous mode, whatever fails on the way."""
    steps = []
    for cav in cavs:
        # OpenCDA's VehicleManager.destroy() leaves the safety sensors (IMU, collision); some have no destroy()
        steps += [s.destroy for s in cav.safety_manager.sensors if hasattr(s, "destroy")]
        steps.append(cav.destroy)
    steps += [v.destroy for v in traffic]
    # the world back to asynchronous first, then the Traffic Manager: the other order (OpenCDA's) aborts the
    # process inside world.apply_settings on Windows with carla 0.9.12
    if manager is not None:
        steps.append(manager.close)          # CARLA's settings from before the scenario
    if tm is not None:
        steps.append(lambda: tm.set_synchronous_mode(False))
    for step in steps:
        try:
            step()
        except Exception as exc:              # keep going: the world must not stay in synchronous mode
            print("cleanup: {}: {}".format(getattr(step, "__qualname__", step), exc), flush=True)


if __name__ == "__main__":
    main()
