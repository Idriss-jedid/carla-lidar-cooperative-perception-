"""
Generate independently positioned, parked scenes around Town03 junction 1221 for the LiDAR
sensitivity benchmark (CARLA running Town03, carla012 env).

    python tools/make_sensitivity_scenes.py flat2 --kind flat --seed 2101
    python tools/make_sensitivity_scenes.py slope2 --kind slope --seed 2201
    python tools/make_sensitivity_scenes.py pop1 --kind population --seed 2301
    python tools/make_sensitivity_scenes.py moving1 --kind moving --seed 2401

Every vehicle is placed on a driving-lane waypoint (outside junction areas) drawn by a seeded
shuffle, so a scene is fully defined by its kind and seed; the result is frozen in a spawn plan
(exact x, y, z, pitch, yaw), and the recordings only read the plan. Vehicles may not overlap:
two vehicles conflict if they are closer than 6.3 m along the first one's heading AND 3.0 m
across it (a Model 3 is 4.8 x 2.2 m), so neighbouring lanes may hold vehicles side by side.

  flat        4 CAVs within 40 m of the junction centre + 11 vehicles within 70 m (the counts of
              the paper's Figure 2), all on flat lanes (|pitch| < 1 deg, off the sloped +x leg)
  slope       4 CAVs, at least 3 on the sloped +x leg (road 18, |pitch| >= 2 deg), + 6 vehicles,
              at least 4 on that leg; the others within 40 m of the centre
  population  a pool of 44 Tesla Model 3 (one model, so a vehicle is the same object whether it is
              a CAV or not): pool 0-9 are the CAV candidates (within 40 m), 10-43 traffic (within
              90 m, inside the paper's spawn box). Plans:
                <name>_c<k>.json    k = 4, 6, 8, 10 CAVs = pool 0..k-1, traffic = pool k..33:
                                    the SAME 34 vehicles, only the LiDAR roles change
                <name>_t<n>.json    4 CAVs + n = 20, 30, 40 traffic = pool 4..3+n (nested:
                                    t20 is part of t30, t30 of t40); t30 = c4
  moving      4 CAVs within 40 m + 20 vehicles within 90 m, mixed models, ALL driven by the Traffic
              Manager (synchronous mode, fixed seeds) and recorded after 6 s of driving. Kept apart from
              the parked scenes; tools/run_ablation.py checks that every run follows the same paths.

Writes config/spawn_plans/sensitivity/<name>*.json, config/experiments/ablation/scene_<name>.yaml
and registers the scene in config/experiments/ablation/scenes.json.
"""
import argparse
import json
import math
import os
import random

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLANS = os.path.join(ROOT, "config", "spawn_plans", "sensitivity")
ABLATION = os.path.join(ROOT, "config", "experiments", "ablation")
REGISTRY = os.path.join(ABLATION, "scenes.json")

CENTRE = (2.9, -136.5)                    # junction 1221
PAPER_BBOX = [-100.0, 100.0, -200.0, 200.0]
SLOPE_ROAD = 18                           # the +x leg, 0 -> 8 m over x 21..71
LENGTH_GAP, WIDTH_GAP = 6.3, 3.0          # conflict box between two vehicle centres (m)
CAV_COUNTS, TRAFFIC_COUNTS, POOL_CAVS, POOL = (4, 6, 8, 10), (20, 30, 40), 10, 44


def _angle(a):
    return (a + 180.0) % 360.0 - 180.0


def lane_slots(carla_map, step=1.0):
    """Driving-lane waypoints outside junctions, as plain dicts."""
    import carla
    out = []
    for wp in carla_map.generate_waypoints(step):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction:
            continue
        t = wp.transform
        x, y = t.location.x, t.location.y
        if not (PAPER_BBOX[0] <= x <= PAPER_BBOX[1] and PAPER_BBOX[2] <= y <= PAPER_BBOX[3]):
            continue
        out.append({"x": round(x, 2), "y": round(y, 2), "z": round(t.location.z, 2),
                    "pitch": round(_angle(t.rotation.pitch), 2), "yaw": round(_angle(t.rotation.yaw), 2),
                    "road_id": wp.road_id, "lane_id": wp.lane_id,
                    "dist": math.hypot(x - CENTRE[0], y - CENTRE[1])})
    out.sort(key=lambda s: (s["road_id"], s["lane_id"], s["x"], s["y"]))   # independent of CARLA's order
    return out


def conflicts(a, chosen):
    for b in chosen:
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        for ref in (a, b):
            h = math.radians(ref["yaw"])
            along = abs(dx * math.cos(h) + dy * math.sin(h))
            across = abs(-dx * math.sin(h) + dy * math.cos(h))
            if along < LENGTH_GAP and across < WIDTH_GAP:
                return True
    return False


def pick(slots, rng, count, chosen, keep):
    """Draw `count` slots satisfying `keep`, in seeded random order, without conflicts."""
    pool = [s for s in slots if keep(s)]
    rng.shuffle(pool)
    out = []
    for s in pool:
        if len(out) == count:
            break
        if not conflicts(s, chosen + out):
            out.append(s)
    if len(out) < count:
        raise SystemExit("only {} of {} positions fit".format(len(out), count))
    return out


def on_slope(s):
    return s["road_id"] == SLOPE_ROAD and abs(s["pitch"]) >= 2.0


def flat_ground(s):
    return abs(s["pitch"]) < 1.0 and s["road_id"] != SLOPE_ROAD


def make_flat(slots, rng):
    cavs = pick(slots, rng, 4, [], lambda s: flat_ground(s) and s["dist"] <= 40)
    traffic = pick(slots, rng, 11, cavs, lambda s: flat_ground(s) and s["dist"] <= 70)
    return cavs, traffic


def make_slope(slots, rng):
    cavs = pick(slots, rng, 3, [], on_slope)
    cavs += pick(slots, rng, 1, cavs, lambda s: s["dist"] <= 40)
    traffic = pick(slots, rng, 4, cavs, on_slope)
    traffic += pick(slots, rng, 2, cavs + traffic, lambda s: s["dist"] <= 40)
    return cavs, traffic


def make_moving(slots, rng):
    cavs = pick(slots, rng, 4, [], lambda s: s["dist"] <= 40)
    traffic = pick(slots, rng, 20, cavs, lambda s: s["dist"] <= 90)
    return cavs, traffic


def make_pool(slots, rng):
    cav_pool = pick(slots, rng, POOL_CAVS, [], lambda s: s["dist"] <= 40)
    rest = pick(slots, rng, POOL - POOL_CAVS, cav_pool, lambda s: s["dist"] <= 90)
    return cav_pool + rest


def entry(role, index, slot, pool_index=None):
    e = {"role": role, "index": index,
         "pose": {"x": slot["x"], "y": slot["y"], "z": slot["z"], "roll": 0.0,
                  "pitch": slot["pitch"], "yaw": slot["yaw"]},
         "road_id": slot["road_id"], "lane_id": slot["lane_id"]}
    if pool_index is not None:
        e["pool_index"] = pool_index
    return e


def write_plan(path, vehicles, source):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"map": "Town03", "source": source, "vehicles": vehicles}, fh, indent=2)
    return os.path.relpath(path, ROOT).replace(os.sep, "/")


def write_scene_yaml(name, kind, seed, plan, num_cavs, num_traffic, traffic_blueprint, moving=False):
    lines = [
        "# Sensitivity benchmark, generated scene '{}' ({}, seed {}) by tools/make_sensitivity_scenes.py.".format(
            name, kind, seed),
        ("# Every vehicle starts at the spawn plan's pose and is driven by the Traffic Manager; only the LiDAR"
         if moving else "# Every vehicle parked and braked at the spawn plan's pose; only the LiDAR (or, for population"),
        ("# changes between runs (the paths must be identical, checked after recording)."
         if moving else "# scenes, the plan variant) changes between runs."),
        "simulation:",
        "  seed: {}".format(seed),
        "scenario:",
        "  num_cavs: {}".format(num_cavs),
        "  num_traffic: {}".format(num_traffic),
        "  spawn_center: [{}, {}]".format(*CENTRE),
        "  spawn_bbox: [{:g}, {:g}, {:g}, {:g}]".format(*PAPER_BBOX),
        "  spawn_plan: {}".format(plan),
        "  traffic_blueprint: {}".format(traffic_blueprint),
        "  autopilot: {}".format("true" if moving else "false"),
        "recording:",
        "  warmup_ticks: {}".format(120 if moving else 40),
        "  num_frames: 5",
        "  save_every: 2",
        "  camera: none",
        "",
    ]
    path = os.path.join(ABLATION, "scene_{}.yaml".format(name))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    return os.path.relpath(path, ROOT).replace(os.sep, "/")


def register(name, info):
    reg = {}
    if os.path.exists(REGISTRY):
        with open(REGISTRY, encoding="utf-8") as fh:
            reg = json.load(fh)
    reg[name] = info
    with open(REGISTRY, "w", encoding="utf-8") as fh:
        json.dump(reg, fh, indent=2)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name")
    parser.add_argument("--kind", choices=["flat", "slope", "population", "moving"], required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    args = parser.parse_args(argv)

    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    carla_map = client.get_world().get_map()
    if not carla_map.name.endswith("Town03"):
        raise SystemExit("CARLA runs {}, Town03 needed".format(carla_map.name))
    slots = lane_slots(carla_map)
    rng = random.Random(args.seed)
    source = "tools/make_sensitivity_scenes.py {} --kind {} --seed {}".format(args.name, args.kind, args.seed)

    if args.kind in ("flat", "slope", "moving"):
        cavs, traffic = {"flat": make_flat, "slope": make_slope, "moving": make_moving}[args.kind](slots, rng)
        vehicles = [entry("cav", i, s) for i, s in enumerate(cavs)] + \
                   [entry("traffic", i, s) for i, s in enumerate(traffic)]
        plan = write_plan(os.path.join(PLANS, args.name + ".json"), vehicles, source)
        moving = args.kind == "moving"
        config = write_scene_yaml(args.name, args.kind, args.seed, plan, len(cavs), len(traffic), "vehicle.*", moving)
        register(args.name, {"config": config, "terrain": args.kind, "kind": "moving" if moving else "sensor",
                             "seed": args.seed})
        for v in vehicles:
            p = v["pose"]
            print("{:8s}{:2d}  x {:7.1f} y {:7.1f} z {:5.2f} pitch {:6.2f} yaw {:7.1f}  road {} lane {}".format(
                v["role"], v["index"], p["x"], p["y"], p["z"], p["pitch"], p["yaw"], v["road_id"], v["lane_id"]))
        return

    pool = make_pool(slots, rng)
    variants = {}
    base_traffic = 30
    for k in CAV_COUNTS:          # same 34 vehicles, k of them CAVs
        members = pool[:4 + base_traffic]
        vehicles = [entry("cav", i, s, i) for i, s in enumerate(members[:k])] + \
                   [entry("traffic", j, s, k + j) for j, s in enumerate(members[k:])]
        variants["c{}".format(k)] = (write_plan(os.path.join(PLANS, "{}_c{}.json".format(args.name, k)),
                                                vehicles, source + " (cavs={})".format(k)), k, len(members) - k)
    for n in TRAFFIC_COUNTS:      # 4 CAVs + n traffic, nested
        members = pool[:4 + n]
        vehicles = [entry("cav", i, s, i) for i, s in enumerate(members[:4])] + \
                   [entry("traffic", j, s, 4 + j) for j, s in enumerate(members[4:])]
        variants["t{}".format(n)] = (write_plan(os.path.join(PLANS, "{}_t{}.json".format(args.name, n)),
                                                vehicles, source + " (traffic={})".format(n)), 4, n)
    config = write_scene_yaml(args.name, args.kind, args.seed, variants["c4"][0], 4, base_traffic,
                              "vehicle.tesla.model3")
    register(args.name, {"config": config, "terrain": "junction", "kind": "population", "seed": args.seed,
                         "variants": {k: {"spawn_plan": v[0], "num_cavs": v[1], "num_traffic": v[2]}
                                      for k, v in variants.items()}})
    slope_n = sum(on_slope(s) for s in pool)
    print("pool of {}: {} CAV candidates within 40 m, {} on the sloped leg; variants {}".format(
        len(pool), POOL_CAVS, slope_n, ", ".join(variants)))


if __name__ == "__main__":
    main()
