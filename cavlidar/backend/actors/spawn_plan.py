"""
Spawn plans: JSON files that fix where some vehicles start (no CARLA needed here).

    {"map": "Town03",
     "vehicles": [{"role": "cav", "index": 0, "pose": {"x": .., "y": .., "z": .., "yaw": ..}}, ...]}

scenario.spawn_plan points to such a file. Planned vehicles are spawned first,
at their exact pose; if the config asks for more vehicles than the plan has,
the rest are spawned at random spawn points as usual.
"""
import json
import os

from ...core.enums import Role
from ...core.errors import ConfigError
from ..geometry.transforms import in_bbox

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def resolve_path(path):
    """A relative path is looked up in the current folder, then in the project folder."""
    if os.path.isabs(path) or os.path.exists(path):
        return path
    return os.path.join(PROJECT_ROOT, path)


def load_plan(path, town, num_cavs, num_traffic, bbox=None):
    """
    Read and check a plan. Returns {Role.CAV: [pose, ...], Role.TRAFFIC: [pose, ...]},
    each list ordered by index. Raises ConfigError if the plan does not fit the config.
    """
    full = resolve_path(path)
    if not os.path.isfile(full):
        raise ConfigError("scenario.spawn_plan file not found: {}".format(path))
    with open(full, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    if data.get("map") != town:
        raise ConfigError("Spawn plan {} is for {}, the config uses {}".format(path, data.get("map"), town))

    poses = {Role.CAV: [], Role.TRAFFIC: []}
    for vehicle in sorted(data.get("vehicles", []), key=lambda v: v["index"]):
        try:
            role = Role.parse(vehicle["role"])
        except ValueError:
            role = None
        if role not in poses:
            raise ConfigError("Spawn plan {}: role must be cav or traffic, got '{}'".format(path, vehicle["role"]))
        pose = vehicle["pose"]
        if bbox is not None and not in_bbox(pose["x"], pose["y"], bbox):
            raise ConfigError("Spawn plan {}: {}_{} at ({:.1f}, {:.1f}) is outside scenario.spawn_bbox".format(
                path, role, vehicle["index"], pose["x"], pose["y"]))
        poses[role].append(pose)

    if len(poses[Role.CAV]) > num_cavs or len(poses[Role.TRAFFIC]) > num_traffic:
        raise ConfigError("Spawn plan {} has {} CAVs and {} traffic vehicles, more than the config's "
                          "num_cavs={} and num_traffic={}".format(path, len(poses[Role.CAV]),
                                                                 len(poses[Role.TRAFFIC]), num_cavs, num_traffic))
    return poses


def read_plan(path):
    """The raw plan dict (map, source, vehicles) without checks against a config."""
    with open(resolve_path(path), "r", encoding="utf-8") as fh:
        return json.load(fh)


def save_plan(path, town, vehicles, source=""):
    """
    Write a plan. `vehicles` is a list of dicts with at least role, index and
    pose; extra keys (road_id, lane_id, ...) are kept.
    """
    folder = os.path.dirname(os.path.abspath(path))
    os.makedirs(folder, exist_ok=True)
    data = {"map": town, "source": source, "vehicles": vehicles}
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    return path
