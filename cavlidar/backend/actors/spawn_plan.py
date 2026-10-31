"""
Spawn plans: JSON files that fix where some vehicles start (no CARLA needed here).

    {"map": "Town03",
     "vehicles": [{"role": "cav", "index": 0, "pose": {"x": .., "y": .., "z": .., "yaw": ..}}, ...]}

scenario.spawn_plan points to such a file. Planned vehicles are spawned first,
at their exact pose; if the config asks for more vehicles than the plan has,
the rest are spawned at random spawn points as usual.

Optional keys of a vehicle:
    "speed_kmh": 30           drives at this speed (with the Traffic Manager if it has no route)
    "route": [[x, y], ...]    drives through these points along the lanes (route_driver.py)
    "route_end": "stop"       at the last point: "stop" there, or "autopilot" (keep driving)
"""
import json
import os
from dataclasses import dataclass
from typing import List, Optional

from ...core.enums import Role, RouteEnd
from ...core.errors import ConfigError
from ..geometry.transforms import in_bbox

DEFAULT_ROUTE_SPEED = 30.0  # km/h for a route without speed_kmh
MAX_SPEED = 150.0           # km/h


@dataclass
class PlannedSpawn:
    """One vehicle of a plan: where it starts and how it drives."""
    pose: dict
    speed_kmh: Optional[float] = None
    route: Optional[List[list]] = None
    route_end: RouteEnd = RouteEnd.STOP


def _parse_driving(path, name, vehicle):
    """speed_kmh, route and route_end of a plan entry, checked."""
    speed = vehicle.get("speed_kmh")
    if speed is not None and not (isinstance(speed, (int, float)) and 0 < speed <= MAX_SPEED):
        raise ConfigError("Spawn plan {}: {} speed_kmh must be in (0, {:.0f}], got {!r}".format(
            path, name, MAX_SPEED, speed))
    route = vehicle.get("route") or None
    if route is not None:
        if not all(isinstance(p, (list, tuple)) and len(p) in (2, 3)
                   and all(isinstance(c, (int, float)) for c in p) for p in route):
            raise ConfigError("Spawn plan {}: {} route must be a list of [x, y] points".format(path, name))
        route = [list(map(float, p)) for p in route]
        if speed is None:
            speed = DEFAULT_ROUTE_SPEED
    try:
        end = RouteEnd.parse(vehicle.get("route_end", RouteEnd.STOP))
    except ValueError as exc:
        raise ConfigError("Spawn plan {}: {} {}".format(path, name, exc))
    return speed, route, end

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def resolve_path(path):
    """A relative path is looked up in the current folder, then in the project folder."""
    if os.path.isabs(path) or os.path.exists(path):
        return path
    return os.path.join(PROJECT_ROOT, path)


def load_plan(path, town, num_cavs, num_traffic, bbox=None):
    """
    Read and check a plan. Returns {Role.CAV: [PlannedSpawn, ...], Role.TRAFFIC: [...]},
    each list ordered by index. Raises ConfigError if the plan does not fit the config.
    """
    full = resolve_path(path)
    if not os.path.isfile(full):
        raise ConfigError("scenario.spawn_plan file not found: {}".format(path))
    with open(full, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    if data.get("map") != town:
        raise ConfigError("Spawn plan {} is for {}, the config uses {}".format(path, data.get("map"), town))

    planned = {Role.CAV: [], Role.TRAFFIC: []}
    for vehicle in sorted(data.get("vehicles", []), key=lambda v: v["index"]):
        try:
            role = Role.parse(vehicle["role"])
        except ValueError:
            role = None
        if role not in planned:
            raise ConfigError("Spawn plan {}: role must be cav or traffic, got '{}'".format(path, vehicle["role"]))
        pose = vehicle["pose"]
        if bbox is not None and not in_bbox(pose["x"], pose["y"], bbox):
            raise ConfigError("Spawn plan {}: {}_{} at ({:.1f}, {:.1f}) is outside scenario.spawn_bbox".format(
                path, role, vehicle["index"], pose["x"], pose["y"]))
        speed, route, end = _parse_driving(path, role.name_of(vehicle["index"]), vehicle)
        planned[role].append(PlannedSpawn(pose, speed, route, end))

    if len(planned[Role.CAV]) > num_cavs or len(planned[Role.TRAFFIC]) > num_traffic:
        raise ConfigError("Spawn plan {} has {} CAVs and {} traffic vehicles, more than the config's "
                          "num_cavs={} and num_traffic={}".format(path, len(planned[Role.CAV]),
                                                                 len(planned[Role.TRAFFIC]), num_cavs, num_traffic))
    return planned


def read_plan(path):
    """The raw plan dict (map, source, vehicles) without checks against a config."""
    with open(resolve_path(path), "r", encoding="utf-8") as fh:
        return json.load(fh)


def save_plan(path, town, vehicles, source=""):
    """
    Write a plan. `vehicles` is a list of dicts with at least role, index and
    pose; extra keys (road_id, lane_id, speed_kmh, route, ...) are kept.
    """
    folder = os.path.dirname(os.path.abspath(path))
    os.makedirs(folder, exist_ok=True)
    data = {"map": town, "source": source, "vehicles": vehicles}
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    return path
