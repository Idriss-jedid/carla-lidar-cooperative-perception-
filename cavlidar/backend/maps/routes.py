"""
Routes: the lane path a vehicle drives from its start pose through its route points.

Works on any carla.Map (the server's, or one built from OpenDRIVE on the client),
so the editor draws exactly the path the vehicle follows in the simulation.
A route follows the lanes in their driving direction, may take any branch at a
junction, and changes to a neighbouring lane only where the markings allow it.

The speed profile says how fast the vehicle may go at every point of the path:
the chosen speed, less in sharp turns, braking smoothly to 0 at the end.
"""
import heapq
import math
from dataclasses import dataclass, field
from typing import List

import carla
import numpy as np

from ...core.errors import RouteError

STEP = 2.0                 # metres between path points
LANE_CHANGE_LENGTH = 8.0   # metres driven while moving to the next lane
LANE_CHANGE_COST = 10.0    # extra metres, so a lane change is used only when needed
MAX_NODES = 50000          # search limit (about 100 km of lanes)

LATERAL_ACCEL = 2.5        # m/s^2 allowed in turns (comfortable driving)
DECEL = 3.0                # m/s^2 for planned braking (end of route, red light, vehicle ahead)
CURVATURE_SPAN = 3         # path points on each side used to measure the curvature


@dataclass
class Route:
    """A drivable path: points every ~STEP m, with the lane of each point."""
    points: np.ndarray                      # (n, 3) x, y, z in the CARLA frame
    lanes: List[tuple] = field(default_factory=list)  # (road_id, lane_id) of each point

    @property
    def distances(self):
        """Distance along the path from the start to each point (m)."""
        steps = np.linalg.norm(np.diff(self.points[:, :2], axis=0), axis=1)
        return np.concatenate([[0.0], np.cumsum(steps)])

    @property
    def length(self):
        return float(self.distances[-1]) if len(self.points) > 1 else 0.0


def _location(x, y, z):
    return carla.Location(x=float(x), y=float(y), z=float(z))


def _driving_waypoint(carla_map, x, y, z):
    return carla_map.get_waypoint(_location(x, y, z), project_to_road=True, lane_type=carla.LaneType.Driving)


def _key(wp):
    return wp.road_id, wp.section_id, wp.lane_id, int(round(wp.s / (STEP / 2.0)))


def _neighbours(wp):
    """(next waypoint, cost in metres) reachable from wp."""
    for nxt in wp.next(STEP):
        yield nxt, STEP
    if wp.is_junction:
        return
    for allowed, side in ((carla.LaneChange.Left, wp.get_left_lane), (carla.LaneChange.Right, wp.get_right_lane)):
        if not wp.lane_change & allowed:
            continue
        lane = side()
        if lane is None or lane.lane_type != carla.LaneType.Driving or lane.lane_id * wp.lane_id <= 0:
            continue  # not a driving lane, or one going the other way
        for nxt in lane.next(LANE_CHANGE_LENGTH):
            yield nxt, LANE_CHANGE_LENGTH + LANE_CHANGE_COST


def _reached(wp, goal):
    return (wp.road_id == goal.road_id and wp.lane_id == goal.lane_id
            and wp.transform.location.distance(goal.transform.location) <= STEP / 2.0 + 0.05)


def _search(start, goal):
    """Waypoints from start to goal along the lanes (A*), or None if goal cannot be reached."""
    target = goal.transform.location
    counter = 0  # tie-breaker: waypoints cannot be compared
    queue = [(0.0, 0.0, counter, start, None)]
    parents = {}
    while queue and len(parents) < MAX_NODES:
        _, cost, _, wp, parent = heapq.heappop(queue)
        key = _key(wp)
        if key in parents:
            continue
        parents[key] = (wp, parent)
        if parent is not None and _reached(wp, goal):
            path = []
            while key is not None:
                wp, key = parents[key]
                path.append(wp)
            return path[::-1]
        for nxt, step in _neighbours(wp):
            if _key(nxt) not in parents:
                counter += 1
                h = nxt.transform.location.distance(target)
                heapq.heappush(queue, (cost + step + h, cost + step, counter, nxt, key))
    return None


def trace_route(carla_map, start, points):
    """
    Route from the pose `start` (dict with x, y, z) through every (x, y) or (x, y, z)
    of `points`, in order. Raises RouteError if a point cannot be reached by
    driving along the lanes.
    """
    if not points:
        raise RouteError("A route needs at least one point")
    wp = _driving_waypoint(carla_map, start["x"], start["y"], start.get("z", 0.0))
    if wp is None:
        raise RouteError("The start ({:.1f}, {:.1f}) is not on a driving lane".format(start["x"], start["y"]))
    path = [wp]
    for n, point in enumerate(points, 1):
        z = point[2] if len(point) > 2 else path[-1].transform.location.z
        goal = _driving_waypoint(carla_map, point[0], point[1], z)
        if goal is None:
            raise RouteError("Route point {} ({:.1f}, {:.1f}) is not on a driving lane".format(n, *point[:2]))
        leg = _search(path[-1], goal)
        if leg is None:
            raise RouteError("Route point {} ({:.1f}, {:.1f}) cannot be reached along the lanes from the "
                             "previous point (one-way road or wrong side?)".format(n, *point[:2]))
        path += leg[1:-1] + [goal]  # end exactly on the point
    xyz = np.array([[w.transform.location.x, w.transform.location.y, w.transform.location.z] for w in path])
    return Route(xyz, [(w.road_id, w.lane_id) for w in path])


def curvature(points, span=CURVATURE_SPAN):
    """Curvature (1/m) at each point, from the heading change over +-span points."""
    n = len(points)
    kappa = np.zeros(n)
    if n < 3:
        return kappa
    for i in range(n):
        a, b = max(0, i - span), min(n - 1, i + span)
        if b - a < 2:
            continue
        v1 = points[i, :2] - points[a, :2]
        v2 = points[b, :2] - points[i, :2]
        l1, l2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if l1 < 1e-6 or l2 < 1e-6:
            continue
        turn = math.atan2(v1[0] * v2[1] - v1[1] * v2[0], float(np.dot(v1, v2)))
        kappa[i] = abs(turn) / ((l1 + l2) / 2.0)
    return kappa


def speed_profile(route, speed_mps, stop_at_end=True, lateral_accel=LATERAL_ACCEL, decel=DECEL):
    """
    Highest speed (m/s) at each path point: the chosen speed, lower in turns
    (v^2 * curvature <= lateral_accel), reached by braking at most `decel`.
    """
    s = route.distances
    with np.errstate(divide="ignore"):
        limit = np.minimum(speed_mps, np.sqrt(lateral_accel / np.maximum(curvature(route.points), 1e-9)))
    if stop_at_end:
        limit[-1] = 0.0
    for i in range(len(limit) - 2, -1, -1):  # backwards: brake in time for every slower point
        limit[i] = min(limit[i], math.sqrt(limit[i + 1] ** 2 + 2.0 * decel * (s[i + 1] - s[i])))
    return limit
