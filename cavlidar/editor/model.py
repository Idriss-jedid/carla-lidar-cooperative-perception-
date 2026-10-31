"""
The scenario being edited, independent of the window and of CARLA.

A ScenarioPlan is an ordered list of vehicles (role + pose, and optionally a
speed and a route). Names follow the order within each role: the first CAV is
cav_0, the second cav_1, ... Every change can be undone.
"""
import copy
import math
from dataclasses import dataclass, field, replace
from typing import List, Optional

from ..backend.actors import spawn_plan
from ..core.enums import Role, RouteEnd

VEHICLE_LENGTH = 4.8   # metres, footprint used for the overlap check
VEHICLE_WIDTH = 2.1
GAP = 0.6              # free space required between two footprints (m)
UNDO_LIMIT = 200


@dataclass
class PlannedVehicle:
    role: Role
    pose: dict                  # x, y, z, roll, pitch, yaw
    road_id: int = 0
    lane_id: int = 0
    in_junction: bool = False
    speed_kmh: Optional[float] = None    # None: Traffic Manager speed (or parked)
    route: List[list] = field(default_factory=list)  # [[x, y], ...] points to drive through, in order
    route_end: RouteEnd = RouteEnd.STOP


def footprint(pose, length=VEHICLE_LENGTH, width=VEHICLE_WIDTH):
    """Four (x, y) corners of a vehicle seen from above."""
    yaw = math.radians(pose["yaw"])
    fx, fy = math.cos(yaw), math.sin(yaw)
    rx, ry = -fy, fx
    hl, hw = length / 2.0, width / 2.0
    x, y = pose["x"], pose["y"]
    return [(x + fx * hl + rx * hw, y + fy * hl + ry * hw), (x + fx * hl - rx * hw, y + fy * hl - ry * hw),
            (x - fx * hl - rx * hw, y - fy * hl - ry * hw), (x - fx * hl + rx * hw, y - fy * hl + ry * hw)]


def overlap(pose_a, pose_b, gap=GAP):
    """True if two vehicle footprints (grown by gap/2 each) intersect (separating axis test)."""
    if abs(pose_a.get("z", 0.0) - pose_b.get("z", 0.0)) > 3.0:
        return False  # different levels (bridge / underpass)
    a = footprint(pose_a, VEHICLE_LENGTH + gap, VEHICLE_WIDTH + gap)
    b = footprint(pose_b, VEHICLE_LENGTH + gap, VEHICLE_WIDTH + gap)
    for poly in (a, b):
        for i in range(4):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % 4]
            nx, ny = y2 - y1, x1 - x2
            pa = [nx * x + ny * y for x, y in a]
            pb = [nx * x + ny * y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


@dataclass
class ScenarioPlan:
    town: str
    vehicles: List[PlannedVehicle] = field(default_factory=list)
    source: str = "Placed by hand in the scenario editor"
    _undo: list = field(default_factory=list, repr=False)
    _redo: list = field(default_factory=list, repr=False)

    # ------------------------------------------------------------------ queries
    def name(self, index):
        role = self.vehicles[index].role
        return role.name_of(sum(1 for v in self.vehicles[:index] if v.role == role))

    def counts(self):
        return {role: sum(1 for v in self.vehicles if v.role == role) for role in (Role.CAV, Role.TRAFFIC)}

    def collisions(self, pose, ignore=None):
        """Indices of vehicles whose footprint overlaps a vehicle at `pose`."""
        return [i for i, v in enumerate(self.vehicles) if i != ignore and overlap(pose, v.pose)]

    def outside(self, bbox):
        if bbox is None:
            return []
        return [i for i, v in enumerate(self.vehicles)
                if not (bbox[0] <= v.pose["x"] <= bbox[1] and bbox[2] <= v.pose["y"] <= bbox[3])]

    def hit(self, x, y):
        """Index of the vehicle under the world point (x, y), or None (last drawn wins)."""
        for i in reversed(range(len(self.vehicles))):
            p = self.vehicles[i].pose
            yaw = math.radians(p["yaw"])
            dx, dy = x - p["x"], y - p["y"]
            along = dx * math.cos(yaw) + dy * math.sin(yaw)
            across = -dx * math.sin(yaw) + dy * math.cos(yaw)
            if abs(along) <= VEHICLE_LENGTH / 2 + 0.5 and abs(across) <= VEHICLE_WIDTH / 2 + 0.5:
                return i
        return None

    def centre(self):
        """Mean (x, y) of the CAVs (of all vehicles if there is no CAV), or None."""
        pool = [v for v in self.vehicles if v.role == Role.CAV] or self.vehicles
        if not pool:
            return None
        return (sum(v.pose["x"] for v in pool) / len(pool), sum(v.pose["y"] for v in pool) / len(pool))

    def planned(self):
        """[(name, role, pose)] for drawing or test-spawning in CARLA."""
        return [(self.name(i), v.role, v.pose) for i, v in enumerate(self.vehicles)]

    # ------------------------------------------------------------------ edits
    def _snapshot(self):
        self._undo.append(copy.deepcopy(self.vehicles))
        del self._undo[:-UNDO_LIMIT]
        self._redo.clear()

    def add(self, vehicle: PlannedVehicle):
        self._snapshot()
        self.vehicles.append(vehicle)
        return len(self.vehicles) - 1

    def replace(self, index, vehicle: PlannedVehicle, record=True):
        """Change vehicle `index`; record=False while dragging (one undo step per drag)."""
        if record:
            self._snapshot()
        self.vehicles[index] = vehicle

    def begin_change(self):
        """Save an undo point before a series of replace(..., record=False)."""
        self._snapshot()

    def set_role(self, index, role):
        self._snapshot()
        self.vehicles[index] = replace(self.vehicles[index], role=role)

    def set_driving(self, index, **changes):
        """Change speed_kmh, route and/or route_end of vehicle `index` (one undo step)."""
        self._snapshot()
        self.vehicles[index] = replace(self.vehicles[index], **changes)

    def remove(self, index):
        self._snapshot()
        del self.vehicles[index]

    def clear(self):
        if self.vehicles:
            self._snapshot()
            self.vehicles = []

    def undo(self):
        if not self._undo:
            return False
        self._redo.append(copy.deepcopy(self.vehicles))
        self.vehicles = self._undo.pop()
        return True

    def redo(self):
        if not self._redo:
            return False
        self._undo.append(copy.deepcopy(self.vehicles))
        self.vehicles = self._redo.pop()
        return True

    # ------------------------------------------------------------------ files
    def to_records(self):
        """Plan entries for spawn_plan.save_plan, ordered as the vehicles are numbered."""
        records = []
        for i, v in enumerate(self.vehicles):
            record = {"role": v.role.value, "index": int(self.name(i).rsplit("_", 1)[1]),
                      "pose": dict(v.pose), "road_id": v.road_id, "lane_id": v.lane_id}
            if v.speed_kmh:
                record["speed_kmh"] = v.speed_kmh
            if v.route:
                record["route"] = [list(p) for p in v.route]
                record["route_end"] = v.route_end.value
            records.append(record)
        return records

    def save(self, path):
        return spawn_plan.save_plan(path, self.town, self.to_records(), self.source)

    @classmethod
    def load(cls, path):
        data = spawn_plan.read_plan(path)
        entries = sorted(data.get("vehicles", []), key=lambda e: (Role.parse(e["role"]) != Role.CAV, e["index"]))
        vehicles = [PlannedVehicle(Role.parse(e["role"]), dict(e["pose"]), e.get("road_id", 0),
                                   e.get("lane_id", 0), speed_kmh=e.get("speed_kmh"),
                                   route=[list(p) for p in e.get("route") or []],
                                   route_end=RouteEnd.parse(e.get("route_end", RouteEnd.STOP)))
                    for e in entries]
        return cls(data["map"], vehicles, data.get("source", ""))
