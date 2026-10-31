"""
A CARLA road network built on the client from an OpenDRIVE file.

carla.Map(name, xodr) gives the same lanes and waypoints as the server, but
needs no running simulator, so any map can be edited while CARLA runs
another one (or is closed).
"""
import math
from dataclasses import dataclass

import carla
import numpy as np

SAMPLE_STEP = 1.0  # metres between lane samples (drawing and z lookup)


def wrap_degrees(angle):
    """Angle in (-180, 180]."""
    angle = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if angle == -180.0 else angle


@dataclass
class LanePose:
    """A legal vehicle pose on a driving lane."""
    pose: dict          # x, y, z, roll, pitch, yaw (CARLA frame, degrees)
    road_id: int
    lane_id: int
    in_junction: bool
    distance: float     # metres between the requested point and the lane centre


class RoadMap:

    def __init__(self, name, xodr_text):
        self.name = name
        self.map = carla.Map(name, xodr_text)
        self.lanes = self.map.generate_waypoints(SAMPLE_STEP)
        self._xyz = np.array([[w.transform.location.x, w.transform.location.y, w.transform.location.z]
                              for w in self.lanes])

    @classmethod
    def from_file(cls, name, path):
        with open(path, "r", encoding="utf-8") as fh:
            return cls(name, fh.read())

    def bounds(self, margin=20.0):
        """(x_min, x_max, y_min, y_max) of the road network plus a margin."""
        lo, hi = self._xyz[:, :2].min(axis=0), self._xyz[:, :2].max(axis=0)
        return lo[0] - margin, hi[0] + margin, lo[1] - margin, hi[1] + margin

    def road_height(self, x, y):
        """z of the nearest lane sample: picks the right level where roads cross at different heights."""
        i = int(np.argmin((self._xyz[:, 0] - x) ** 2 + (self._xyz[:, 1] - y) ** 2))
        return float(self._xyz[i, 2])

    def snap(self, x, y, max_distance=6.0):
        """Nearest driving-lane pose to (x, y), heading along the lane; None if farther than max_distance."""
        wp = self.map.get_waypoint(carla.Location(x=x, y=y, z=self.road_height(x, y)),
                                   project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None:
            return None
        t = wp.transform
        distance = math.hypot(t.location.x - x, t.location.y - y)
        if distance > max_distance:
            return None
        pose = {"x": round(t.location.x, 2), "y": round(t.location.y, 2), "z": round(t.location.z, 2),
                "roll": 0.0, "pitch": round(wrap_degrees(t.rotation.pitch), 2),
                "yaw": round(wrap_degrees(t.rotation.yaw), 1)}
        return LanePose(pose, wp.road_id, wp.lane_id, wp.is_junction, distance)
