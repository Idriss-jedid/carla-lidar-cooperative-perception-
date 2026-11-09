"""
Ego-body mask: drop the LiDAR returns that hit the CAV carrying the sensor.

Below about 2 m the sensor sees its own roof and hood (sensitivity analysis: 9 244 returns
per scan at a 1.6 m mount, 917 at 1.9 m, none at 2.4 m). Those points are real hits on the
CAV, not obstacles, so they must not reach the ground removal, the BEV or the sector quality.

Only two things are used, both fixed for a given CAV:
- the sensor-to-vehicle calibration: the mount pose (x, y, z, roll, pitch, yaw) of the LiDAR
  in the vehicle frame (lidar.mount_* of the run's config; our mounts have no rotation);
- the vehicle model's body box: half extents and centre offset in the vehicle frame
  (EGO_MODELS, measured once from CARLA's actor.bounding_box).
No pose, no frame-by-frame annotation, no other vehicle. Because the sensor is rigidly
attached, the body box is fixed in the sensor frame: the vehicle's pitch on a slope does
not change the mask.

The mask is the box grown by `margin` metres on every side. If the sensor itself lies inside
the body box, the mount is physically impossible (the sensor would be inside the car) and
EgoGeometry.mount_valid is False: masking cannot repair that recording.
"""
import logging
from dataclasses import dataclass, field

import numpy as np

from ..geometry.transforms import pose_to_matrix

log = logging.getLogger("cavlidar")

DEFAULT_MARGIN = 0.1   # metres added around the body box

# Body boxes of CAV models, vehicle frame (x forward, y right, z up from the ground), metres.
# Measured from CARLA 0.9.12 actor.bounding_box (the same in every recording of this project).
EGO_MODELS = {
    "vehicle.tesla.model3": {"extent": (2.3959, 1.0817, 0.7442), "center_offset": (0.0292, 0.0, 0.7355)},
}


@dataclass
class EgoGeometry:
    """Body box of the CAV and pose of its LiDAR, both in the vehicle frame."""
    extent: tuple
    center_offset: tuple
    mount: dict = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 2.4, "roll": 0.0, "pitch": 0.0,
                                                 "yaw": 0.0})
    source: str = ""

    @property
    def mount_valid(self):
        """False if the sensor origin is inside the body box (impossible mount)."""
        local = np.array([self.mount["x"], self.mount["y"], self.mount["z"]]) - np.array(self.center_offset)
        return not bool(np.all(np.abs(local) < np.array(self.extent)))

    @property
    def roof_clearance(self):
        """Height of the sensor above the top of the body box (m); negative = below the roof line."""
        return self.mount["z"] - (self.center_offset[2] + self.extent[2])

    def summary(self):
        return {"extent": list(self.extent), "center_offset": list(self.center_offset), "mount": dict(self.mount),
                "mount_valid": self.mount_valid, "roof_clearance_m": round(self.roof_clearance, 3),
                "source": self.source}


def geometry_for(blueprint, lidar_cfg, fallback_vehicle=None):
    """
    EgoGeometry of a CAV model with the LiDAR mount of a run's config ({mount_x, mount_y, mount_z,
    optional mount_roll / mount_pitch / mount_yaw}). An unknown model falls back to the static box of
    `fallback_vehicle` (its extent and centre offset only, never its pose); None if neither exists.
    """
    mount = {"x": float(lidar_cfg.get("mount_x", 0.0)), "y": float(lidar_cfg.get("mount_y", 0.0)),
             "z": float(lidar_cfg.get("mount_z", 2.4)), "roll": float(lidar_cfg.get("mount_roll", 0.0)),
             "pitch": float(lidar_cfg.get("mount_pitch", 0.0)), "yaw": float(lidar_cfg.get("mount_yaw", 0.0))}
    if blueprint in EGO_MODELS:
        box = EGO_MODELS[blueprint]
        return EgoGeometry(tuple(box["extent"]), tuple(box["center_offset"]), mount, "EGO_MODELS[{}]".format(blueprint))
    if fallback_vehicle is not None:
        return EgoGeometry(tuple(fallback_vehicle["extent"]), tuple(fallback_vehicle["center_offset"]), mount,
                           "recorded static box of {}".format(fallback_vehicle.get("type_id", "?")))
    return None


def ego_mask(points, geometry, margin=DEFAULT_MARGIN):
    """True for the points (sensor frame) inside the CAV's body box grown by `margin`."""
    xyz = np.asarray(points)[:, :3].astype(np.float64)
    m = pose_to_matrix(geometry.mount)                       # sensor -> vehicle frame
    vehicle = xyz @ m[:3, :3].T + m[:3, 3]
    local = np.abs(vehicle - np.array(geometry.center_offset))
    return np.all(local <= np.array(geometry.extent) + margin, axis=1)
