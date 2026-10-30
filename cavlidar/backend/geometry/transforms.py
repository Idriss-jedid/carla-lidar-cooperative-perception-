"""
Poses and coordinate transforms (numpy only, usable without CARLA installed).

CARLA / Unreal uses a LEFT-handed frame: x forward, y right, z up, angles in
degrees. A LiDAR point cloud is expressed in its sensor frame; multiply by the
sensor pose matrix to get world coordinates, which is what point-level
(early) fusion between CAVs needs.
"""
import math

import numpy as np


def transform_to_dict(transform):
    """carla.Transform -> plain dict (JSON-serialisable)."""
    loc, rot = transform.location, transform.rotation
    return {"x": loc.x, "y": loc.y, "z": loc.z,
            "roll": rot.roll, "pitch": rot.pitch, "yaw": rot.yaw}


def pose_to_matrix(pose):
    """Pose dict -> 4x4 sensor-to-world matrix (same formula as carla.Transform.get_matrix)."""
    cy, sy = math.cos(math.radians(pose["yaw"])), math.sin(math.radians(pose["yaw"]))
    cr, sr = math.cos(math.radians(pose["roll"])), math.sin(math.radians(pose["roll"]))
    cp, sp = math.cos(math.radians(pose["pitch"])), math.sin(math.radians(pose["pitch"]))
    return np.array([
        [cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr, pose["x"]],
        [sy * cp, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr, pose["y"]],
        [sp, -cp * sr, cp * cr, pose["z"]],
        [0.0, 0.0, 0.0, 1.0],
    ])


def sensor_to_world(points, pose):
    """Transform (N, >=3) points from the sensor frame to the world frame."""
    xyz1 = np.hstack([points[:, :3], np.ones((len(points), 1))])
    world = xyz1 @ pose_to_matrix(pose).T
    out = points.copy()
    out[:, :3] = world[:, :3]
    return out


def in_bbox(x, y, bbox):
    """True if (x, y) lies inside bbox = [x_min, x_max, y_min, y_max] (edges included)."""
    return bbox[0] <= x <= bbox[1] and bbox[2] <= y <= bbox[3]


def box_corners_2d(pose, extent, center_offset=(0.0, 0.0)):
    """Four (x, y) world corners of a vehicle bounding box seen from above."""
    yaw = math.radians(pose["yaw"])
    c, s = math.cos(yaw), math.sin(yaw)
    ex, ey = extent[0], extent[1]
    local = np.array([[ex, ey], [ex, -ey], [-ex, -ey], [-ex, ey]]) + np.array(center_offset[:2])
    rot = np.array([[c, -s], [s, c]])
    return local @ rot.T + np.array([pose["x"], pose["y"]])
