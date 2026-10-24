"""Ray-cast LiDAR sensor: blueprint, mounting and frame-synchronised reading."""
import queue
from dataclasses import dataclass

import carla
import numpy as np

from ...core.constants import LIDAR_BLUEPRINT, SENSOR_TIMEOUT
from ...core.errors import SensorTimeout
from ..geometry.transforms import transform_to_dict


@dataclass
class LidarScan:
    """One full LiDAR measurement."""
    name: str
    frame: int
    timestamp: float
    points: np.ndarray       # (N, 4) float32: x, y, z, intensity in the sensor frame
    sensor_pose: dict        # sensor pose in the world (x, y, z, roll, pitch, yaw)


def build_blueprint(world, cfg):
    """Create a ray-cast LiDAR blueprint from a LidarConfig."""
    bp = world.get_blueprint_library().find(LIDAR_BLUEPRINT)
    attributes = {
        "channels": cfg.channels,
        "range": cfg.range,
        "points_per_second": cfg.points_per_second,
        "rotation_frequency": cfg.rotation_frequency,
        "upper_fov": cfg.upper_fov,
        "lower_fov": cfg.lower_fov,
        "dropoff_general_rate": cfg.dropoff_general_rate,
        "dropoff_intensity_limit": cfg.dropoff_intensity_limit,
        "dropoff_zero_intensity": cfg.dropoff_zero_intensity,
        "noise_stddev": cfg.noise_stddev,
    }
    for key, value in attributes.items():
        bp.set_attribute(key, str(value))
    return bp


def to_numpy(measurement):
    """Convert a carla.LidarMeasurement to an (N, 4) float32 array."""
    return np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4).copy()


class Lidar:
    """
    A LiDAR attached to a vehicle.

    CARLA delivers measurements on a background thread; they are queued and
    get(frame) returns the one belonging to a given simulation frame.
    """

    def __init__(self, world, vehicle, cfg, name):
        self.name = name
        self.vehicle = vehicle
        mount = carla.Transform(carla.Location(x=cfg.mount_x, y=cfg.mount_y, z=cfg.mount_z))
        self.actor = world.spawn_actor(build_blueprint(world, cfg), mount, attach_to=vehicle)
        self._queue = queue.Queue()
        self.actor.listen(self._queue.put)

    def get(self, frame, timeout=SENSOR_TIMEOUT):
        """Return the LidarScan of `frame`, discarding older measurements."""
        while True:
            try:
                m = self._queue.get(timeout=timeout)
            except queue.Empty:
                raise SensorTimeout("{}: no measurement for frame {} after {:.0f} s".format(
                    self.name, frame, timeout))
            if m.frame == frame:
                return LidarScan(self.name, m.frame, m.timestamp, to_numpy(m),
                                 transform_to_dict(m.transform))
            if m.frame > frame:
                raise SensorTimeout("{}: expected frame {}, got {}".format(self.name, frame, m.frame))

    def destroy(self):
        if self.actor is not None:
            self.actor.stop()
            self.actor.destroy()
            self.actor = None
