"""
Enumerations: every option that can only take a fixed set of values.

They inherit from `str`, so YAML files and the command line simply use the
lowercase text value (e.g. camera: follow) and the code compares against
CameraMode.FOLLOW instead of a raw string.
"""
from enum import Enum


class _StrEnum(str, Enum):
    @classmethod
    def parse(cls, value):
        """Convert 'follow' / CameraMode.FOLLOW to the enum, with a clear error message."""
        if isinstance(value, cls):
            return value
        try:
            return cls(str(value).lower())
        except ValueError:
            raise ValueError("'{}' is not a valid {}. Choose one of: {}".format(
                value, cls.__name__, ", ".join(m.value for m in cls)))

    def __str__(self):
        return self.value


class CameraMode(_StrEnum):
    """What the CARLA window shows while a script runs."""
    FOLLOW = "follow"   # camera above CAV 0, moving with it
    TOP = "top"         # fixed camera above scenario.spawn_center
    NONE = "none"       # do not move the camera


class PointFormat(_StrEnum):
    """File format(s) for saved point clouds."""
    NPY = "npy"         # numpy binary: fastest, for Python processing
    PLY = "ply"         # binary PLY: opens in MeshLab / CloudCompare / Open3D


class Role(_StrEnum):
    """Role of a vehicle in the scenario."""
    CAV = "cav"         # connected & autonomous vehicle, carries a LiDAR
    TRAFFIC = "traffic" # non-connected vehicle, no sensor
    OTHER = "other"     # vehicle not created by this project

    def name_of(self, index):
        """Role.CAV.name_of(2) -> 'cav_2' (used as CARLA role_name)."""
        return "{}_{}".format(self.value, index)

    @classmethod
    def of(cls, role_name):
        """'cav_2' -> Role.CAV ; unknown -> Role.OTHER."""
        prefix = str(role_name).split("_")[0]
        return cls(prefix) if prefix in (m.value for m in cls) else cls.OTHER


class ViewMode(_StrEnum):
    """Which plots `show` produces for a recorded frame."""
    SENSOR = "sensor"   # one panel per CAV, in its own LiDAR frame
    WORLD = "world"     # all CAVs fused in the world frame + ground-truth boxes
    BOTH = "both"
