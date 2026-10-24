"""
cavlidar - LiDAR data collection in CARLA for cooperative perception between
connected and autonomous vehicles (CAVs).

Layers:
    cavlidar.api      simple public functions (what scripts call)
    cavlidar.core     config (all parameters), enums, constants, errors
    cavlidar.backend  implementation (simulator, actors, recording, geometry, visualization)
"""
from .api import check_connection, drive_cars, load_config, record_lidar, show_frame
from .core.enums import CameraMode, PointFormat, Role, ViewMode
from .core.errors import CavLidarError

__all__ = [
    "load_config", "check_connection", "drive_cars", "record_lidar", "show_frame",
    "CameraMode", "PointFormat", "Role", "ViewMode", "CavLidarError",
]
__version__ = "0.1.0"
