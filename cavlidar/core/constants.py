"""Fixed names used across the project (not meant to be tuned)."""

# CARLA blueprint ids
LIDAR_BLUEPRINT = "sensor.lidar.ray_cast"

# Run folder layout (see backend/storage.py)
FRAMES_DIR = "frames"
META_FILE = "meta.json"
CONFIG_FILE = "config.yaml"
FRAME_DIR_FORMAT = "{:06d}"
PREVIEW_SENSOR = "preview_sensor.png"
PREVIEW_WORLD = "preview_world.png"

# Name of the point-cloud file of CAV i inside a frame folder
CAV_CLOUD_NAME = "cav{}"

# Seconds to wait for one LiDAR measurement before giving up
SENSOR_TIMEOUT = 30.0
