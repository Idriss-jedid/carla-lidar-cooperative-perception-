"""
Configuration of the whole project.

Every tunable parameter is a field of one of the dataclasses below. Values are
read in this order (later wins):

    1. the defaults written in this file
    2. config/default.yaml
    3. any experiment file (config/experiments/*.yaml)
    4. command-line overrides:  --set lidar.channels=64

Unknown keys and invalid values raise ConfigError, so a typo in a YAML file is
never silently ignored. See docs/PARAMETERS.md for what each parameter does.
"""
from dataclasses import dataclass, field, fields
from enum import Enum
from typing import List, Optional

import yaml

from .constants import TM_START_SECONDS
from .enums import CameraMode, PointFormat
from .errors import ConfigError


@dataclass
class ServerConfig:
    host: str = "127.0.0.1"
    port: int = 2000
    tm_port: int = 8000
    timeout: float = 60.0
    town: str = "Town03"
    allow_map_change: bool = False
    carla_root: Optional[str] = None


@dataclass
class SimulationConfig:
    fixed_delta_seconds: float = 0.05
    no_rendering: bool = False
    seed: int = 7


@dataclass
class ScenarioConfig:
    num_cavs: int = 4
    num_traffic: int = 20
    cav_blueprint: str = "vehicle.tesla.model3"
    traffic_blueprint: str = "vehicle.*"
    spawn_center: Optional[List[float]] = None
    spawn_radius: float = 80.0
    spawn_bbox: Optional[List[float]] = None
    spawn_plan: Optional[str] = None
    autopilot: bool = True
    speed_difference_range: List[float] = field(default_factory=lambda: [-20.0, 20.0])
    ignore_lights_range: List[float] = field(default_factory=lambda: [0.0, 0.0])


@dataclass
class LidarConfig:
    channels: int = 32
    range: float = 120.0
    points_per_second: int = 1000000
    rotation_frequency: float = 20.0
    upper_fov: float = 2.0
    lower_fov: float = -25.0
    mount_x: float = 0.0
    mount_y: float = 0.0
    mount_z: float = 2.4
    dropoff_general_rate: float = 0.45
    dropoff_intensity_limit: float = 0.8
    dropoff_zero_intensity: float = 0.4
    noise_stddev: float = 0.0


@dataclass
class RecordingConfig:
    output_dir: str = "data/runs"
    warmup_ticks: int = 100
    num_frames: int = 10
    save_every: int = 1
    formats: List[PointFormat] = field(default_factory=lambda: [PointFormat.NPY])
    save_ground_truth: bool = True
    camera: CameraMode = CameraMode.FOLLOW
    camera_height: float = 40.0
    preview: bool = True


@dataclass
class Config:
    server: ServerConfig = field(default_factory=ServerConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    scenario: ScenarioConfig = field(default_factory=ScenarioConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    recording: RecordingConfig = field(default_factory=RecordingConfig)

    def to_dict(self):
        """Plain nested dict (enums turned into their text value), ready for YAML."""
        return {f.name: {k: _plain(v) for k, v in vars(getattr(self, f.name)).items()}
                for f in fields(self)}


# --------------------------------------------------------------------------- loading

def load_config(*paths, overrides=None):
    """
    Build a validated Config from defaults + YAML files + overrides.

        cfg = load_config("config/default.yaml", "config/experiments/e02.yaml",
                          overrides=["lidar.channels=64"])
    """
    data = Config().to_dict()
    for path in paths:
        if path is None:
            continue
        with open(path, "r", encoding="utf-8") as fh:
            _merge(data, yaml.safe_load(fh) or {})
    for text in overrides or []:
        _merge(data, _parse_override(text))
    cfg = _from_dict(data)
    validate(cfg)
    return cfg


def save_config(cfg, path):
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(cfg.to_dict(), fh, sort_keys=False)


def validate(cfg):
    """Raise ConfigError on impossible settings; return a list of warnings."""
    errors, warnings = [], []
    sc, li, sim, rec = cfg.scenario, cfg.lidar, cfg.simulation, cfg.recording

    if sc.num_cavs < 0 or sc.num_traffic < 0:
        errors.append("scenario.num_cavs and scenario.num_traffic must be >= 0")
    if sc.spawn_radius <= 0:
        errors.append("scenario.spawn_radius must be > 0")
    if sc.spawn_center is not None and len(sc.spawn_center) != 2:
        errors.append("scenario.spawn_center must be [x, y] or null")
    if sc.spawn_bbox is not None and (len(sc.spawn_bbox) != 4 or sc.spawn_bbox[0] >= sc.spawn_bbox[1]
                                      or sc.spawn_bbox[2] >= sc.spawn_bbox[3]):
        errors.append("scenario.spawn_bbox must be [x_min, x_max, y_min, y_max] or null")
    for name in ("speed_difference_range", "ignore_lights_range"):
        lo_hi = getattr(sc, name)
        if len(lo_hi) != 2 or lo_hi[0] > lo_hi[1]:
            errors.append("scenario.{} must be [low, high] with low <= high".format(name))
    if not 0 <= sc.ignore_lights_range[0] <= sc.ignore_lights_range[-1] <= 100:
        errors.append("scenario.ignore_lights_range must stay within [0, 100]")

    if li.channels < 1 or li.range <= 0 or li.points_per_second <= 0 or li.rotation_frequency <= 0:
        errors.append("lidar.channels, range, points_per_second, rotation_frequency must be > 0")
    if li.lower_fov >= li.upper_fov:
        errors.append("lidar.lower_fov must be smaller than lidar.upper_fov")
    for name in ("dropoff_general_rate", "dropoff_intensity_limit", "dropoff_zero_intensity"):
        if not 0.0 <= getattr(li, name) <= 1.0:
            errors.append("lidar.{} must be between 0 and 1".format(name))
    if li.noise_stddev < 0:
        errors.append("lidar.noise_stddev must be >= 0")

    if sim.fixed_delta_seconds <= 0 or sim.fixed_delta_seconds > 0.1:
        errors.append("simulation.fixed_delta_seconds must be in (0, 0.1] (CARLA physics limit)")
    if rec.save_every < 1 or rec.num_frames < 1 or rec.warmup_ticks < 0:
        errors.append("recording.save_every and num_frames must be >= 1, warmup_ticks >= 0")
    if not rec.formats:
        errors.append("recording.formats must contain at least one format")
    if rec.camera == CameraMode.TOP and sc.spawn_center is None:
        errors.append("recording.camera 'top' needs scenario.spawn_center")
    if errors:
        raise ConfigError("Invalid configuration:\n  - " + "\n  - ".join(errors))

    sweep = li.rotation_frequency * sim.fixed_delta_seconds
    if abs(sweep - 1.0) > 1e-6:
        warnings.append(
            "Each LiDAR scan covers {:.0f} degrees (rotation_frequency x fixed_delta_seconds = "
            "{:.2f}). Use fixed_delta_seconds = 1 / rotation_frequency for full 360-degree "
            "sweeps.".format(360.0 * min(sweep, 1.0), sweep))
    warmup = rec.warmup_ticks * sim.fixed_delta_seconds
    if sc.autopilot and warmup < TM_START_SECONDS:
        warnings.append(
            "Warm-up is only {:.2f} s: the Traffic Manager needs about {:.0f} s to start the "
            "vehicles, so the first frames will show parked cars. Use recording.warmup_ticks >= "
            "{:.0f}.".format(warmup, TM_START_SECONDS, TM_START_SECONDS / sim.fixed_delta_seconds))
    return warnings


# --------------------------------------------------------------------------- helpers

def _plain(value):
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, list):
        return [_plain(v) for v in value]
    return value


def _merge(base, update, path=""):
    """Recursively merge `update` into `base`, rejecting unknown keys."""
    if not isinstance(update, dict):
        raise ConfigError("Expected a mapping at '{}', got {!r}".format(path.rstrip(".") or "<root>", update))
    for key, value in update.items():
        where = path + str(key)
        if key not in base:
            raise ConfigError("Unknown parameter '{}'. Valid keys here: {}".format(
                where, ", ".join(sorted(base))))
        if isinstance(base[key], dict):
            _merge(base[key], value, where + ".")
        else:
            base[key] = value
    return base


def _parse_override(text):
    """'lidar.channels=64' -> {'lidar': {'channels': 64}} (value parsed as YAML)."""
    if "=" not in text:
        raise ConfigError("Override must look like section.key=value, got '{}'".format(text))
    dotted, raw = text.split("=", 1)
    keys = dotted.strip().split(".")
    result = node = {}
    for key in keys[:-1]:
        node = node.setdefault(key, {})
    node[keys[-1]] = yaml.safe_load(raw)
    return result


def _from_dict(data):
    try:
        rec = dict(data["recording"])
        formats = rec["formats"] if isinstance(rec["formats"], list) else [rec["formats"]]
        rec["formats"] = [PointFormat.parse(f) for f in formats]
        rec["camera"] = CameraMode.parse(rec["camera"])
        return Config(
            server=ServerConfig(**data["server"]),
            simulation=SimulationConfig(**data["simulation"]),
            scenario=ScenarioConfig(**data["scenario"]),
            lidar=LidarConfig(**data["lidar"]),
            recording=RecordingConfig(**rec),
        )
    except ValueError as exc:
        raise ConfigError(str(exc))
