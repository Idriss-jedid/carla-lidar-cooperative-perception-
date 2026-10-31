"""Project-specific exceptions, so callers can tell what went wrong."""


class CavLidarError(Exception):
    """Base class of every error raised on purpose by this package."""


class ConfigError(CavLidarError, ValueError):
    """A parameter is unknown or has an impossible value."""


class ServerUnavailable(CavLidarError):
    """The CARLA server could not be reached (not started, still loading, wrong port)."""


class MapMismatch(CavLidarError):
    """The server has another map loaded than the one the config asks for."""


class SpawnError(CavLidarError):
    """Not enough vehicles could be spawned."""


class SensorTimeout(CavLidarError):
    """A sensor did not deliver its measurement in time."""


class RouteError(ConfigError):
    """A vehicle's route cannot be driven along the lanes (wrong way, unconnected roads)."""
