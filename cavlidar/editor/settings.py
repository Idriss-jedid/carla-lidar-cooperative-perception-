"""Where the editor finds CARLA's maps, and the small settings it remembers between runs."""
import json
import os

from ..backend.actors.spawn_plan import PROJECT_ROOT
from ..backend.maps import catalog

MAPS_CACHE = os.path.join(PROJECT_ROOT, "data", "maps")
SETTINGS_FILE = os.path.join(PROJECT_ROOT, "data", "editor_settings.json")


def load_settings():
    try:
        with open(SETTINGS_FILE, "r", encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def save_settings(settings):
    os.makedirs(os.path.dirname(SETTINGS_FILE), exist_ok=True)
    with open(SETTINGS_FILE, "w", encoding="utf-8") as fh:
        json.dump(settings, fh, indent=2)


def carla_root(cfg, settings=None):
    """server.carla_root, then the folder remembered by the editor, then $CARLA_ROOT."""
    settings = load_settings() if settings is None else settings
    return catalog.find_carla_root(cfg.server.carla_root, settings.get("carla_root"))


def available_maps(cfg, settings=None):
    return catalog.list_maps(carla_root(cfg, settings), MAPS_CACHE)


def road_map(cfg, town):
    """RoadMap of `town` from its OpenDRIVE file, or None if the file is not found."""
    from ..backend.maps.road_map import RoadMap

    info = catalog.find_map(available_maps(cfg), town)
    return RoadMap.from_file(info.name, info.xodr_path) if info else None
