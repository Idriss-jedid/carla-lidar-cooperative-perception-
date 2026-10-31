"""
Which CARLA maps are available.

A map can be *edited* when its OpenDRIVE file (.xodr, the road network) is
known, and *simulated* only when its level (.umap) is installed in CARLA.
The CARLA package ships both under CarlaUE4/Content/Carla/Maps/. OpenDRIVE
files fetched from a running server are cached in data/maps/ as well.
"""
import os
from dataclasses import dataclass
from typing import List, Optional

MAPS_SUBDIR = os.path.join("CarlaUE4", "Content", "Carla", "Maps")
OPENDRIVE_SUBDIR = os.path.join(MAPS_SUBDIR, "OpenDrive")


@dataclass
class MapInfo:
    name: str
    xodr_path: str
    installed: bool  # the level exists, so the CARLA server can run this map

    @property
    def label(self):
        return self.name if self.installed else "{}  (not installed: edit only)".format(self.name)


def is_carla_root(path):
    return bool(path) and os.path.isdir(os.path.join(path, OPENDRIVE_SUBDIR))


def find_carla_root(*hints):
    """First of `hints` (then $CARLA_ROOT) that contains the CARLA maps, else None."""
    for path in list(hints) + [os.environ.get("CARLA_ROOT")]:
        if is_carla_root(path):
            return path
    return None


def list_maps(carla_root=None, cache_dir=None) -> List[MapInfo]:
    """
    All maps with an OpenDRIVE file, sorted by name. The '_Opt' (layered)
    variants are skipped: they share the road network of the base map.
    """
    found = {}
    if cache_dir and os.path.isdir(cache_dir):
        for f in os.listdir(cache_dir):
            if f.endswith(".xodr"):
                found[f[:-5]] = MapInfo(f[:-5], os.path.join(cache_dir, f), installed=False)
    if is_carla_root(carla_root):
        xodr_dir = os.path.join(carla_root, OPENDRIVE_SUBDIR)
        levels = os.path.join(carla_root, MAPS_SUBDIR)
        for f in os.listdir(xodr_dir):
            name = f[:-5]
            if not f.endswith(".xodr") or name.endswith("_Opt"):
                continue
            installed = os.path.isfile(os.path.join(levels, name + ".umap"))
            found[name] = MapInfo(name, os.path.join(xodr_dir, f), installed)
    return [found[k] for k in sorted(found)]


def find_map(maps, name) -> Optional[MapInfo]:
    return next((m for m in maps if m.name == name), None)


def cache_server_map(world, cache_dir):
    """Save the OpenDRIVE of the map loaded in the server; returns its MapInfo."""
    name = os.path.basename(world.get_map().name)
    os.makedirs(cache_dir, exist_ok=True)
    path = os.path.join(cache_dir, name + ".xodr")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(world.get_map().to_opendrive())
    return MapInfo(name, path, installed=True)
