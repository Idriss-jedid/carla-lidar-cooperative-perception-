"""
The map CARLA loads when it starts (CarlaUE4/Config/DefaultEngine.ini).

Changing maps from a client with load_world() crashed CARLA on a 4 GB GPU;
setting the startup map and restarting CARLA is the safe way to switch.
"""
import os
import re
import shutil

INI = os.path.join("CarlaUE4", "Config", "DefaultEngine.ini")
_KEYS = ("GameDefaultMap", "ServerDefaultMap")


def _ini(carla_root):
    path = os.path.join(carla_root, INI)
    if not os.path.isfile(path):
        raise FileNotFoundError("CARLA settings not found: {}".format(path))
    return path


def get_startup_map(carla_root):
    """'Town03' from GameDefaultMap=/Game/Carla/Maps/Town03.Town03, or None."""
    with open(_ini(carla_root), "r", encoding="utf-8") as fh:
        match = re.search(r"^GameDefaultMap=.*/([^/.\s]+)\.\1\s*$", fh.read(), re.M)
    return match.group(1) if match else None


def set_startup_map(carla_root, town):
    """
    Make CARLA start on `town` (takes effect at the next CARLA start).
    The original file is kept once as DefaultEngine.ini.bak.
    """
    path = _ini(carla_root)
    backup = path + ".bak"
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
    with open(path, "r", encoding="utf-8") as fh:
        text = fh.read()
    value = "/Game/Carla/Maps/{0}.{0}".format(town)
    for key in _KEYS:
        text, count = re.subn(r"^{}=.*$".format(key), "{}={}".format(key, value), text, flags=re.M)
        if count == 0:
            raise ValueError("{} not found in {}".format(key, path))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)
    return path
