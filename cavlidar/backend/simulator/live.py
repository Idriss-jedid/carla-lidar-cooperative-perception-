"""
Quick checks of a spawn plan in the running simulator (used by the editor).

Nothing here changes the world settings; spawned test vehicles are removed
before returning.
"""
import math

import carla

from ...core.enums import Role
from ..actors import vehicles
from ..actors.vehicles import SPAWN_LIFT

_COLORS = {Role.CAV: carla.Color(40, 120, 255), Role.TRAFFIC: carla.Color(255, 90, 50)}


def _transform(pose, lift=SPAWN_LIFT):
    return carla.Transform(carla.Location(x=pose["x"], y=pose["y"], z=pose["z"] + lift),
                           carla.Rotation(pitch=pose.get("pitch", 0.0), yaw=pose["yaw"]))


def draw_plan(world, planned, seconds=60.0, camera_height=80.0):
    """
    Draw each planned vehicle (box, heading arrow, name) in the CARLA window
    for `seconds` and put the spectator above them. `planned` = [(name, role, pose)].
    """
    if not planned:
        return
    for name, role, pose in planned:
        color = _COLORS.get(role, carla.Color(255, 255, 255))
        t = _transform(pose, lift=1.0)
        box = carla.BoundingBox(t.location, carla.Vector3D(2.3, 1.0, 0.8))
        world.debug.draw_box(box, t.rotation, thickness=0.15, color=color, life_time=seconds)
        yaw = math.radians(pose["yaw"])
        tip = carla.Location(x=t.location.x + 4 * math.cos(yaw), y=t.location.y + 4 * math.sin(yaw),
                             z=t.location.z)
        world.debug.draw_arrow(t.location, tip, thickness=0.15, arrow_size=0.6, color=color, life_time=seconds)
        world.debug.draw_string(carla.Location(x=t.location.x, y=t.location.y, z=t.location.z + 2.5),
                                name, draw_shadow=True, color=color, life_time=seconds)
    xs = [p["x"] for _, _, p in planned]
    ys = [p["y"] for _, _, p in planned]
    z = max(p["z"] for _, _, p in planned)
    world.get_spectator().set_transform(carla.Transform(
        carla.Location(x=(min(xs) + max(xs)) / 2, y=(min(ys) + max(ys)) / 2, z=z + camera_height),
        carla.Rotation(pitch=-90, yaw=-90)))


def test_spawn(client, world, planned, cav_blueprint="vehicle.tesla.model3"):
    """
    Try to spawn every planned vehicle, then remove them all.
    Returns the names whose pose is blocked (by a building, a parked car or another plan entry).
    """
    bp = world.get_blueprint_library().find(cav_blueprint)
    spawned, blocked = [], []
    try:
        for name, _, pose in planned:
            actor = world.try_spawn_actor(bp, _transform(pose))
            if actor is None:
                blocked.append(name)
            else:
                spawned.append(actor)
    finally:
        vehicles.destroy(client, spawned)
    return blocked
