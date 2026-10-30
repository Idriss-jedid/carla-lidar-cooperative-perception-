"""Spawning vehicles, Traffic Manager behaviour and cleanup."""
import logging
import math

import carla

from ..geometry.transforms import in_bbox

log = logging.getLogger("cavlidar")


def candidate_spawn_points(world, rng, center=None, radius=None, bbox=None):
    """
    Return the map's spawn points in the order they should be tried.

    With a bbox [x_min, x_max, y_min, y_max]: points outside it are dropped
    (hard limit, paper Table 1 "Vehicle Spawn Area").
    Without a center: the remaining points, shuffled.
    With a center [x, y]: the points within `radius` metres come first
    (shuffled), followed by the remaining points from nearest to farthest.
    """
    points = list(world.get_map().get_spawn_points())
    if bbox is not None:
        points = [p for p in points if in_bbox(p.location.x, p.location.y, bbox)]
        log.info("%d spawn points inside bbox x[%.0f, %.0f] y[%.0f, %.0f]", len(points), *bbox)
    if center is None:
        rng.shuffle(points)
        return points

    cx, cy = center

    def dist(tf):
        return math.hypot(tf.location.x - cx, tf.location.y - cy)

    inside = [p for p in points if dist(p) <= radius]
    outside = sorted((p for p in points if dist(p) > radius), key=dist)
    rng.shuffle(inside)
    log.info("%d spawn points within %.0f m of (%.1f, %.1f)", len(inside), radius, cx, cy)
    return inside + outside


def _blueprints(world, pattern):
    """Four-wheel vehicle blueprints matching `pattern` (e.g. 'vehicle.*')."""
    bps = [bp for bp in world.get_blueprint_library().filter(pattern)
           if not bp.has_attribute("number_of_wheels")
           or int(bp.get_attribute("number_of_wheels")) == 4]
    if not bps:
        raise ValueError("No 4-wheel vehicle blueprint matches '{}'".format(pattern))
    return bps


def spawn_vehicles(world, count, pattern, candidates, rng, role):
    """
    Spawn up to `count` vehicles, consuming points from the front of `candidates`.

    `candidates` is modified in place, so successive calls (CAVs, then traffic)
    never reuse a spawn point. Vehicle i gets CARLA role_name role.name_of(i),
    e.g. 'cav_0'. Returns the list of spawned actors (may be shorter than count).
    """
    blueprints = _blueprints(world, pattern)
    vehicles = []
    while len(vehicles) < count and candidates:
        bp = rng.choice(blueprints)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", role.name_of(len(vehicles)))
        if bp.has_attribute("color"):
            bp.set_attribute("color", rng.choice(bp.get_attribute("color").recommended_values))
        vehicle = world.try_spawn_actor(bp, candidates.pop(0))
        if vehicle is not None:
            vehicles.append(vehicle)
    return vehicles


def enable_autopilot(vehicles, tm, speed_difference_range, ignore_lights_range, rng):
    """
    Hand the vehicles to the Traffic Manager with randomised behaviour.

    speed_difference: percentage BELOW the speed limit (CARLA convention):
        +20 -> 20 % slower than the limit, -20 -> 20 % faster.
    ignore_lights: percentage of chance to run a red light (0 = always obey).
    """
    for vehicle in vehicles:
        vehicle.set_autopilot(True, tm.get_port())
        tm.vehicle_percentage_speed_difference(vehicle, rng.uniform(*speed_difference_range))
        tm.ignore_lights_percentage(vehicle, rng.uniform(*ignore_lights_range))


def destroy(client, actors):
    """Destroy a list of actors (vehicles or sensors) in one batch."""
    actors = [a for a in actors if a is not None]
    if actors:
        client.apply_batch_sync([carla.command.DestroyActor(a) for a in actors])
