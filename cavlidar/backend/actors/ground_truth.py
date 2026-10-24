"""Ground-truth vehicle annotations (used later to compute AP@0.5)."""
from ...core.enums import Role
from ..geometry.transforms import transform_to_dict


def collect_vehicles(world, role_names):
    """
    Describe every vehicle in the world at the current frame.

    role_names: {actor_id: 'cav_0' | 'traffic_3' | ...}; others are Role.OTHER.
    Returns a list of dicts with pose, box extent (half sizes, metres),
    box centre offset in the vehicle frame and speed (m/s).
    """
    vehicles = []
    for actor in world.get_actors().filter("vehicle.*"):
        bb = actor.bounding_box
        v = actor.get_velocity()
        name = role_names.get(actor.id, Role.OTHER.value)
        vehicles.append({
            "id": actor.id,
            "type_id": actor.type_id,
            "role": Role.of(name).value,
            "name": name,
            "pose": transform_to_dict(actor.get_transform()),
            "extent": [bb.extent.x, bb.extent.y, bb.extent.z],
            "center_offset": [bb.location.x, bb.location.y, bb.location.z],
            "speed_mps": (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5,
        })
    return vehicles
