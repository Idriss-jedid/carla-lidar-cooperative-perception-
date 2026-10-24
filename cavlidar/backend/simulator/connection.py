"""Connection to the CARLA server and map handling."""
import carla

from ...core.errors import MapMismatch, ServerUnavailable


def connect(host, port, timeout):
    """Open a client and make one round trip so an unreachable server fails here."""
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    try:
        client.get_server_version()
    except RuntimeError as exc:
        raise ServerUnavailable(
            "No CARLA server answered at {}:{} ({}). Start CarlaUE4.exe and wait until the "
            "city is visible.".format(host, port, exc))
    return client


def versions(client):
    """Return (client_version, server_version) as strings."""
    return client.get_client_version(), client.get_server_version()


def map_name(world):
    """Short map name, e.g. 'Town03' for 'Carla/Maps/Town03'."""
    return world.get_map().name.split("/")[-1]


def ensure_town(client, town, allow_map_change):
    """
    Return the world, making sure `town` is the loaded map.

    Changing map with load_world() crashed CARLA 0.9.12 on the 4 GB GPU used
    for this project, so it is only done when allow_map_change is True.
    The safer alternative is to set the default map in
    CarlaUE4/Config/DefaultEngine.ini and restart the server.
    """
    world = client.get_world()
    current = map_name(world)
    if current == town:
        return world
    if not allow_map_change:
        raise MapMismatch(
            "CARLA has map '{}' loaded but the config asks for '{}'. Either set the server's "
            "default map to {} in DefaultEngine.ini and restart CARLA, or set "
            "server.allow_map_change: true.".format(current, town, town))
    return client.load_world(town)
