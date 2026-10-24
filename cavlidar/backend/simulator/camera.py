"""Moving the spectator (the view shown in the CARLA window)."""
import carla


def top_view(world, x, y, z, height):
    """Place the spectator `height` metres above (x, y, z), looking straight down."""
    world.get_spectator().set_transform(carla.Transform(
        carla.Location(x=x, y=y, z=z + height), carla.Rotation(pitch=-90.0)))


def follow(world, actor, height):
    """Place the spectator above an actor (call it every tick to follow it)."""
    loc = actor.get_location()
    top_view(world, loc.x, loc.y, loc.z, height)
