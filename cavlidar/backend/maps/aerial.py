"""
Aerial photo of the map loaded in the CARLA server.

A camera looks straight down from 1000 m with a narrow field of view, which is
close to an orthographic view (buildings barely lean). Tiles are taken on a
grid and pasted into one image with the same Georef as the road image, so the
two backgrounds line up.
"""
import math
import queue

import carla
import numpy as np
from PIL import Image

from ..simulator.sync import SyncMode
from .raster import Georef

ALTITUDE = 1000.0   # metres above the road
TILE_PX = 900       # pixels per tile side
SETTLE_TICKS = 3    # ticks after moving the camera before taking the image


def capture(client, world, bounds, ppm=None, tm_port=8000, progress=None):
    """
    Photograph the area `bounds` = (x_min, x_max, y_min, y_max).
    Returns (PIL image, Georef). `progress(done, total)` is called after each tile.
    """
    georef = Georef.for_bounds(bounds, ppm)
    tile_m = TILE_PX / georef.ppm
    cols = int(math.ceil(georef.width / float(TILE_PX)))
    rows = int(math.ceil(georef.height / float(TILE_PX)))
    ground_z = 0.0

    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(TILE_PX))
    bp.set_attribute("image_size_y", str(TILE_PX))
    bp.set_attribute("fov", str(math.degrees(2 * math.atan(tile_m / 2.0 / ALTITUDE))))

    mosaic = Image.new("RGB", (georef.width, georef.height))
    images = queue.Queue()
    with SyncMode(client, world, 0.05, tm_port=tm_port) as sync:
        camera = world.spawn_actor(bp, carla.Transform(carla.Location(z=ALTITUDE), carla.Rotation(pitch=-90)))
        try:
            camera.listen(images.put)
            for r in range(rows):
                for c in range(cols):
                    # yaw -90: image right = +x, image down = +y (same as the road image)
                    x = georef.x_min + (c + 0.5) * tile_m
                    y = georef.y_min + (r + 0.5) * tile_m
                    camera.set_transform(carla.Transform(carla.Location(x=x, y=y, z=ground_z + ALTITUDE),
                                                         carla.Rotation(pitch=-90, yaw=-90)))
                    for _ in range(SETTLE_TICKS):
                        frame = sync.tick()
                    image = _wait_for(images, frame)
                    mosaic.paste(_to_pil(image), (c * TILE_PX, r * TILE_PX))
                    if progress:
                        progress(r * cols + c + 1, rows * cols)
        finally:
            camera.destroy()
    return mosaic, georef


def _wait_for(images, frame, timeout=10.0):
    while True:
        image = images.get(timeout=timeout)
        if image.frame >= frame:
            return image


def _to_pil(image):
    bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
    return Image.fromarray(bgra[:, :, 2::-1].copy())
