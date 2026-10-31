"""
Top-view images of a map, georeferenced so that world (x, y) <-> pixels.

Image axes follow CARLA seen from above (left-handed frame): x to the right,
y downwards, like the BEV plots of visualization/bev.py.
"""
import json
import math
import os
from dataclasses import asdict, dataclass

import carla
from PIL import Image, ImageDraw

MAX_IMAGE_SIDE = 4200   # pixels; large maps get fewer pixels per metre
MAX_PPM = 6.0           # pixels per metre for small maps

BACKGROUND = (30, 35, 40)
ROAD = (74, 79, 85)
SIDEWALK = (108, 112, 116)
SHOULDER = (60, 64, 69)
WHITE = (226, 228, 230)
YELLOW = (226, 185, 59)
ARROW = (138, 144, 150)

_SIDE_COLORS = {carla.LaneType.Sidewalk: SIDEWALK, carla.LaneType.Shoulder: SHOULDER,
                carla.LaneType.Parking: SHOULDER, carla.LaneType.Biking: SHOULDER}


@dataclass
class Georef:
    """Placement of an image in the world: pixel = (world - min) * ppm."""
    x_min: float
    y_min: float
    ppm: float
    width: int
    height: int

    @classmethod
    def for_bounds(cls, bounds, ppm=None):
        x_min, x_max, y_min, y_max = bounds
        if ppm is None:
            ppm = min(MAX_PPM, MAX_IMAGE_SIDE / max(x_max - x_min, y_max - y_min))
        return cls(x_min, y_min, ppm, int(math.ceil((x_max - x_min) * ppm)),
                   int(math.ceil((y_max - y_min) * ppm)))

    def to_px(self, x, y):
        return (x - self.x_min) * self.ppm, (y - self.y_min) * self.ppm

    def to_world(self, px, py):
        return self.x_min + px / self.ppm, self.y_min + py / self.ppm


def _quad(loc, yaw_deg, length, half_width, offset=0.0):
    """Corners of a rectangle centred on a lane sample, shifted `offset` metres to the right."""
    yaw = math.radians(yaw_deg)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rx, ry = -fy, fx  # right of the heading in CARLA's left-handed frame
    cx, cy = loc.x + rx * offset, loc.y + ry * offset
    h = length / 2.0
    return [(cx + fx * h + rx * half_width, cy + fy * h + ry * half_width),
            (cx + fx * h - rx * half_width, cy + fy * h - ry * half_width),
            (cx - fx * h - rx * half_width, cy - fy * h - ry * half_width),
            (cx - fx * h + rx * half_width, cy - fy * h + ry * half_width)]


def render_roads(road_map, georef):
    """Draw lanes, sidewalks, lane markings and driving-direction arrows."""
    img = Image.new("RGB", (georef.width, georef.height), BACKGROUND)
    draw = ImageDraw.Draw(img)
    px = lambda pts: [georef.to_px(x, y) for x, y in pts]  # noqa: E731
    step = 1.2  # slightly longer than the sample step, so no gaps

    # Side lanes (sidewalks, shoulders) first, from the outermost driving lane outwards
    for wp in road_map.lanes:
        side = wp.get_right_lane()
        if side is not None and side.lane_type == carla.LaneType.Driving:
            continue
        while side is not None and side.lane_type in _SIDE_COLORS:
            draw.polygon(px(_quad(side.transform.location, side.transform.rotation.yaw, step,
                                  side.lane_width / 2.0)), fill=_SIDE_COLORS[side.lane_type])
            side = side.get_right_lane()

    for wp in road_map.lanes:
        draw.polygon(px(_quad(wp.transform.location, wp.transform.rotation.yaw, step,
                              wp.lane_width / 2.0 + 0.05)), fill=ROAD)

    # Markings on the left edge of every lane (and the right edge of the outer one), outside junctions
    marking_w = max(1, int(round(0.15 * georef.ppm)))
    for wp in road_map.lanes:
        if wp.is_junction:
            continue
        edges = [(wp.left_lane_marking, -wp.lane_width / 2.0)]
        right = wp.get_right_lane()
        if right is None or right.lane_type != carla.LaneType.Driving:
            edges.append((wp.right_lane_marking, wp.lane_width / 2.0))
        for marking, offset in edges:
            kind = marking.type
            if kind == carla.LaneMarkingType.NONE:
                continue
            if kind == carla.LaneMarkingType.Broken and int(wp.s / 3.0) % 2:
                continue
            color = YELLOW if marking.color == carla.LaneMarkingColor.Yellow else WHITE
            line = _quad(wp.transform.location, wp.transform.rotation.yaw, step, 0.0, offset)
            draw.line(px([line[0], line[2]]), fill=color, width=marking_w)

    # Chevrons showing the driving direction, every 12 m
    for wp in road_map.lanes:
        if wp.is_junction or int(wp.s) % 12:
            continue
        yaw = math.radians(wp.transform.rotation.yaw)
        loc = wp.transform.location
        tip = (loc.x + math.cos(yaw) * 1.2, loc.y + math.sin(yaw) * 1.2)
        for side in (1, -1):
            back = (loc.x - math.cos(yaw) * 0.6 - side * math.sin(yaw) * 0.9,
                    loc.y - math.sin(yaw) * 0.6 + side * math.cos(yaw) * 0.9)
            draw.line(px([back, tip]), fill=ARROW, width=max(1, int(round(0.18 * georef.ppm))))
    return img


def save_image(img, georef, path):
    img.save(path)
    with open(os.path.splitext(path)[0] + ".json", "w", encoding="utf-8") as fh:
        json.dump(asdict(georef), fh, indent=2)


def load_image(path):
    """(PIL image, Georef) saved by save_image, or None if missing."""
    meta = os.path.splitext(path)[0] + ".json"
    if not (os.path.isfile(path) and os.path.isfile(meta)):
        return None
    with open(meta, "r", encoding="utf-8") as fh:
        georef = Georef(**json.load(fh))
    return Image.open(path).convert("RGB"), georef


def road_image(road_map, xodr_path, cache_dir):
    """Road image of a map, rendered once and cached next to the other map files."""
    path = os.path.join(cache_dir, "{}_roads.png".format(road_map.name))
    if os.path.isfile(path) and os.path.getmtime(path) >= os.path.getmtime(xodr_path):
        cached = load_image(path)
        if cached:
            return cached
    georef = Georef.for_bounds(road_map.bounds())
    img = render_roads(road_map, georef)
    os.makedirs(cache_dir, exist_ok=True)
    save_image(img, georef, path)
    return img, georef


def aerial_path(cache_dir, town):
    return os.path.join(cache_dir, "{}_aerial.png".format(town))
