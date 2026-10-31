"""
Build config/spawn_plans/figure2_town03.json: the vehicles of the paper's
Figure 2 placed on the same Town03 junction.

How the positions were obtained
-------------------------------
1. Figure 2 is a top view of Town03 junction 1221 (centre x=2.9, y=-136.5):
   same buildings at the same corners, and the leg towards +x climbs from
   z=0 to z=8 m (Table 1: "four-leg, one leg sloped").
2. The figure draws one box per vehicle (4 "Ego" = CAVs, 11 "Other").
   Box centres were read from the PDF (page points) and converted to world
   x, y with the junction box as calibration: 13.14 figure pixels per metre,
   figure up = +x, figure right = +y.
3. Each position is snapped to the nearest driving lane whose direction
   matches the box orientation (wide box = along y, tall box = along x),
   so the heading is a legal lane heading.

Usage (CARLA running on Town03):
    python tools/make_figure2_plan.py
"""
import json
import math
import os

import carla

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT = os.path.join(ROOT, "config", "spawn_plans", "figure2_town03.json")

JUNCTION_ID = 1221
# Figure 2 image on page 8 of the PDF (points) and its size in pixels
PAGE_BOX = (327.979, 62.008, 508.268, 194.237)
FIG_SIZE = (1365, 990)
# Junction centre in figure pixels, and figure pixels per metre
FIG_JUNCTION = (639.0, 543.0)
PX_PER_M = 13.14

# Vehicle boxes drawn on Figure 2 (page points: x0, y0, x1, y1)
EGO = [(408, 125, 415, 135), (466, 119, 477, 126), (445, 125, 456, 132), (426, 143, 436, 150)]
OTHER = [(333, 143, 342, 149), (381, 119, 391, 126), (412, 160, 419, 171), (412, 82, 419, 93),
         (331, 119, 340, 125), (487, 120, 498, 126), (496, 145, 506, 151), (358, 143, 369, 150),
         (385, 143, 395, 150), (436, 138, 447, 144), (435, 119, 446, 126)]


def find_junction(carla_map, junction_id):
    for wp in carla_map.generate_waypoints(2.0):
        if wp.is_junction and wp.get_junction().id == junction_id:
            return wp.get_junction()
    raise RuntimeError("Junction {} not found: is Town03 loaded?".format(junction_id))


def box_to_world(box, centre):
    """Figure box (page points) -> world (x, y) and lane axis ('x' or 'y')."""
    u = ((box[0] + box[2]) / 2 - PAGE_BOX[0]) * FIG_SIZE[0] / (PAGE_BOX[2] - PAGE_BOX[0])
    v = ((box[1] + box[3]) / 2 - PAGE_BOX[1]) * FIG_SIZE[1] / (PAGE_BOX[3] - PAGE_BOX[1])
    x = centre.x - (v - FIG_JUNCTION[1]) / PX_PER_M
    y = centre.y + (u - FIG_JUNCTION[0]) / PX_PER_M
    axis = "y" if box[2] - box[0] > box[3] - box[1] else "x"
    return x, y, axis


def lane_waypoints(carla_map, junction):
    """Waypoints every 1 m on all driving lanes near the junction, including its inner lanes."""
    points = list(carla_map.generate_waypoints(1.0))
    for entry, _ in junction.get_waypoints(carla.LaneType.Driving):
        wp = entry
        for _ in range(60):
            points.append(wp)
            nxt = wp.next(1.0)
            if not nxt or not nxt[0].is_junction:
                break
            wp = nxt[0]
    return [w for w in points if w.lane_type == carla.LaneType.Driving]


def snap(points, x, y, axis):
    """Nearest lane waypoint whose heading runs along `axis`."""
    def aligned(wp):
        yaw = math.radians(wp.transform.rotation.yaw)
        return abs(math.cos(yaw) if axis == "x" else math.sin(yaw)) > 0.95

    near = [w for w in points if aligned(w)]
    return min(near, key=lambda w: math.hypot(w.transform.location.x - x, w.transform.location.y - y))


def main():
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(60.0)
    carla_map = client.get_world().get_map()
    junction = find_junction(carla_map, JUNCTION_ID)
    centre = junction.bounding_box.location
    points = lane_waypoints(carla_map, junction)

    vehicles = []
    for role, boxes in (("cav", EGO), ("traffic", OTHER)):
        for index, box in enumerate(boxes):
            x, y, axis = box_to_world(box, centre)
            wp = snap(points, x, y, axis)
            t = wp.transform
            vehicles.append({
                "role": role, "index": index,
                "pose": {"x": round(t.location.x, 2), "y": round(t.location.y, 2),
                         "z": round(t.location.z, 2), "roll": 0.0, "pitch": 0.0,
                         "yaw": round((t.rotation.yaw + 180.0) % 360.0 - 180.0, 1)},
                "road_id": wp.road_id, "lane_id": wp.lane_id,
                "snap_m": round(math.hypot(t.location.x - x, t.location.y - y), 2),
            })
            print("{}_{}: ({:.1f}, {:.1f}) yaw {:.0f}, moved {:.1f} m to the lane".format(
                role, index, t.location.x, t.location.y, t.rotation.yaw, vehicles[-1]["snap_m"]))

    plan = {
        "map": "Town03",
        "source": "Paper Figure 2, positions measured on the figure and snapped to lanes "
                  "(tools/make_figure2_plan.py)",
        "junction_id": JUNCTION_ID,
        "junction_center": [round(centre.x, 2), round(centre.y, 2), round(centre.z, 2)],
        "vehicles": vehicles,
    }
    os.makedirs(os.path.dirname(OUTPUT), exist_ok=True)
    with open(OUTPUT, "w", encoding="utf-8") as fh:
        json.dump(plan, fh, indent=2)
    print("Saved", OUTPUT)


if __name__ == "__main__":
    main()
