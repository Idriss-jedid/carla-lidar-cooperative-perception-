"""
Drivable-road filter: keep only the LiDAR returns above the drivable road region.

Paper, "Point Cloud Cleaning": before ground removal, "an additional filtering
step is introduced to isolate the drivable road region ... This pre-filtering
step removes points outside the road surface, such as off-road structures and
irrelevant background elements". The paper does not say how the region is
obtained; here it comes from the map's OpenDRIVE road network (an HD map, as
in OpenCDA / OpenCOOD), placed with the CAV's pose:

- only the 2D outline of the lanes is used (Driving, Parking, Bidirectional
  lanes, grown by `margin` for the curb): no road height, no vehicle boxes;
- a point is kept if its (x, y) in the world frame falls inside the outline,
  whatever its height, so vehicles, also tall ones, stay whole.
"""
import math
import os

import carla
import numpy as np

from ..geometry.transforms import sensor_to_world

DRIVABLE_LANES = (carla.LaneType.Driving, carla.LaneType.Parking, carla.LaneType.Bidirectional)
RESOLUTION = 0.25   # metres per raster cell
MARGIN = 0.5        # metres added around the lanes (curb, wheels over the lane line)
LANE_STEP = 1.0     # metres between lane samples


class DrivableArea:
    """Top-view raster of the drivable lanes of one map, world frame."""

    def __init__(self, mask, x0, y0, resolution):
        self.mask, self.x0, self.y0, self.resolution = mask, float(x0), float(y0), float(resolution)

    @classmethod
    def from_carla_map(cls, carla_map, resolution=RESOLUTION, margin=MARGIN):
        lanes = list(carla_map.generate_waypoints(LANE_STEP))
        for wp in list(lanes):  # parking / bidirectional lanes next to the driving lanes
            for side in (wp.get_left_lane(), wp.get_right_lane()):
                if side is not None and side.lane_type in DRIVABLE_LANES[1:]:
                    lanes.append(side)
        centre = np.array([[w.transform.location.x, w.transform.location.y] for w in lanes])
        yaw = np.radians([w.transform.rotation.yaw for w in lanes])
        width = np.array([w.lane_width for w in lanes])
        # fill each lane sample's rectangle (LANE_STEP long, lane wide) with sample points
        step = resolution * 0.8
        along = np.arange(-LANE_STEP / 2 - step, LANE_STEP / 2 + step + 1e-9, step)
        across = np.linspace(-0.5, 0.5, int(math.ceil(width.max() / step)) + 1)
        a, b = np.meshgrid(along, across, indexing="ij")
        a, b = a.ravel()[None, :], b.ravel()[None, :] * width[:, None]
        fx, fy = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        x = (centre[:, 0:1] + a * fx - b * fy).ravel()
        y = (centre[:, 1:2] + a * fy + b * fx).ravel()
        pad = margin + 2 * resolution
        x0, y0 = x.min() - pad, y.min() - pad
        nx = int(math.ceil((x.max() + pad - x0) / resolution))
        ny = int(math.ceil((y.max() + pad - y0) / resolution))
        mask = np.zeros((nx, ny), dtype=bool)
        mask[((x - x0) / resolution).astype(int), ((y - y0) / resolution).astype(int)] = True
        for _ in range(int(round(margin / resolution))):  # grow by the margin
            grown = mask.copy()
            grown[1:] |= mask[:-1]
            grown[:-1] |= mask[1:]
            grown[:, 1:] |= mask[:, :-1]
            grown[:, :-1] |= mask[:, 1:]
            mask = grown
        return cls(mask, x0, y0, resolution)

    def contains(self, x, y):
        """True where (x, y) (world frame, arrays) is on the drivable road."""
        i = np.floor((np.asarray(x) - self.x0) / self.resolution).astype(np.int64)
        j = np.floor((np.asarray(y) - self.y0) / self.resolution).astype(np.int64)
        inside = (i >= 0) & (i < self.mask.shape[0]) & (j >= 0) & (j < self.mask.shape[1])
        out = np.zeros(i.shape, dtype=bool)
        out[inside] = self.mask[i[inside], j[inside]]
        return out

    def save(self, path):
        np.savez_compressed(path, mask=self.mask, origin=[self.x0, self.y0], resolution=self.resolution)

    @classmethod
    def load(cls, path):
        data = np.load(path)
        return cls(data["mask"], data["origin"][0], data["origin"][1], float(data["resolution"]))


def load_drivable_area(name, xodr_path, cache_dir=None, resolution=RESOLUTION, margin=MARGIN):
    """DrivableArea of a map from its OpenDRIVE file, cached as .npz in cache_dir."""
    cache = None
    if cache_dir:
        cache = os.path.join(cache_dir, "{}_drivable_{:g}m_margin{:g}m.npz".format(name, resolution, margin))
        if os.path.isfile(cache) and os.path.getmtime(cache) >= os.path.getmtime(xodr_path):
            return DrivableArea.load(cache)
    with open(xodr_path, "r", encoding="utf-8") as fh:
        area = DrivableArea.from_carla_map(carla.Map(name, fh.read()), resolution, margin)
    if cache:
        os.makedirs(cache_dir, exist_ok=True)
        area.save(cache)
    return area


def road_mask(points, sensor_pose, area):
    """True for the points (sensor frame) that lie above the drivable road."""
    world = sensor_to_world(np.asarray(points)[:, :3], sensor_pose)
    return area.contains(world[:, 0], world[:, 1])
