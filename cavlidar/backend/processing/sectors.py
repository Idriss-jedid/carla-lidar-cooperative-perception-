"""
Static (fixed) angular sectorization and sector metadata (paper, Sec. III-A-3 and III-A-4).

Static sectorization: "the perception space around the ego-CAV is divided into a
predefined number of sectors with fixed angular boundaries. The starting angle and
the number of sectors are specified in advance (e.g., 4 sectors starting at pi/4),
and the BEV grid is partitioned accordingly."

Angles are azimuths in the LiDAR sensor frame, measured from the forward axis +x
towards +y (CARLA's right, i.e. clockwise seen from above), in radians. Sector k
covers [start + k * width, start + (k + 1) * width), width = 2 pi / count.

Metadata of sector s of CAV i, the only thing sent to the central node:
- size sigma_{i,s}: data volume of the sector's raw points (what is transmitted if
  the sector is selected); the ILP compares it with the budget B (MB);
- quality S~_{i,s}: the sector's non-ground points are downsampled by averaging
  points that are too close to each other (here: points in the same `downsample` m
  voxel), and S~ = number of downsampled points / number of all points of the sector;
- footprint A_{i,s}: start and end angles and sensing range, also as a world-frame
  polygon (from the CAV pose) for the sector-level IoU of the optimization step.

Not given by the paper and chosen here: the downsampling radius (0.2 m voxels), the
range of the footprint (the LiDAR's nominal range, 120 m in the paper's Table 1; the
farthest return of the sector is stored too) and the angle convention above.
"""
import math

import numpy as np

from ..geometry.transforms import pose_to_matrix

BYTES_PER_POINT = 16   # x, y, z, intensity as float32, as recorded
FOOTPRINT_ARC_STEP = math.radians(2.0)


def static_sectors(count=4, start=math.pi / 4):
    """[(start, end)] of `count` equal sectors, the first one beginning at `start` (radians)."""
    if count < 1:
        raise ValueError("count must be at least 1")
    width = 2 * math.pi / count
    return [(start + k * width, start + (k + 1) * width) for k in range(count)]


def sector_index(x, y, sectors):
    """Index of the sector each (x, y) (sensor frame) falls in."""
    start, width = sectors[0][0], sectors[0][1] - sectors[0][0]
    azimuth = np.arctan2(np.asarray(y, dtype=np.float64), np.asarray(x, dtype=np.float64))
    k = np.floor(np.mod(azimuth - start, 2 * math.pi) / width).astype(np.int64)
    return np.minimum(k, len(sectors) - 1)  # azimuth - start == 2 pi after rounding


def voxel_downsample(points, size):
    """Average the points that fall in the same `size` m voxel (all columns are averaged)."""
    pts = np.asarray(points, dtype=np.float64)
    if not len(pts):
        return pts.reshape(0, pts.shape[1] if pts.ndim == 2 else 4)
    keys = np.floor(pts[:, :3] / float(size)).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.ravel()
    sums = np.zeros((len(counts), pts.shape[1]))
    np.add.at(sums, inverse, pts)
    return sums / counts[:, None]


def footprint_polygon(sensor_pose, start, end, radius, arc_step=FOOTPRINT_ARC_STEP):
    """World-frame (x, y) vertices of the sector wedge: apex at the sensor, arc at `radius`."""
    n = max(2, int(math.ceil((end - start) / arc_step)) + 1)
    angles = np.linspace(start, end, n)
    local = np.vstack([[0.0, 0.0], np.column_stack((radius * np.cos(angles), radius * np.sin(angles)))])
    m = pose_to_matrix(sensor_pose)
    # the wedge lies in the sensor's horizontal plane; only its top view is used
    world = local @ m[:2, :2].T + m[:2, 3]
    return world


def bev_sector_grid(shape, xy_bounds, sectors):
    """Sector index of every BEV cell (cell centres; same layout as bev_grid.bev_from_points)."""
    nx, ny = shape
    x0, x1, y0, y1 = xy_bounds
    rx, ry = (x1 - x0) / nx, (y1 - y0) / ny
    xs = x1 - (np.arange(nx) + 0.5) * rx      # row 0 = largest x (forward, top)
    ys = y0 + (np.arange(ny) + 0.5) * ry      # columns increase with +y (right)
    return sector_index(xs[:, None] + 0 * ys[None, :], ys[None, :] + 0 * xs[:, None], sectors).astype(np.int8)


def sector_metadata(raw, non_ground, sectors, sensor_pose, lidar_range, downsample=0.2,
                    bytes_per_point=BYTES_PER_POINT):
    """
    Metadata of every sector of one CAV.
    raw: all points of the scan (what a selected sector sends); non_ground: the
    obstacle points left after road filtering and ground removal (both sensor frame).
    """
    raw = np.asarray(raw)
    non_ground = np.asarray(non_ground)
    raw_k = sector_index(raw[:, 0], raw[:, 1], sectors)
    ng_k = sector_index(non_ground[:, 0], non_ground[:, 1], sectors) if len(non_ground) else np.zeros(0, int)
    yaw = math.radians(sensor_pose["yaw"])
    out = []
    for k, (start, end) in enumerate(sectors):
        in_raw = raw[raw_k == k]
        obstacles = non_ground[ng_k == k]
        reduced = voxel_downsample(obstacles, downsample) if len(obstacles) else obstacles
        n = len(in_raw)
        out.append({
            "index": k,
            "start_rad": start, "end_rad": end,
            "range_m": float(lidar_range),
            "farthest_return_m": float(np.hypot(in_raw[:, 0], in_raw[:, 1]).max()) if n else 0.0,
            "points": int(n),
            "size_bytes": int(n * bytes_per_point),                    # sigma_{i,s}
            "size_mb": n * bytes_per_point / 1e6,
            "non_ground_points": int(len(obstacles)),
            "downsampled_points": int(len(reduced)),
            "quality": float(len(reduced) / n) if n else 0.0,          # S~_{i,s}
            # footprint A_{i,s} in the world frame: CARLA yaw adds to the sensor azimuths
            "world_start_rad": start + yaw, "world_end_rad": end + yaw,
            "apex": [float(sensor_pose["x"]), float(sensor_pose["y"])],
            "footprint": footprint_polygon(sensor_pose, start, end, lidar_range).round(3).tolist(),
        })
    return out
