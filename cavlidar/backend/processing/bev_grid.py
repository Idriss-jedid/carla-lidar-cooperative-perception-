"""Numerical BEV grid: occupancy, return counts, mean LiDAR intensity.

Input is one CAV's non-ground 3D LiDAR points in its SENSOR frame.
Grid rows go from x forward (top) to x backward (bottom); columns
increase with CARLA +y (vehicle right). Spatial bounds are half-open.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class BevGrid:
    occupancy: np.ndarray       # (x_bins, y_bins), uint8
    mean_intensity: np.ndarray  # (x_bins, y_bins), float32
    counts: np.ndarray          # (x_bins, y_bins), int32
    xy_bounds: tuple
    resolution: float


def bev_from_points(points, resolution=0.5, xy_bounds=(-60.0, 60.0, -60.0, 60.0)):
    """Aggregate N x >=4 [x,y,z,intensity] non-ground points into a BEV.

    The occupancy grid tells where at least one return exists. Other cells
    are 'no return', NOT known free space; no ray-casting is performed.
    """
    pts = np.asarray(points)
    if pts.ndim != 2 or pts.shape[1] < 4:
        raise ValueError("points must have shape (N, >=4): x,y,z,intensity")
    if len(xy_bounds) != 4:
        raise ValueError("xy_bounds must be (x_min,x_max,y_min,y_max)")
    x0, x1, y0, y1 = map(float, xy_bounds)
    if not (np.isfinite([x0, x1, y0, y1, resolution]).all() and
            x1 > x0 and y1 > y0 and resolution > 0):
        raise ValueError("invalid BEV bounds or resolution")
    nx_float = (x1 - x0) / resolution
    ny_float = (y1 - y0) / resolution
    nx, ny = int(round(nx_float)), int(round(ny_float))
    if not (abs(nx_float - nx) < 1e-6 and abs(ny_float - ny) < 1e-6):
        raise ValueError("bounds must be divisible by resolution")
    occupancy = np.zeros((nx, ny), dtype=np.uint8)
    mean_intensity = np.zeros((nx, ny), dtype=np.float32)
    counts = np.zeros((nx, ny), dtype=np.int32)
    if len(pts):
        valid = (np.isfinite(pts[:, :4]).all(axis=1) &
                 (pts[:, 0] >= x0) & (pts[:, 0] < x1) &
                 (pts[:, 1] >= y0) & (pts[:, 1] < y1))
        pts = pts[valid]
        if len(pts):
            col = np.floor((pts[:, 1] - y0) / resolution).astype(np.int64)
            row = nx - 1 - np.floor((pts[:, 0] - x0) / resolution).astype(np.int64)
            flat = row * ny + col
            count_flat = np.bincount(flat, minlength=nx * ny)
            sums = np.bincount(flat, weights=pts[:, 3], minlength=nx * ny)
            populated = count_flat > 0
            values = np.zeros(nx * ny, dtype=np.float32)
            values[populated] = (sums[populated] / count_flat[populated]).astype(np.float32)
            counts = count_flat.reshape(nx, ny).astype(np.int32)
            occupancy = populated.reshape(nx, ny).astype(np.uint8)
            mean_intensity = values.reshape(nx, ny)
    return BevGrid(occupancy, mean_intensity, counts,
                   (x0, x1, y0, y1), float(resolution))
