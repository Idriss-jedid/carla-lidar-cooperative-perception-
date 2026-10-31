"""Prototype patchwise local-plane RANSAC for CARLA LiDAR.

Adapted to sloped terrain via a different fitted z = ax + by + c plane
for each XY patch. This is NOT the fully validated Patchwork RANSAC method
of the paper: the paper also filters to drivable-road areas first.

Everything here operates in the *individual LiDAR sensor frame* (x, y, z).
No CARLA runtime, Open3D, or scikit-learn dependency is required.
"""
import math

import numpy as np


def patchwise_ground_mask(points, patch_size=8.0, residual_threshold=0.18,
                          iterations=70, min_points=18, max_slope_deg=25.0, seed=0):
    """Return a boolean array: True = probable ground, False = other.

    Patches with too few returns or no plausible plane remain *unclassified*
    (False), avoiding accidental removal of small obstacles. Filtering roads
    before this call is recommended for a paper-faithful implementation.
    """
    xyz = np.asarray(points)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("points must have shape (N, >=3)")
    if patch_size <= 0 or residual_threshold <= 0 or iterations <= 0 or min_points < 3:
        raise ValueError("patch_size/threshold/iterations must be positive; min_points >= 3")
    if not (0 <= max_slope_deg < 90):
        raise ValueError("max_slope_deg must be in [0, 90)")
    mask = np.zeros(len(xyz), dtype=bool)
    if not len(xyz):
        return mask
    finite_idx = np.flatnonzero(np.isfinite(xyz[:, :3]).all(axis=1))
    if len(finite_idx) == 0:
        return mask
    pts = xyz[finite_idx, :3].astype(np.float64, copy=False)
    keys = np.floor(pts[:, :2] / float(patch_size)).astype(np.int64)
    _, inverse = np.unique(keys, return_inverse=True, axis=0)
    order = np.argsort(inverse, kind="mergesort")
    cut = np.r_[0, np.flatnonzero(np.diff(inverse[order])) + 1, len(order)]
    max_grade = math.tan(math.radians(max_slope_deg))
    rng = np.random.RandomState(seed)

    for k in range(len(cut) - 1):
        ix = order[cut[k]:cut[k + 1]]
        patch = pts[ix]
        n = len(patch)
        if n < min_points:
            continue
        # Sampling the low points helps avoid fitting elevated obstacle tops.
        cutoff = np.percentile(patch[:, 2], 45.0)
        low = np.flatnonzero(patch[:, 2] <= cutoff)
        if len(low) < 3:
            continue
        design = np.column_stack((patch[:, 0], patch[:, 1], np.ones(n)))
        best_inliers = None
        best_n = 0
        best_z = np.inf
        for _ in range(int(iterations)):
            sample = rng.choice(low, 3, replace=False)
            try:
                abc = np.linalg.solve(design[sample], patch[sample, 2])
            except np.linalg.LinAlgError:
                continue
            if np.hypot(abc[0], abc[1]) > max_grade:
                continue
            inliers = np.abs(design @ abc - patch[:, 2]) <= residual_threshold
            count = int(inliers.sum())
            level = float(np.median(patch[inliers, 2])) if count else np.inf
            if count > best_n or (count == best_n and level < best_z):
                best_n, best_inliers, best_z = count, inliers, level
        # Do not erase points if this patch has little evidence of road.
        if best_inliers is None or best_n < max(3, int(math.ceil(0.15 * n))):
            continue
        abc, _, rank, _ = np.linalg.lstsq(design[best_inliers],
                                          patch[best_inliers, 2], rcond=None)
        if rank != 3 or np.hypot(abc[0], abc[1]) > max_grade:
            continue
        final = np.abs(design @ abc - patch[:, 2]) <= residual_threshold
        mask[finite_idx[ix[final]]] = True
    return mask
