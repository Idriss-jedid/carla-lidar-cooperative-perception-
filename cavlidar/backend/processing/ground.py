"""Patchwise local-plane RANSAC ground removal for CARLA LiDAR, with a slope-aware height prior.

Each XY patch gets its own plane z = ax + by + c, so sloped roads are handled.
A plane is only accepted as ground if it agrees with what the LiDAR data says
the ground should look like there:

1. seed: the ground plane right around the sensor, fitted on the lowest
   returns 2.5-15 m away (gives the sensor height and tilt from the data);
2. continuity: patches are processed from the sensor outwards; a patch plane
   must meet the planes of its accepted neighbours along their shared edge
   (both ends, within `max_step`), so the ground may bend with a slope but
   can neither jump onto a car roof ~1.5 m higher nor tilt up a car's side;
3. lowest surface: a plane with returns clearly below it (road around a car
   under its roof) is not ground.

Patches with too few returns, or without an acceptable plane, are labelled
conservatively: only points within `residual_threshold` of the ground predicted
by their accepted neighbours become ground; everything else is kept.

This is NOT yet the paper's full method: the paper also filters to the
drivable-road region first. Everything works in the individual LiDAR sensor
frame (x, y, z) and uses nothing but the points: no map, no ground truth.
No CARLA runtime, Open3D or scikit-learn dependency is required.
"""
import math

import numpy as np

SEED_RING = (2.5, 15.0)    # metres from the sensor where the seed plane is fitted
SEED_LOW_PERCENTILE = 30   # the seed is fitted on the lowest returns of the ring
SEED_MAX_POINTS = 5000     # ring returns used for the seed (random subset, for speed)
SEED_TOL_GROWTH = 0.06     # extra metres of tolerance per metre from the sensor, seed reference only
BELOW_MARGIN = 0.45        # a return this far below a plane means the plane is not the ground
MAX_BELOW_FRACTION = 0.03  # ... unless such returns are this rare (fraction of the patch)


def _ransac(design, z, candidates, rng, iterations, max_grade):
    """(k, 3) plane coefficients a, b, c from random triples of `candidates` (all iterations at once)."""
    if len(candidates) < 3:
        return np.empty((0, 3))
    triples = candidates[rng.randint(0, len(candidates), size=(int(iterations), 3))]
    distinct = (triples[:, 0] != triples[:, 1]) & (triples[:, 0] != triples[:, 2]) & (triples[:, 1] != triples[:, 2])
    triples = triples[distinct]
    a = design[triples]
    ok = np.abs(np.linalg.det(a)) > 1e-9
    if not ok.any():
        return np.empty((0, 3))
    abc = np.linalg.solve(a[ok], z[triples[ok]][..., None])[..., 0]
    return abc[np.hypot(abc[:, 0], abc[:, 1]) <= max_grade]


def estimate_seed_plane(points, residual_threshold=0.18, iterations=200, max_slope_deg=15.0, seed=0):
    """
    Ground plane (a, b, c) around the sensor, from the lowest returns 2.5-15 m away;
    -c is the sensor height above the ground. None if there are too few returns.
    """
    xyz = np.asarray(points)[:, :3].astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    ring = xyz[(r >= SEED_RING[0]) & (r <= SEED_RING[1])]
    if len(ring) < 30:
        return None
    rng = np.random.RandomState(seed)
    if len(ring) > SEED_MAX_POINTS:
        ring = ring[rng.choice(len(ring), SEED_MAX_POINTS, replace=False)]
    design = np.column_stack((ring[:, 0], ring[:, 1], np.ones(len(ring))))
    low = np.flatnonzero(ring[:, 2] <= np.percentile(ring[:, 2], SEED_LOW_PERCENTILE))
    planes = _ransac(design, ring[:, 2], low, rng, iterations, math.tan(math.radians(max_slope_deg)))
    if not len(planes):
        return None
    counts = (np.abs(ring[:, 2][:, None] - design @ planes.T) <= residual_threshold).sum(axis=0)
    inliers = np.abs(ring[:, 2] - design @ planes[int(np.argmax(counts))]) <= residual_threshold
    abc = np.linalg.lstsq(design[inliers], ring[inliers, 2], rcond=None)[0]
    return abc


def patchwise_ground_mask(points, patch_size=8.0, residual_threshold=0.18, iterations=70, min_points=18,
                          max_slope_deg=15.0, seed=0, max_step=0.5, seed_plane=None):
    """Return a boolean array: True = probable ground, False = other (kept).

    `max_step`: largest height difference (m) between two neighbouring patch
    planes at their shared edge. `seed_plane`: (a, b, c) of the ground around
    the sensor; estimated from the points when None.
    """
    xyz = np.asarray(points)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("points must have shape (N, >=3)")
    if patch_size <= 0 or residual_threshold <= 0 or iterations <= 0 or min_points < 3 or max_step <= 0:
        raise ValueError("patch_size/threshold/iterations/max_step must be positive; min_points >= 3")
    if not (0 <= max_slope_deg < 90):
        raise ValueError("max_slope_deg must be in [0, 90)")
    mask = np.zeros(len(xyz), dtype=bool)
    if not len(xyz):
        return mask
    finite_idx = np.flatnonzero(np.isfinite(xyz[:, :3]).all(axis=1))
    if len(finite_idx) == 0:
        return mask
    pts = xyz[finite_idx, :3].astype(np.float64, copy=False)
    if seed_plane is None:
        seed_plane = estimate_seed_plane(pts, residual_threshold, max_slope_deg=max_slope_deg, seed=seed)
    keys = np.floor(pts[:, :2] / float(patch_size)).astype(np.int64)
    uniq, inverse = np.unique(keys, return_inverse=True, axis=0)
    inverse = inverse.ravel()
    order = np.argsort(inverse, kind="mergesort")
    cut = np.r_[0, np.flatnonzero(np.diff(inverse[order])) + 1, len(order)]
    members = {tuple(uniq[k]): order[cut[k]:cut[k + 1]] for k in range(len(uniq))}
    centre = {key: ((key[0] + 0.5) * patch_size, (key[1] + 0.5) * patch_size) for key in members}
    max_grade = math.tan(math.radians(max_slope_deg))
    rng = np.random.RandomState(seed)
    planes = {}  # accepted ground plane of each patch

    half = patch_size / 2.0
    corners = np.array([(-half, -half), (-half, half), (half, -half), (half, half)])

    def references(key, reach):
        """
        Where the patch plane is checked: (group, x, y, expected z, tolerance) rows,
        one group per accepted neighbour (points along the side facing it), else
        one group from the seed plane (the patch corners). None if nothing is known.
        """
        cx, cy = centre[key]
        rows = []
        group = 0
        for d in range(1, reach + 1):
            for dx in range(-d, d + 1):
                for dy in range(-d, d + 1):
                    other = (key[0] + dx, key[1] + dy)
                    if max(abs(dx), abs(dy)) != d or other not in planes:
                        continue
                    # this patch's corners on the side facing the neighbour
                    side = [(px, py) for px, py in corners
                            if (dx == 0 or np.sign(px) == np.sign(dx)) and (dy == 0 or np.sign(py) == np.sign(dy))]
                    tol = max_step + SEED_TOL_GROWTH * (d - 1) * patch_size  # extrapolated further
                    for px, py in side:
                        x, y = cx + px, cy + py
                        rows.append((group, x, y, planes[other] @ (x, y, 1.0), tol))
                    group += 1
            if rows:
                break
        if not rows and seed_plane is not None and math.hypot(cx, cy) <= SEED_RING[1] + patch_size:
            for px, py in corners:
                x, y = cx + px, cy + py
                rows.append((0, x, y, seed_plane @ (x, y, 1.0), max_step + SEED_TOL_GROWTH * math.hypot(x, y)))
        if not rows:
            return None
        return np.array(rows, dtype=np.float64)

    def agreeing(candidates, refs):
        """Candidates that every point of the majority of reference groups accepts."""
        error = np.abs(candidates @ np.column_stack((refs[:, 1], refs[:, 2], np.ones(len(refs)))).T
                       - refs[:, 3]) - refs[:, 4]
        groups = np.unique(refs[:, 0])
        worst = np.stack([error[:, refs[:, 0] == g].max(axis=1) for g in groups], axis=1)
        return np.median(worst, axis=1) <= 0.0

    rejected = []
    for key in sorted(members, key=lambda k: math.hypot(*centre[k])):
        ix = members[key]
        patch = pts[ix]
        n = len(patch)
        refs = references(key, reach=2)
        if n < min_points or refs is None:
            rejected.append(key)  # too sparse, or nothing tells where the ground is: decide later
            continue
        design = np.column_stack((patch[:, 0], patch[:, 1], np.ones(n)))
        # Sampling the low points helps avoid fitting elevated obstacle tops.
        low = np.flatnonzero(patch[:, 2] <= np.percentile(patch[:, 2], 45.0))
        candidates = _ransac(design, patch[:, 2], low, rng, iterations, max_grade)
        best = None
        if len(candidates):
            residual = patch[:, 2][:, None] - design @ candidates.T
            counts = (np.abs(residual) <= residual_threshold).sum(axis=0)
            below = (residual < -BELOW_MARGIN).sum(axis=0)
            ok = below <= max(2, MAX_BELOW_FRACTION * n)
            ok &= agreeing(candidates, refs)
            ok &= counts >= max(3, int(math.ceil(0.15 * n)))  # enough evidence of road in this patch
            if ok.any():
                best = candidates[np.flatnonzero(ok)[int(np.argmax(counts[ok]))]]
        if best is None:
            rejected.append(key)
            continue
        inliers = np.abs(patch[:, 2] - design @ best) <= residual_threshold
        abc, _, rank, _ = np.linalg.lstsq(design[inliers], patch[inliers, 2], rcond=None)
        if rank == 3 and np.hypot(abc[0], abc[1]) <= max_grade and agreeing(abc[None, :], refs)[0]:
            best = abc
        planes[key] = best
        final = np.abs(patch[:, 2] - design @ best) <= residual_threshold
        mask[finite_idx[ix[final]]] = True

    # Sparse / rejected patches: only points on the ground predicted by accepted neighbours
    for key in rejected:
        near = [planes[(key[0] + dx, key[1] + dy)] for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                if (dx or dy) and (key[0] + dx, key[1] + dy) in planes]
        if not near:
            continue  # no evidence: keep every point
        ix = members[key]
        patch = pts[ix]
        design = np.column_stack((patch[:, 0], patch[:, 1], np.ones(len(patch))))
        predicted = (design @ np.array(near).T).mean(axis=1)
        final = np.abs(patch[:, 2] - predicted) <= residual_threshold
        mask[finite_idx[ix[final]]] = True
    return mask
