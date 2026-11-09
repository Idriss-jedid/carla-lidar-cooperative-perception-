"""
Alignment between a vehicle's LiDAR returns and its ground-truth box, measured along the vehicle's heading.

Only the END of the box that faces the sensor is measured by the returns:

    rear end visible  (sensor behind the box):  offset = min(a) + e_x
    front end visible (sensor ahead of the box): offset = max(a) - e_x
    side view         (sensor beside the box):   both ends, averaged

a = coordinate along the heading in the box frame (origin at the box centre), e_x = half length.
offset < 0: the returns lie behind the box, i.e. the box is ahead of the vehicle the LiDAR saw. A box
k ticks late gives offset = -k * v * dt in EVERY view. The sampling gap (the extreme return lies a
little inside the true end) has opposite signs at the rear (offset >= 0) and at the front (<= 0), so it
cannot be mistaken for a timing error, which has the same sign at both ends.

Why not "the shift that holds the most returns" (the first version of the check): seen from behind a
vehicle shows only its rear face, and every backward shift of the box keeps those returns inside. The
best shift is then undetermined on that side, and the median of the tied shifts is half the search
window (about -1 m for a +-2 m search) whatever the true alignment; seen from ahead it is about +1 m.
`tie_shift` keeps that estimator so its bias can be shown.

Pure numpy; vehicles are ground-truth dicts as in the recorder's meta.json (pose, extent, center_offset).
"""
import numpy as np

from cavlidar.backend.geometry.transforms import pose_to_matrix

VIEWS = ("rear", "side", "front")
TIE_SHIFTS = np.round(np.arange(-2.0, 2.0001, 0.05), 2)   # search of the first version of the check


def box_frame(points, vehicle):
    """(N, 3) world points -> (a, b, h): along the heading and to the side of the box centre, height above its bottom."""
    e = vehicle["extent"]
    off = vehicle.get("center_offset", [0.0, 0.0, e[2]])
    xyz1 = np.hstack([np.asarray(points, dtype=np.float64)[:, :3], np.ones((len(points), 1))])
    local = xyz1 @ np.linalg.inv(pose_to_matrix(vehicle["pose"])).T
    return local[:, 0] - off[0], local[:, 1] - off[1], local[:, 2] - (off[2] - e[2])


def view_of(sensor_xyz, vehicle):
    """'rear', 'front' or 'side': where the sensor is relative to the box, along the vehicle's heading."""
    a = box_frame(np.asarray(sensor_xyz, dtype=np.float64).reshape(1, 3), vehicle)[0][0]
    e_x = vehicle["extent"][0]
    return "rear" if a < -e_x else ("front" if a > e_x else "side")


def face_offset(a, e_x, view, q=0.0):
    """
    Offset (m) between the returns and the box along the heading, from the visible end(s).
    q: quantile used instead of the extreme return (0 = extreme; 0.02 ignores a few stray returns).
    """
    a = np.asarray(a, dtype=np.float64)
    if len(a) == 0:
        return float("nan")
    rear = float(np.quantile(a, q)) + e_x
    front = float(np.quantile(a, 1.0 - q)) - e_x
    if view == "rear":
        return rear
    if view == "front":
        return front
    return 0.5 * (rear + front)


def near_box(points, vehicle, others=(), along=1.5, across=0.3, z_min=0.3):
    """
    Returns attributed to `vehicle` without labels: within `along` m of its box ends and `across` m of its
    sides, more than z_min above its bottom (no ground), and outside every other vehicle's box (0.3 m margin).
    `along` bounds the largest offset that can be measured.
    """
    e = vehicle["extent"]
    a, b, h = box_frame(points, vehicle)
    mask = (np.abs(a) <= e[0] + along) & (np.abs(b) <= e[1] + across) & (h > z_min) & (h < 2 * e[2] + 0.3)
    for o in others:
        if o["id"] == vehicle["id"]:
            continue
        ea = o["extent"]
        oa, ob, oh = box_frame(points, o)
        mask &= ~((np.abs(oa) <= ea[0] + 0.3) & (np.abs(ob) <= ea[1] + 0.3) & (oh > -0.5) & (oh < 2 * ea[2] + 0.3))
    return mask


def tie_shift(points, vehicle, shifts=TIE_SHIFTS, margin=0.1, z_min=0.3):
    """
    The first version of the check: slide the box along its heading and keep the median of the shifts
    holding the most returns. Returns (estimate, lowest tied shift, highest tied shift); a positive shift
    moves the box forward. Biased for end views (see the module docstring); kept to show that bias.
    """
    e = vehicle["extent"]
    a, b, h = box_frame(points, vehicle)
    keep = (np.abs(b) <= e[1] + margin) & (h > z_min) & (h < 2 * e[2] + 0.3)
    a = a[keep]
    counts = np.array([(np.abs(a - s) <= e[0] + margin).sum() for s in shifts])
    tied = np.asarray(shifts)[counts == counts.max()]
    return float(np.median(tied)), float(tied.min()), float(tied.max())


def along_speed(vehicle_velocity, yaw_deg):
    """Velocity component along a heading (m/s); vehicle_velocity is (vx, vy[, vz]) in the world."""
    h = np.radians(yaw_deg)
    return float(vehicle_velocity[0] * np.cos(h) + vehicle_velocity[1] * np.sin(h))


def pose_difference(p, q):
    """(position difference m, largest angle difference deg) between two pose dicts."""
    dpos = float(np.sqrt(sum((p[k] - q[k]) ** 2 for k in ("x", "y", "z"))))
    dang = max(abs((p[k] - q[k] + 180.0) % 360.0 - 180.0) for k in ("roll", "pitch", "yaw"))
    return dpos, float(dang)


def compose(parent, mount_xyz):
    """World pose of a sensor mounted at mount_xyz (no rotation) on a parent pose (CARLA attachment)."""
    m = pose_to_matrix(parent)
    loc = m @ np.array([mount_xyz[0], mount_xyz[1], mount_xyz[2], 1.0])
    return {"x": float(loc[0]), "y": float(loc[1]), "z": float(loc[2]),
            "roll": parent["roll"], "pitch": parent["pitch"], "yaw": parent["yaw"]}


def subset_share(small, large, tol=0.0):
    """
    Share of the rows of `small` (N, 3) found among the rows of `large`: bit for bit (tol = 0) or within
    tol metres on every axis (points snapped to a tol grid, neighbouring cells included).
    """
    if len(small) == 0:
        return 1.0
    s = np.asarray(small, dtype=np.float32)[:, :3]
    big = np.asarray(large, dtype=np.float32)[:, :3]
    if tol == 0.0:
        row = np.dtype((np.void, 12))
        return float(np.isin(np.ascontiguousarray(s).view(row).ravel(),
                             np.ascontiguousarray(big).view(row).ravel()).mean())
    qs = np.floor(s / tol).astype(np.int64)
    qb = np.floor(big / tol).astype(np.int64)
    key = lambda q: (q[:, 0] * 1000003 + q[:, 1]) * 1000033 + q[:, 2]   # noqa: E731
    kb = np.unique(key(qb))
    found = np.zeros(len(s), dtype=bool)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                found |= np.isin(key(qs + np.array([dx, dy, dz])), kb)
    return float(found.mean())


def frame_checks(ticks):
    """
    Frame and time consistency of a recorded diagnostic (list of tick records): every measurement and the
    WorldSnapshot carry the frame world.tick() returned, the same simulation time, a delta of the fixed step,
    and the client received exactly one snapshot per tick. Returns {check: number of violations, ...}.
    """
    out = {"ticks": len(ticks), "snapshot_frame": 0, "sensor_frame": 0, "sensor_time": 0, "delta": 0,
           "snapshots_received": 0, "consecutive": 0}
    prev = None
    for t in ticks:
        snap = t["snapshot"]
        out["snapshot_frame"] += snap["frame"] != t["frame"]
        out["delta"] += abs(snap["delta"] - t["dt"]) > 1e-6
        for s in t["sensors"].values():
            for m in (s["ray"], s["sem"]):
                out["sensor_frame"] += m["frame"] != t["frame"]
                out["sensor_time"] += abs(m["timestamp"] - snap["elapsed"]) > 1e-6
        received = [f for f, _ in t.get("received", [])]
        out["snapshots_received"] += received != [t["frame"]]
        if prev is not None:
            out["consecutive"] += t["frame"] != prev + 1
        prev = t["frame"]
    return out
