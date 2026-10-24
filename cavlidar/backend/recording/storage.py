"""
Reading and writing runs on disk (numpy only, usable without CARLA).

Layout of one run:

    data/runs/<date>_<time>_<name>/
        config.yaml                 exact parameters used
        frames/<frame>/cav0.npy     point cloud of CAV 0 (sensor frame)
        frames/<frame>/cav0.ply     same, if 'ply' is in recording.formats
        frames/<frame>/meta.json    sensor poses + ground-truth vehicles
        preview_*.png               quick-look images
"""
import json
import os
import time

import numpy as np

from ...core.constants import FRAME_DIR_FORMAT, FRAMES_DIR, META_FILE
from ...core.enums import PointFormat

_PLY_TYPES = {"float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8"}


def create_run_dir(base, name):
    """Create data/runs/<date>_<time>_<name>/frames and return the run folder."""
    path = os.path.join(base, "{}_{}".format(time.strftime("%Y-%m-%d_%H%M%S"), name))
    os.makedirs(os.path.join(path, FRAMES_DIR))
    return path


def frame_dir(run_dir, frame):
    path = os.path.join(run_dir, FRAMES_DIR, FRAME_DIR_FORMAT.format(frame))
    os.makedirs(path, exist_ok=True)
    return path


def write_ply(path, points):
    """Binary little-endian PLY with x, y, z, intensity (opens in MeshLab/Open3D)."""
    pts = np.ascontiguousarray(points[:, :4], dtype="<f4")
    header = ("ply\nformat binary_little_endian 1.0\nelement vertex {}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property float intensity\nend_header\n").format(len(pts))
    with open(path, "wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(pts.tobytes())


def read_ply(path):
    """Read an ASCII (CARLA save_to_disk) or binary PLY into an (N, k) float32 array."""
    with open(path, "rb") as fh:
        fmt, count, props = None, 0, []
        while True:
            line = fh.readline().decode("ascii").strip()
            if line.startswith("format"):
                fmt = line.split()[1]
            elif line.startswith("element vertex"):
                count = int(line.split()[-1])
            elif line.startswith("property"):
                props.append(_PLY_TYPES[line.split()[1]])
            elif line == "end_header":
                break
        if fmt == "ascii":
            data = np.loadtxt(fh, dtype=np.float32, max_rows=count, ndmin=2)
        else:
            dtype = np.dtype([("p{}".format(i), t) for i, t in enumerate(props)])
            raw = np.frombuffer(fh.read(dtype.itemsize * count), dtype=dtype)
            data = np.stack([raw[n].astype(np.float32) for n in dtype.names], axis=1)
    return data


def save_points(directory, name, points, formats):
    """Save one point cloud as <name>.npy and/or <name>.ply."""
    for fmt in formats:
        fmt = PointFormat.parse(fmt)
        path = os.path.join(directory, "{}.{}".format(name, fmt.value))
        if fmt == PointFormat.NPY:
            np.save(path, points)
        else:
            write_ply(path, points)


def load_points(path):
    return np.load(path) if path.endswith(".npy") else read_ply(path)


def save_json(path, data):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=1)


def load_json(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def list_frames(run_dir):
    """Sorted frame numbers stored in a run."""
    return sorted(int(d) for d in os.listdir(os.path.join(run_dir, FRAMES_DIR)) if d.isdigit())


def load_frame(run_dir, frame):
    """Return (meta, {cav_name: points}) for one stored frame."""
    directory = os.path.join(run_dir, FRAMES_DIR, FRAME_DIR_FORMAT.format(frame))
    meta = load_json(os.path.join(directory, META_FILE))
    clouds = {}
    for name in meta["cavs"]:
        for fmt in PointFormat:
            path = os.path.join(directory, "{}.{}".format(name, fmt.value))
            if os.path.exists(path):
                clouds[name] = load_points(path)
                break
    return meta, clouds
