"""
Bird's-eye-view plots of recorded frames (numpy + matplotlib, no CARLA needed).

CARLA's y axis points to the right of x, so the y axis of every plot is
inverted: the image then looks like the scene seen from above.
"""
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from ...core.enums import Role  # noqa: E402
from ..geometry.transforms import box_corners_2d, sensor_to_world  # noqa: E402

_COLORS = plt.get_cmap("tab10").colors


def plot_sensor_frame(meta, clouds, out_path, view_range=60.0):
    """One panel per CAV, each in its own sensor frame (car at the origin)."""
    names = sorted(clouds)
    cols = min(len(names), 3)
    rows = int(math.ceil(len(names) / float(cols)))
    fig, axes = plt.subplots(rows, cols, figsize=(5.5 * cols, 5.5 * rows), squeeze=False)
    for ax in axes.ravel():
        ax.axis("off")
    for ax, name in zip(axes.ravel(), names):
        pts = clouds[name]
        ax.axis("on")
        ax.scatter(pts[:, 0], pts[:, 1], s=0.2, c=pts[:, 2], cmap="viridis", vmin=-2.5, vmax=2.0)
        ax.plot(0, 0, "r^", ms=9)
        ax.set_xlim(-view_range, view_range)
        ax.set_ylim(view_range, -view_range)
        ax.set_aspect("equal")
        ax.set_title("{}  ({} points)".format(name, len(pts)))
        ax.set_xlabel("x forward (m)")
        ax.set_ylabel("y right (m)")
    fig.suptitle("Frame {} - each CAV in its own LiDAR frame".format(meta["frame"]))
    fig.tight_layout()
    fig.savefig(out_path, dpi=80)
    plt.close(fig)
    return out_path


def plot_world_frame(meta, clouds, out_path, margin=30.0, max_height=3.0):
    """
    All CAV clouds transformed to the world frame on one plot (point-level
    fusion), with ground-truth boxes: black = CAV, grey = other vehicles.
    Points higher than `max_height` above each sensor's ground are dropped to
    keep building tops from hiding the road.
    """
    fig, ax = plt.subplots(figsize=(11, 11))
    xs, ys = [], []
    for i, name in enumerate(sorted(clouds)):
        pose = meta["cavs"][name]["sensor_pose"]
        pts = clouds[name]
        pts = pts[pts[:, 2] < max_height - meta["cavs"][name]["mount_z"]]
        world = sensor_to_world(pts, pose)
        color = _COLORS[i % len(_COLORS)]
        ax.scatter(world[:, 0], world[:, 1], s=0.15, color=color, alpha=0.5)
        ax.plot(pose["x"], pose["y"], "*", color=color, ms=16, mec="k", label=name)
        xs.append(pose["x"])
        ys.append(pose["y"])

    for v in meta.get("vehicles", []):
        corners = box_corners_2d(v["pose"], v["extent"], v["center_offset"])
        is_cav = v["role"] == Role.CAV.value
        ax.fill(corners[:, 0], corners[:, 1], fill=False,
                ec="k" if is_cav else "0.45", lw=1.6 if is_cav else 1.0)

    if xs:
        ax.set_xlim(min(xs) - margin, max(xs) + margin)
        ax.set_ylim(max(ys) + margin, min(ys) - margin)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", markerscale=0.8)
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.set_title("Frame {} - all CAVs fused in the world frame, boxes = ground truth"
                 .format(meta["frame"]))
    fig.tight_layout()
    fig.savefig(out_path, dpi=80)
    plt.close(fig)
    return out_path
