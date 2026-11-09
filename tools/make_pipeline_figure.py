"""
One recorded LiDAR sweep through the per-CAV processing chain, in four panels:
raw sweep -> ego mask + drivable-road filter -> ground removal -> BEV occupancy with the static sectors.

    python tools/make_pipeline_figure.py data/runs/<run> --cav cav2
    python tools/make_pipeline_figure.py data/runs/<run> --cav cav0 --frame 31264 --out docs/images/pipeline_stages.png

Calls the same functions, with the same default settings, as `python -m cavlidar.backend.processing`, but in
memory: nothing is written into the run folder. Ground-truth boxes are drawn for orientation only; the processing
never reads them. The drivable area needs the map's OpenDRIVE file (server.carla_root, the editor's CARLA folder or
$CARLA_ROOT).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from cavlidar.backend.geometry.transforms import box_corners_2d, pose_to_matrix  # noqa: E402
from cavlidar.backend.processing.__main__ import (_drivable_area, _ego_geometry, _lidar_range,  # noqa: E402
                                                  _run_config)
from cavlidar.backend.processing.bev_grid import bev_from_points  # noqa: E402
from cavlidar.backend.processing.ego_mask import ego_mask  # noqa: E402
from cavlidar.backend.processing.ground import estimate_seed_plane, patchwise_ground_mask  # noqa: E402
from cavlidar.backend.processing.road_filter import road_mask  # noqa: E402
from cavlidar.backend.processing.sectors import sector_metadata, static_sectors  # noqa: E402

VIEW = 60.0            # metres, half side of the panels = BEV extent
# colours (light theme of the project's figures)
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#c9c8c3"
GRID, SURFACE = "#e4e3df", "#fcfcfb"
KEPT, OBSTACLE, TRUTH = "#2a78d6", "#eb6834", "#1baf7a"


def run_chain(run_dir, frame, cav):
    """Every intermediate result of the processing chain for one CAV of one frame."""
    data_dir = os.path.join(run_dir, "frames", "{:06d}".format(frame))
    with open(os.path.join(data_dir, "meta.json"), encoding="utf-8") as fh:
        meta = json.load(fh)
    raw = np.load(os.path.join(data_dir, cav + ".npy"))
    pose = meta["cavs"][cav]["sensor_pose"]
    geometry = _ego_geometry(_run_config(run_dir), meta, cav)
    ego = ego_mask(raw, geometry)
    on_road = road_mask(raw, pose, _drivable_area(meta["town"], 0.5))
    kept = raw[on_road & ~ego]
    seed = estimate_seed_plane(kept, 0.18, max_slope_deg=15.0)
    ground = patchwise_ground_mask(kept, seed_plane=seed)
    non_ground = kept[~ground]
    bev = bev_from_points(non_ground, 0.5, (-VIEW, VIEW, -VIEW, VIEW))
    sectors = static_sectors(4, math.pi / 4)
    sector_meta = sector_metadata(raw, non_ground, sectors, pose, _lidar_range(run_dir))
    return {"meta": meta, "pose": pose, "raw": raw, "ego": ego, "on_road": on_road, "kept": kept,
            "ground": ground, "non_ground": non_ground, "bev": bev, "sectors": sectors,
            "sector_meta": sector_meta, "sensor_height": None if seed is None else -seed[2]}


def boxes_in_sensor_frame(meta, pose, cav):
    """(name, 4 x 2 corners) of every other vehicle's ground-truth box, sensor frame (x forward, y right)."""
    to_sensor = np.linalg.inv(pose_to_matrix(pose))
    own = meta["cavs"][cav].get("vehicle_id")
    out = []
    for v in meta.get("vehicles", []):
        if v.get("id") == own:
            continue
        corners = box_corners_2d(v["pose"], v["extent"], v["center_offset"])
        z = v["pose"]["z"] + v["center_offset"][2]
        local = np.column_stack((corners, np.full(4, z), np.ones(4))) @ to_sensor.T
        if np.abs(local[:, :2]).min(axis=0).max() < VIEW:
            out.append((v["name"], local[:, :2]))
    return out


def figure(chain, title, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.edgecolor": GRID, "font.family": "DejaVu Sans"})
    fig, axes = plt.subplots(2, 2, figsize=(11.5, 12.4), facecolor=SURFACE)
    raw, kept, non_ground = chain["raw"], chain["kept"], chain["non_ground"]
    on_road, ego, ground = chain["on_road"], chain["ego"], chain["ground"]

    def scatter(ax, pts, color, size=0.35):
        # horizontal = y (right), vertical = x (forward): the layout of the BEV grid
        ax.scatter(pts[:, 1], pts[:, 0], s=size, c=color, linewidths=0, rasterized=True)

    def dot(color, label):
        return Line2D([], [], ls="", marker="o", ms=5, color=color, label=label)

    a, b, c, d = axes.ravel()
    scatter(a, raw, INK2)
    a.set_title("1  Raw sweep: {:,} returns (sensor frame)".format(len(raw)), loc="left")
    a.legend(handles=[dot(INK2, "LiDAR return")], loc="lower left", frameon=False)

    scatter(b, raw[~on_road & ~ego], MUTED)
    scatter(b, kept, KEPT)
    if ego.any():
        scatter(b, raw[ego], OBSTACLE, 2.0)
    b.set_title("2  Ego mask + drivable-road filter: {:,} kept".format(len(kept)), loc="left")
    b.legend(handles=[dot(KEPT, "on the drivable road ({:.0f} %)".format(100.0 * len(kept) / len(raw))),
                      dot(MUTED, "off-road, removed ({:,})".format(int((~on_road & ~ego).sum()))),
                      dot(OBSTACLE, "own body, removed ({:,})".format(int(ego.sum())))],
             loc="lower left", frameon=False)

    scatter(c, kept[ground], MUTED)
    scatter(c, non_ground, OBSTACLE, 1.2)
    c.set_title("3  Ground removal: {:,} non-ground points".format(len(non_ground)), loc="left")
    height = chain["sensor_height"]
    c.legend(handles=[dot(OBSTACLE, "non-ground (obstacles)"),
                      dot(MUTED, "ground, removed ({:,})".format(int(ground.sum())))],
             loc="lower left", frameon=False,
             title=None if height is None else "sensor height from the data: {:.2f} m".format(height),
             title_fontsize=9)

    bev = chain["bev"]
    x0, x1, y0, y1 = bev.xy_bounds
    d.imshow(bev.occupancy, cmap=matplotlib.colors.ListedColormap([SURFACE, INK]), origin="upper",
             extent=(y0, y1, x0, x1), interpolation="nearest")
    for name, corners in chain["boxes"]:
        ring = np.vstack([corners, corners[:1]])
        d.plot(ring[:, 1], ring[:, 0], color=TRUTH, lw=1.0)
    reach = VIEW * 1.5
    for (start, end), meta in zip(chain["sectors"], chain["sector_meta"]):
        d.plot([0, reach * math.sin(start)], [0, reach * math.cos(start)], color=INK2, lw=0.9, ls=(0, (4, 3)))
        mid = (start + end) / 2.0
        d.text(0.62 * VIEW * math.sin(mid), 0.62 * VIEW * math.cos(mid),
               "s{}\n{:,} pts, {:.2f} MB\n$\\tilde{{S}}$ = {:.3f}".format(meta["index"], meta["points"],
                                                                         meta["size_mb"], meta["quality"]),
               ha="center", va="center", fontsize=8.5, color=INK,
               bbox={"boxstyle": "round,pad=0.3", "fc": SURFACE, "ec": GRID, "lw": 0.8})
    # zoom on the cells around the CAV, to show the 0.5 m grid
    zoom = d.inset_axes([0.05, 0.61, 0.31, 0.31])
    zoom.imshow(bev.occupancy, cmap=matplotlib.colors.ListedColormap([SURFACE, INK]), origin="upper",
                extent=(y0, y1, x0, x1), interpolation="nearest")
    for name, corners in chain["boxes"]:
        ring = np.vstack([corners, corners[:1]])
        zoom.plot(ring[:, 1], ring[:, 0], color=TRUTH, lw=1.2)
    zoom.plot(0, 0, marker="^", ms=7, color=INK, ls="")
    zoom.set_xlim(-8, 8)
    zoom.set_ylim(-14, 2)
    zoom.set_xticks(np.arange(-8, 8.1, 0.5), minor=True)
    zoom.set_yticks(np.arange(-14, 2.1, 0.5), minor=True)
    zoom.grid(which="minor", color=GRID, lw=0.4)
    zoom.tick_params(which="both", length=0, labelsize=7)
    zoom.set_facecolor(SURFACE)
    zoom.set_title("zoom: 16 x 16 m behind the CAV", fontsize=8, color=INK2)
    d.indicate_inset_zoom(zoom, edgecolor=INK2)
    d.set_title("4  BEV 0.5 m: {:,} occupied cells, 4 static sectors".format(int(bev.occupancy.sum())), loc="left")
    d.legend(handles=[Line2D([], [], ls="", marker="s", ms=6, color=INK, label="occupied cell (≥ 1 return)"),
                      Line2D([], [], color=TRUTH, lw=1.2, label="ground-truth box (display only)"),
                      Line2D([], [], color=INK2, lw=0.9, ls=(0, (4, 3)), label="sector boundary")],
             loc="lower left", frameon=True, facecolor=SURFACE, edgecolor=GRID)

    for ax in axes.ravel():
        ax.set_facecolor(SURFACE)
        ax.set_xlim(-VIEW, VIEW)
        ax.set_ylim(-VIEW, VIEW)
        ax.set_aspect("equal")
        ax.plot(0, 0, marker="^", ms=8, color=INK, ls="")  # the CAV, facing up (+x)
        ax.set_xlabel("y, to the right (m)")
        ax.set_ylabel("x, forward (m)")
        ax.grid(color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    fig.suptitle(title, x=0.02, ha="left", fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_path, dpi=110, facecolor=SURFACE)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    parser.add_argument("run_dir")
    parser.add_argument("--cav", default="cav0")
    parser.add_argument("--frame", type=int, default=None, help="default: last frame of the run")
    parser.add_argument("--out", default=os.path.join(ROOT, "docs", "images", "pipeline_stages.png"))
    args = parser.parse_args(argv)
    frames = sorted(int(f) for f in os.listdir(os.path.join(args.run_dir, "frames")) if f.isdigit())
    frame = frames[-1] if args.frame is None else args.frame
    chain = run_chain(args.run_dir, frame, args.cav)
    chain["boxes"] = boxes_in_sensor_frame(chain["meta"], chain["pose"], args.cav)
    run = os.path.basename(os.path.normpath(args.run_dir))
    figure(chain, "One LiDAR sweep through the processing chain ({}, {}, frame {})".format(run, args.cav, frame),
           args.out)
    print("raw {}, ego {}, off-road {}, ground {}, non-ground {}, occupied cells {}".format(
        len(chain["raw"]), int(chain["ego"].sum()), int((~chain["on_road"] & ~chain["ego"]).sum()),
        int(chain["ground"].sum()), len(chain["non_ground"]), int(chain["bev"].occupancy.sum())))
    print("saved", args.out)


if __name__ == "__main__":
    main()
