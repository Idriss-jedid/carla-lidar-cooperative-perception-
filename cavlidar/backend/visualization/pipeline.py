"""
One CAV's sweep through the processing chain, stage by stage: for figures and for the editor's BEV view.

    stages = stages_from_processed(run_dir, frame, "cav2")   # what `python main.py process` saved
    stages = stages_in_memory(run_dir, frame, "cav2")        # the same chain run again, nothing written
    save_figure(stages, "title", "out.png")                  # raw -> drivable road -> ground removed -> BEV + sectors
    bev_cells_world(occupancy, xy_bounds, sensor_pose)       # occupied BEV cells as world-frame squares

A stages dict holds point arrays (N, 4) in the sensor frame: raw, ego, offroad, ground, non_ground; the BEV
occupancy and its bounds; the sector metadata; the sensor pose and the sensor height found by the ground removal;
and the other vehicles' ground-truth boxes (sensor frame), which are only drawn, for orientation: the processing
never reads them. numpy only; matplotlib is imported when a figure is saved.
"""
import json
import math
import os

import numpy as np

from ...core.constants import FRAME_DIR_FORMAT, FRAMES_DIR, META_FILE
from ..geometry.transforms import box_corners_2d, pose_to_matrix

PROCESSED_DIR = "processed"
FIGURE_SUFFIX = "_pipeline.png"

# colours of the figure (light background)
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#c9c8c3"
GRID, SURFACE = "#e4e3df", "#fcfcfb"
KEPT, OBSTACLE, TRUTH = "#2a78d6", "#eb6834", "#1baf7a"


def frame_dir(run_dir, frame):
    return os.path.join(run_dir, FRAMES_DIR, FRAME_DIR_FORMAT.format(frame))


def processed_dir(run_dir, frame):
    return os.path.join(run_dir, PROCESSED_DIR, FRAME_DIR_FORMAT.format(frame))


def is_processed(run_dir, frame):
    """True when the processing wrote the sector metadata of this frame (its last output)."""
    return os.path.isfile(os.path.join(processed_dir(run_dir, frame), "metadata.json"))


def _json(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def boxes_in_sensor_frame(meta, sensor_pose, own_id=None, half_range=60.0):
    """[(name, 4 x 2 corners)] of the vehicles' ground-truth boxes in the sensor frame (x forward, y right)."""
    to_sensor = np.linalg.inv(pose_to_matrix(sensor_pose))
    out = []
    for v in meta.get("vehicles", []):
        if own_id is not None and v.get("id") == own_id:
            continue
        corners = box_corners_2d(v["pose"], v["extent"], v["center_offset"])
        z = v["pose"]["z"] + v["center_offset"][2]
        local = np.column_stack((corners, np.full(4, z), np.ones(4))) @ to_sensor.T
        if np.abs(local[:, :2]).min(axis=0).max() < half_range:
            out.append((v["name"], local[:, :2]))
    return out


def stages_from_processed(run_dir, frame, cav):
    """The stages of one CAV as saved by the processing in <run>/processed/<frame>/."""
    pdir = processed_dir(run_dir, frame)
    meta = _json(os.path.join(frame_dir(run_dir, frame), META_FILE))
    summary = _json(os.path.join(pdir, "summary.json"))
    metadata = _json(os.path.join(pdir, "metadata.json"))

    def load(suffix):
        path = os.path.join(pdir, "{}_{}.npy".format(cav, suffix))
        return np.load(path) if os.path.isfile(path) else np.zeros((0, 4), dtype=np.float32)

    half = float(summary["settings"]["bev_half_range_m"])
    pose = meta["cavs"][cav]["sensor_pose"]
    return {"frame": frame, "cav": cav, "town": meta.get("town"),
            "raw": np.load(os.path.join(frame_dir(run_dir, frame), cav + ".npy")),
            "ego": load("ego"), "offroad": load("offroad"), "ground": load("ground"),
            "non_ground": load("nonground"), "occupancy": load("bev_occupancy"),
            "xy_bounds": (-half, half, -half, half), "sectors": metadata["cavs"][cav]["sectors"],
            "sensor_pose": pose, "sensor_height": summary["cavs"][cav].get("estimated_sensor_height_m"),
            "boxes": boxes_in_sensor_frame(meta, pose, meta["cavs"][cav].get("vehicle_id"), half)}


def stages_in_memory(run_dir, frame, cav, half_range=60.0):
    """The same stages computed again with the processing's default settings; nothing is written."""
    from ..processing.__main__ import _drivable_area, _ego_geometry, _lidar_range, _run_config
    from ..processing.bev_grid import bev_from_points
    from ..processing.ego_mask import ego_mask
    from ..processing.ground import estimate_seed_plane, patchwise_ground_mask
    from ..processing.road_filter import road_mask
    from ..processing.sectors import sector_metadata, static_sectors

    meta = _json(os.path.join(frame_dir(run_dir, frame), META_FILE))
    raw = np.load(os.path.join(frame_dir(run_dir, frame), cav + ".npy"))
    pose = meta["cavs"][cav]["sensor_pose"]
    ego = ego_mask(raw, _ego_geometry(_run_config(run_dir), meta, cav))
    on_road = road_mask(raw, pose, _drivable_area(meta["town"], 0.5))
    kept = raw[on_road & ~ego]
    seed = estimate_seed_plane(kept, 0.18, max_slope_deg=15.0)
    ground = patchwise_ground_mask(kept, seed_plane=seed)
    non_ground = kept[~ground]
    bounds = (-half_range, half_range, -half_range, half_range)
    bev = bev_from_points(non_ground, 0.5, bounds)
    sectors = sector_metadata(raw, non_ground, static_sectors(4, math.pi / 4), pose, _lidar_range(run_dir))
    return {"frame": frame, "cav": cav, "town": meta.get("town"), "raw": raw, "ego": raw[ego],
            "offroad": raw[~on_road & ~ego], "ground": kept[ground], "non_ground": non_ground,
            "occupancy": bev.occupancy, "xy_bounds": bounds, "sectors": sectors, "sensor_pose": pose,
            "sensor_height": None if seed is None else float(-seed[2]),
            "boxes": boxes_in_sensor_frame(meta, pose, meta["cavs"][cav].get("vehicle_id"), half_range)}


def bev_cells_world(occupancy, xy_bounds, sensor_pose):
    """
    (K, 4, 2) world-frame (x, y) corners of the K occupied BEV cells. Same cell layout as
    processing/bev_grid.py (row 0 = largest x, columns grow with +y); the cells lie in the sensor's
    horizontal plane, placed with the sensor pose like the sector footprints.
    """
    occupancy = np.asarray(occupancy)
    rows, cols = np.nonzero(occupancy)
    nx, ny = occupancy.shape
    x0, x1, y0, y1 = xy_bounds
    rx, ry = (x1 - x0) / float(nx), (y1 - y0) / float(ny)
    xa, xb = x1 - (rows + 1) * rx, x1 - rows * rx
    ya, yb = y0 + cols * ry, y0 + (cols + 1) * ry
    local = np.stack([np.stack([xa, xa, xb, xb], axis=1), np.stack([ya, yb, yb, ya], axis=1)], axis=2)
    m = pose_to_matrix(sensor_pose)
    return local @ m[:2, :2].T + m[:2, 3]


def save_figure(stages, title, out_path, dpi=110):
    """Four panels: raw sweep, ego mask + road filter, ground removal, BEV with the static sectors."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D

    plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.edgecolor": GRID, "font.family": "DejaVu Sans"})
    x0, x1, y0, y1 = stages["xy_bounds"]
    view = max(abs(v) for v in stages["xy_bounds"])
    raw, ego, offroad = stages["raw"], stages["ego"], stages["offroad"]
    ground, non_ground = stages["ground"], stages["non_ground"]
    kept = np.vstack([ground, non_ground]) if len(ground) + len(non_ground) else np.zeros((0, 4))
    occupancy = stages["occupancy"]
    cells = ListedColormap([SURFACE, INK])

    fig, axes = plt.subplots(2, 2, figsize=(11.5, 12.4), facecolor=SURFACE)

    def scatter(ax, pts, color, size=0.35):
        # horizontal = y (right), vertical = x (forward): the layout of the BEV grid
        if len(pts):
            ax.scatter(pts[:, 1], pts[:, 0], s=size, c=color, linewidths=0, rasterized=True)

    def dot(color, label):
        return Line2D([], [], ls="", marker="o", ms=5, color=color, label=label)

    def draw_boxes(ax, width):
        for _, corners in stages["boxes"]:
            ring = np.vstack([corners, corners[:1]])
            ax.plot(ring[:, 1], ring[:, 0], color=TRUTH, lw=width)

    a, b, c, d = axes.ravel()
    scatter(a, raw, INK2)
    a.set_title("1  Raw sweep: {:,} returns (sensor frame)".format(len(raw)), loc="left")
    a.legend(handles=[dot(INK2, "LiDAR return")], loc="lower left", frameon=False)

    scatter(b, offroad, MUTED)
    scatter(b, kept, KEPT)
    scatter(b, ego, OBSTACLE, 2.0)
    b.set_title("2  Ego mask + drivable-road filter: {:,} kept".format(len(kept)), loc="left")
    b.legend(handles=[dot(KEPT, "on the drivable road ({:.0f} %)".format(100.0 * len(kept) / max(len(raw), 1))),
                      dot(MUTED, "off-road, removed ({:,})".format(len(offroad))),
                      dot(OBSTACLE, "own body, removed ({:,})".format(len(ego)))],
             loc="lower left", frameon=False)

    scatter(c, ground, MUTED)
    scatter(c, non_ground, OBSTACLE, 1.2)
    c.set_title("3  Ground removal: {:,} non-ground points".format(len(non_ground)), loc="left")
    height = stages.get("sensor_height")
    c.legend(handles=[dot(OBSTACLE, "non-ground (obstacles)"),
                      dot(MUTED, "ground, removed ({:,})".format(len(ground)))],
             loc="lower left", frameon=False, title_fontsize=9,
             title=None if height is None else "sensor height from the data: {:.2f} m".format(height))

    d.imshow(occupancy, cmap=cells, vmin=0, vmax=1, origin="upper", extent=(y0, y1, x0, x1), interpolation="nearest")
    draw_boxes(d, 1.0)
    reach = view * 1.5
    for meta in stages["sectors"]:
        start, end = meta["start_rad"], meta["end_rad"]
        d.plot([0, reach * math.sin(start)], [0, reach * math.cos(start)], color=INK2, lw=0.9, ls=(0, (4, 3)))
        mid = (start + end) / 2.0
        d.text(0.62 * view * math.sin(mid), 0.62 * view * math.cos(mid),
               "s{}\n{:,} pts, {:.2f} MB\n$\\tilde{{S}}$ = {:.3f}".format(meta["index"], meta["points"],
                                                                         meta["size_mb"], meta["quality"]),
               ha="center", va="center", fontsize=8.5, color=INK,
               bbox={"boxstyle": "round,pad=0.3", "fc": SURFACE, "ec": GRID, "lw": 0.8})
    # zoom on the cells around the CAV, to show the 0.5 m grid
    zoom = d.inset_axes([0.05, 0.61, 0.31, 0.31])
    zoom.imshow(occupancy, cmap=cells, vmin=0, vmax=1, origin="upper", extent=(y0, y1, x0, x1),
                interpolation="nearest")
    draw_boxes(zoom, 1.2)
    zoom.plot(0, 0, marker="^", ms=7, color=INK, ls="")
    zoom.set_xlim(-8, 8)
    zoom.set_ylim(-8, 8)
    zoom.set_xticks(np.arange(-8, 8.1, 0.5), minor=True)
    zoom.set_yticks(np.arange(-8, 8.1, 0.5), minor=True)
    zoom.grid(which="minor", color=GRID, lw=0.4)
    zoom.tick_params(which="both", length=0, labelsize=7)
    zoom.set_facecolor(SURFACE)
    zoom.set_title("zoom: 16 x 16 m around the CAV", fontsize=8, color=INK2)
    d.indicate_inset_zoom(zoom, edgecolor=INK2)
    d.set_title("4  BEV {:g} m: {:,} occupied cells, {} static sectors".format(
        (x1 - x0) / occupancy.shape[0], int(occupancy.sum()), len(stages["sectors"])), loc="left")
    d.legend(handles=[Line2D([], [], ls="", marker="s", ms=6, color=INK, label="occupied cell (≥ 1 return)"),
                      Line2D([], [], color=TRUTH, lw=1.2, label="ground-truth box (display only)"),
                      Line2D([], [], color=INK2, lw=0.9, ls=(0, (4, 3)), label="sector boundary")],
             loc="lower left", frameon=True, facecolor=SURFACE, edgecolor=GRID)

    for ax in axes.ravel():
        ax.set_facecolor(SURFACE)
        ax.set_xlim(-view, view)
        ax.set_ylim(-view, view)
        ax.set_aspect("equal")
        ax.plot(0, 0, marker="^", ms=8, color=INK, ls="")  # the CAV, facing up (+x)
        ax.set_xlabel("y, to the right (m)")
        ax.set_ylabel("x, forward (m)")
        ax.grid(color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    fig.suptitle(title, x=0.02, ha="left", fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_path, dpi=dpi, facecolor=SURFACE)
    plt.close(fig)
    return out_path


# ====================================================================== every CAV, before and after filtering
# One colour per CAV: these four pass the all-pairs colour checks on a light surface (CVD in the 6-8 band, so every
# CAV is also named next to its marker). With more CAVs (the paper goes up to 10) colours are not reused, which would
# give two CAVs the same one: every CAV gets the first colour and is told apart by its label and its own figure.
# The vehicle counts use the ground truth only to count.
CAV_COLORS = ("#2a78d6", "#eda100", "#1baf7a", "#e34948")
SEEN_POINTS = 10          # a vehicle counts as seen with at least this many points on its body
BODY_MARGIN = 0.3         # metres around the box; points lower than 0.3 m above the box bottom are not counted


def cav_color(i, count):
    """Colour of CAV number `i` of `count`: its own up to len(CAV_COLORS) CAVs, the same one for all beyond."""
    return CAV_COLORS[i] if count <= len(CAV_COLORS) else CAV_COLORS[0]


def _cav_point_handles(stages, size=7):
    """Legend entries for the points: one per CAV, or one for all when they share a colour."""
    from matplotlib.lines import Line2D
    names = sorted(stages)
    if len(names) > len(CAV_COLORS):
        return [Line2D([], [], ls="", marker="o", ms=size, color=CAV_COLORS[0],
                       label="points of the {} CAVs (one colour: more CAVs than colours)".format(len(names)))]
    return [Line2D([], [], ls="", marker="o", ms=size, color=cav_color(i, len(names)), label="points of " + cav)
            for i, cav in enumerate(names)]


def all_stages(run_dir, frame):
    """{cav: stages_from_processed(...)} for every CAV of a processed frame."""
    meta = _json(os.path.join(frame_dir(run_dir, frame), META_FILE))
    return {cav: stages_from_processed(run_dir, frame, cav) for cav in sorted(meta["cavs"])}


def to_world(points, pose):
    if not len(points):
        return np.zeros((0, 3))
    m = pose_to_matrix(pose)
    return np.asarray(points)[:, :3].astype(np.float64) @ m[:3, :3].T + m[:3, 3]


def on_body(world_xyz, vehicle):
    """True for the world points on the vehicle's body (its box + BODY_MARGIN, above the lowest 0.3 m)."""
    if not len(world_xyz):
        return np.zeros(0, dtype=bool)
    m = np.linalg.inv(pose_to_matrix(vehicle["pose"]))
    local = np.c_[world_xyz, np.ones(len(world_xyz))] @ m.T
    (ex, ey, ez), (ox, oy, oz) = vehicle["extent"], vehicle["center_offset"]
    return ((np.abs(local[:, 0] - ox) <= ex + BODY_MARGIN) & (np.abs(local[:, 1] - oy) <= ey + BODY_MARGIN) &
            (local[:, 2] >= oz - ez + 0.3) & (local[:, 2] <= oz + ez + BODY_MARGIN))


def vehicle_returns(run_dir, frame, stages=None):
    """
    Per vehicle of the frame, per CAV: distance, points on its body in the raw sweep ("raw"), after the ego mask and
    the road filter ("road"), after the ground removal ("non_ground"), and occupied BEV cells on its box ("cells").
    A CAV's own body is not counted for itself.
    """
    stages = stages or all_stages(run_dir, frame)
    meta = _json(os.path.join(frame_dir(run_dir, frame), META_FILE))
    world = {}
    for cav, s in stages.items():
        pose = s["sensor_pose"]
        road = np.vstack([s["ground"], s["non_ground"]]) if len(s["ground"]) + len(s["non_ground"]) else s["ground"]
        cells = bev_cells_world(s["occupancy"], s["xy_bounds"], pose).mean(axis=1) if s["occupancy"].any() \
            else np.zeros((0, 2))
        world[cav] = {"raw": to_world(s["raw"], pose), "road": to_world(road, pose),
                      "non_ground": to_world(s["non_ground"], pose), "cells": cells, "pose": pose,
                      "own": meta["cavs"][cav].get("vehicle_id")}
    rows = []
    for v in meta.get("vehicles", []):
        row = {"name": v["name"], "model": v["type_id"].split(".", 1)[-1], "role": v["role"], "cavs": {}}
        for cav, w in world.items():
            if v["id"] == w["own"]:
                continue
            counts = {k: int(on_body(w[k], v).sum()) for k in ("raw", "road", "non_ground")}
            cells = np.c_[w["cells"], np.full(len(w["cells"]), v["pose"]["z"] + v["center_offset"][2])]
            m = np.linalg.inv(pose_to_matrix(v["pose"]))
            local = np.c_[cells, np.ones(len(cells))] @ m.T if len(cells) else np.zeros((0, 4))
            ex, ey, _ = v["extent"]
            counts["cells"] = int(((np.abs(local[:, 0] - v["center_offset"][0]) <= ex + 0.5) &
                                   (np.abs(local[:, 1] - v["center_offset"][1]) <= ey + 0.5)).sum())
            counts["dist"] = float(math.hypot(v["pose"]["x"] - w["pose"]["x"], v["pose"]["y"] - w["pose"]["y"]))
            row["cavs"][cav] = counts
        for k in ("raw", "road", "non_ground", "cells"):
            row[k] = sum(c[k] for c in row["cavs"].values())
        row["nearest"] = min(c["dist"] for c in row["cavs"].values()) if row["cavs"] else float("nan")
        row["seen_by"] = [cav for cav, c in row["cavs"].items() if c["non_ground"] >= SEEN_POINTS]
        rows.append(row)
    rows.sort(key=lambda r: (-r["non_ground"], r["nearest"]))
    return rows


def cav_summaries(stages, rows):
    """Per CAV: points at each stage, BEV cells, vehicles with >= SEEN_POINTS on the body before / after."""
    out = {}
    for cav, s in stages.items():
        hit = [r for r in rows if cav in r["cavs"]]
        out[cav] = {"raw": len(s["raw"]), "ego": len(s["ego"]), "offroad": len(s["offroad"]),
                    "road": len(s["ground"]) + len(s["non_ground"]), "ground": len(s["ground"]),
                    "non_ground": len(s["non_ground"]), "cells": int(s["occupancy"].sum()),
                    "seen_raw": sum(r["cavs"][cav]["raw"] >= SEEN_POINTS for r in hit),
                    "seen_after": sum(r["cavs"][cav]["non_ground"] >= SEEN_POINTS for r in hit),
                    "vehicle_points_raw": sum(r["cavs"][cav]["raw"] for r in hit),
                    "vehicle_points_after": sum(r["cavs"][cav]["non_ground"] for r in hit)}
    return out


def write_tables(rows, summaries, folder):
    """vehicles.csv (one line per vehicle and CAV) and cavs.csv (one line per CAV) in `folder`."""
    import csv
    vehicles = os.path.join(folder, "vehicles.csv")
    with open(vehicles, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["vehicle", "model", "role", "cav", "distance_m", "raw_points", "after_road_filter",
                    "after_ground_removal", "bev_cells"])
        for r in rows:
            for cav, c in sorted(r["cavs"].items()):
                w.writerow([r["name"], r["model"], r["role"], cav, round(c["dist"], 1), c["raw"], c["road"],
                            c["non_ground"], c["cells"]])
    cavs = os.path.join(folder, "cavs.csv")
    keys = ["raw", "ego", "offroad", "road", "ground", "non_ground", "cells", "vehicle_points_raw",
            "vehicle_points_after", "seen_raw", "seen_after"]
    with open(cavs, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["cav"] + keys)
        for cav, s in sorted(summaries.items()):
            w.writerow([cav] + [s[k] for k in keys])
    return vehicles, cavs


def _style(plt):
    plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.edgecolor": GRID, "font.family": "DejaVu Sans"})


def _boxes_world(ax, meta, rows=None, colour_seen=INK2, colour_unseen="#a8a7a1"):
    """CARLA's boxes (world frame). With `rows`: solid if a CAV sees the vehicle after filtering, dashed if not;
    without: every box solid grey."""
    seen = {r["name"] for r in rows if r["seen_by"]} if rows is not None else None
    for v in meta.get("vehicles", []):
        c = box_corners_2d(v["pose"], v["extent"], v["center_offset"])
        ring = np.vstack([c, c[:1]])
        if seen is None or v["name"] in seen:
            ax.plot(ring[:, 0], ring[:, 1], color=colour_seen, lw=1.1)
        else:
            ax.plot(ring[:, 0], ring[:, 1], color=colour_unseen, lw=0.9, ls=(0, (3, 2)))


def _sequential(matplotlib, start=0.3):
    """Blues from a visible light blue to dark: a cell with one point must still show on the light surface."""
    import matplotlib.cm as cm
    from matplotlib.colors import ListedColormap
    cmap = ListedColormap(cm.get_cmap("Blues")(np.linspace(start, 1.0, 256)))
    cmap.set_bad(SURFACE)
    return cmap


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style(plt)
    return matplotlib, plt


def _world_extent(stages, half=62.0):
    """A square (x0, x1, y0, y1) holding every CAV's BEV range."""
    xs = [s["sensor_pose"]["x"] for s in stages.values()]
    ys = [s["sensor_pose"]["y"] for s in stages.values()]
    x0, x1, y0, y1 = min(xs) - half, max(xs) + half, min(ys) - half, max(ys) + half
    span = max(x1 - x0, y1 - y0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    return cx - span / 2, cx + span / 2, cy - span / 2, cy + span / 2


def _world_axes(ax, extent):
    x0, x1, y0, y1 = extent
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)            # CARLA: y to the right of x, so inverted for a top view
    ax.set_aspect("equal")
    ax.set_facecolor(SURFACE)
    ax.grid(color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")


def _free_corner(counts, origin, res, extent, keep_clear, size=0.41, margin=0.02):
    """[x, y, w, h] (axes fraction) of the corner of a _world_axes plot that hides the fewest occupied cells and
    none of the `keep_clear` (x, y) points if possible; the top right wins ties."""
    x0, x1, y0, y1 = extent
    occupied = counts > 0
    best = None
    for fx, fy in ((1 - margin - size, 1 - margin - size), (margin, 1 - margin - size),
                   (1 - margin - size, margin), (margin, margin)):
        xa, xb = x0 + fx * (x1 - x0), x0 + (fx + size) * (x1 - x0)
        ya, yb = y1 - (fy + size) * (y1 - y0), y1 - fy * (y1 - y0)     # y axis inverted: top of the axes is y0
        pad = 0.04 * (x1 - x0)
        hidden = sum(xa - pad <= x <= xb + pad and ya - pad <= y <= yb + pad for x, y in keep_clear)
        i0, i1 = [min(max(int((v - origin[0]) / res), 0), occupied.shape[0]) for v in (xa, xb)]
        j0, j1 = [min(max(int((v - origin[1]) / res), 0), occupied.shape[1]) for v in (ya, yb)]
        score = (hidden, int(occupied[i0:i1, j0:j1].sum()))
        if best is None or score < best[0]:
            best = (score, [fx, fy, size, size])
    return best[1]


def _mark_cavs(ax, stages, labels=True):
    for i, cav in enumerate(sorted(stages)):
        p = stages[cav]["sensor_pose"]
        color = cav_color(i, len(stages))
        ax.plot(p["x"], p["y"], marker="*", ms=14, color=color, mec=INK, mew=0.8, ls="")
        if labels:
            ax.annotate(cav, (p["x"], p["y"]), xytext=(7, 7), textcoords="offset points", fontsize=10,
                        fontweight="bold", color=INK,
                        bbox={"boxstyle": "round,pad=0.2", "fc": SURFACE, "ec": color, "lw": 1.2})


def _save(fig, plt, title, out_path, dpi, handles=None, ncol=4, header=0.9):
    """Title (`header` inches tall per line) and a one-row legend under it, then the axes below both."""
    height = fig.get_size_inches()[1]
    lines = title.count("\n") + 1
    fig.suptitle(title, x=0.02, y=1 - 0.12 / height, ha="left", va="top", fontsize=12.5, color=INK)
    top = 1 - (0.2 + 0.28 * lines) / height
    if handles:
        fig.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.02, top), ncol=ncol, frameon=False,
                   fontsize=9.5, handletextpad=0.3, columnspacing=1.4)
        top -= 0.45 / height
    fig.tight_layout(rect=(0, 0, 1, top))
    fig.savefig(out_path, dpi=dpi, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.25)
    plt.close(fig)
    return out_path


def _name(run_dir, frame):
    return "{}, frame {}".format(os.path.basename(os.path.normpath(run_dir)), frame)


def world_points_figure(run_dir, frame, out_path, after=False, stages=None, rows=None, dpi=110):
    """
    after=False: the raw point clouds of every CAV superposed in the world frame (colour = CAV), CARLA's boxes in
    grey. after=True: the non-ground points left after ego mask + road filter + ground removal; a box is solid when
    a CAV sees the vehicle (>= SEEN_POINTS points), dashed when none does.
    """
    from matplotlib.lines import Line2D
    _, plt = _plt()
    stages = stages or all_stages(run_dir, frame)
    meta = _json(os.path.join(frame_dir(run_dir, frame), META_FILE))
    extent = _world_extent(stages)
    x0, x1, y0, y1 = extent
    fig, ax = plt.subplots(figsize=(11, 12), facecolor=SURFACE)
    total = 0
    for i, cav in enumerate(sorted(stages)):
        s = stages[cav]
        pts = s["non_ground"] if after else s["raw"]
        total += len(pts)
        w = to_world(pts, s["sensor_pose"])
        w = w[(w[:, 0] >= x0) & (w[:, 0] <= x1) & (w[:, 1] >= y0) & (w[:, 1] <= y1)]
        ax.scatter(w[:, 0], w[:, 1], s=1.6 if after else 0.25, c=cav_color(i, len(stages)), linewidths=0,
                   rasterized=True)
    handles = _cav_point_handles(stages)
    handles.append(Line2D([], [], ls="", marker="*", ms=12, color=SURFACE, mec=INK, label="CAV (LiDAR)"))
    if after:
        rows = rows if rows is not None else vehicle_returns(run_dir, frame, stages)
        _boxes_world(ax, meta, rows)
        seen = sum(1 for r in rows if r["seen_by"])
        handles += [Line2D([], [], color=INK2, lw=1.2, label="CARLA box, seen (>= {} points)".format(SEEN_POINTS)),
                    Line2D([], [], color="#a8a7a1", lw=1.0, ls=(0, (3, 2)), label="CARLA box, not seen")]
        title = "After filtering (ego mask + road filter + ground removal): {:,} non-ground points of {} CAVs, " \
                "{} of {} vehicles seen".format(total, len(stages), seen, len(rows))
    else:
        _boxes_world(ax, meta)
        handles.append(Line2D([], [], color=INK2, lw=1.2, label="CARLA ground-truth box"))
        title = "Raw point clouds of {} CAVs superposed in the world frame: {:,} points".format(len(stages), total)
    _mark_cavs(ax, stages)
    _world_axes(ax, extent)
    return _save(fig, plt, "{}\n{}".format(_name(run_dir, frame), title), out_path, dpi, handles,
                 ncol=min(len(handles), 4))


def world_bev_figure(run_dir, frame, out_path, stages=None, rows=None, dpi=110):
    """The BEV of every CAV together in the world frame: non-ground points per 0.5 m cell, with a zoom."""
    from matplotlib.colors import LogNorm
    from matplotlib.lines import Line2D
    matplotlib, plt = _plt()
    stages = stages or all_stages(run_dir, frame)
    rows = rows if rows is not None else vehicle_returns(run_dir, frame, stages)
    meta = _json(os.path.join(frame_dir(run_dir, frame), META_FILE))
    extent = _world_extent(stages)
    x0, x1, y0, y1 = extent
    res = 0.5
    nx, ny = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    counts = np.zeros((nx, ny))
    for s in stages.values():
        w = to_world(s["non_ground"], s["sensor_pose"])
        if len(w):
            h, _, _ = np.histogram2d(w[:, 0], w[:, 1], bins=(nx, ny), range=((x0, x0 + nx * res), (y0, y0 + ny * res)))
            counts += h
    shown = np.ma.masked_where(counts.T == 0, counts.T)
    cmap, norm = _sequential(matplotlib), LogNorm(vmin=1, vmax=max(2, counts.max()))
    grid_extent = (x0, x0 + nx * res, y0, y0 + ny * res)
    fig, ax = plt.subplots(figsize=(11.5, 12), facecolor=SURFACE)
    image = ax.imshow(shown, origin="lower", extent=grid_extent, cmap=cmap, norm=norm, interpolation="nearest")
    fig.colorbar(image, ax=ax, fraction=0.035, pad=0.02).set_label("non-ground points per 0.5 m cell", color=INK2)
    _boxes_world(ax, meta, rows)
    _mark_cavs(ax, stages)
    _world_axes(ax, extent)
    vehicles = {v["name"]: v for v in meta.get("vehicles", [])}
    top = next((r for r in rows if r["non_ground"] > 0 and r["name"] in vehicles), None)
    if top is not None:
        vp = vehicles[top["name"]]["pose"]
        keep_clear = [(s["sensor_pose"]["x"], s["sensor_pose"]["y"]) for s in stages.values()] + [(vp["x"], vp["y"])]
        zoom = ax.inset_axes(_free_corner(counts, (x0, y0), res, extent, keep_clear))
        zoom.imshow(shown, origin="lower", extent=grid_extent, cmap=cmap, norm=norm, interpolation="nearest")
        _boxes_world(zoom, meta, rows)
        _mark_cavs(zoom, stages, labels=False)
        zoom.set_xlim(vp["x"] - 12, vp["x"] + 12)
        zoom.set_ylim(vp["y"] + 12, vp["y"] - 12)
        zoom.set_aspect("equal")
        zoom.set_xticks(np.arange(math.floor(vp["x"] - 12), vp["x"] + 12, 0.5), minor=True)
        zoom.set_yticks(np.arange(math.floor(vp["y"] - 12), vp["y"] + 12, 0.5), minor=True)
        zoom.grid(which="minor", color=GRID, lw=0.3)
        zoom.tick_params(which="both", length=0, labelsize=7)
        zoom.set_facecolor(SURFACE)
        zoom.set_title("zoom 24 x 24 m: {} ({} points)".format(top["name"], top["non_ground"]), fontsize=8,
                       color=INK2)
        ax.indicate_inset_zoom(zoom, edgecolor=INK2)
    handles = [Line2D([], [], ls="", marker="s", ms=7, color=cmap(0.6), label="occupied 0.5 m cell"),
               Line2D([], [], ls="", marker="*", ms=12, color=SURFACE, mec=INK, label="CAV (LiDAR)"),
               Line2D([], [], color=INK2, lw=1.2, label="CARLA box, seen"),
               Line2D([], [], color="#a8a7a1", lw=1.0, ls=(0, (3, 2)), label="CARLA box, not seen")]
    title = "BEV of all {} CAVs together: {:,} occupied 0.5 m cells".format(len(stages), int((counts > 0).sum()))
    return _save(fig, plt, "{}\n{}".format(_name(run_dir, frame), title), out_path, dpi, handles)


def vehicles_figure(run_dir, frame, out_path, stages=None, rows=None, dpi=110):
    """Points on each vehicle's body, before and after filtering, all CAVs together; who sees it."""
    _, plt = _plt()
    stages = stages or all_stages(run_dir, frame)
    rows = rows if rows is not None else vehicle_returns(run_dir, frame, stages)
    hit = sorted([r for r in rows if r["raw"] > 0], key=lambda r: r["raw"])
    missing = len(rows) - len(hit)
    fig, ax = plt.subplots(figsize=(11, max(4.0, 1.6 + 0.42 * len(hit))), facecolor=SURFACE)
    yy = np.arange(len(hit))
    ax.barh(yy + 0.2, [r["raw"] for r in hit], height=0.38, color="#b9cdeb", label="before (raw sweep)")
    ax.barh(yy - 0.2, [r["non_ground"] for r in hit], height=0.38, color=CAV_COLORS[0],
            label="after (road filter + ground removal)")
    for y, r in zip(yy, hit):
        who = " ".join(r["seen_by"]) or "(no CAV >= {})".format(SEEN_POINTS)
        ax.text(max(r["raw"], 1) * 1.2, y, "{:,} -> {:,}  {}".format(r["raw"], r["non_ground"], who), va="center",
                fontsize=8.5, color=INK2)
    ax.set_yticks(yy)
    ax.set_yticklabels(["{} ({})".format(r["name"], r["model"][:20]) for r in hit], fontsize=8.5)
    ax.set_xscale("symlog", linthresh=1)
    ax.set_xlim(0, max([r["raw"] for r in hit] + [10]) * 300)
    ax.set_xlabel("points on the vehicle's body, all CAVs together (log scale); right: the CAVs that see it")
    ax.legend(loc="lower right", frameon=True, facecolor=SURFACE, edgecolor=GRID)
    ax.grid(axis="x", color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    before = sum(r["raw"] for r in rows)
    after = sum(r["non_ground"] for r in rows)
    title = "Points on each vehicle, before -> after filtering: {:,} -> {:,} ({:.1f} % kept); {} vehicles got no " \
            "point at all".format(before, after, 100.0 * after / max(before, 1), missing)
    return _save(fig, plt, "{}\n{}".format(_name(run_dir, frame), title), out_path, dpi)


def cav_figure(run_dir, frame, cav, out_path, stages=None, rows=None, dpi=100):
    """One CAV in its own frame: raw sweep | after filtering | BEV (points per 0.5 m cell)."""
    from matplotlib.colors import LogNorm
    from matplotlib.lines import Line2D
    matplotlib, plt = _plt()
    stages = stages or all_stages(run_dir, frame)
    rows = rows if rows is not None else vehicle_returns(run_dir, frame, stages)
    sm = cav_summaries(stages, rows)[cav]
    i = sorted(stages).index(cav)
    s = stages[cav]
    view = max(abs(v) for v in s["xy_bounds"])
    seen = {r["name"] for r in rows if cav in r["cavs"] and r["cavs"][cav]["non_ground"] >= SEEN_POINTS}
    fig, (raw_ax, after_ax, bev_ax) = plt.subplots(1, 3, figsize=(16.5, 7.2), facecolor=SURFACE)
    raw = s["raw"]
    raw_ax.scatter(raw[:, 1], raw[:, 0], s=0.25, c=INK2, linewidths=0, rasterized=True)
    raw_ax.set_title("BEFORE: raw sweep, {:,} points\n{} vehicles with >= {} points on them".format(
        sm["raw"], sm["seen_raw"], SEEN_POINTS), loc="left", fontsize=10)
    if len(s["ground"]):
        after_ax.scatter(s["ground"][:, 1], s["ground"][:, 0], s=0.2, c=MUTED, linewidths=0, rasterized=True)
    ng = s["non_ground"]
    if len(ng):
        after_ax.scatter(ng[:, 1], ng[:, 0], s=1.4, c=cav_color(i, len(stages)), linewidths=0, rasterized=True)
    after_ax.set_title("AFTER: {:,} on the road -> {:,} non-ground\n{} vehicles seen; vehicle points {:,} -> {:,}"
                       .format(sm["road"], sm["non_ground"], sm["seen_after"], sm["vehicle_points_raw"],
                               sm["vehicle_points_after"]), loc="left", fontsize=10)
    x0, x1, y0, y1 = s["xy_bounds"]
    counts = np.zeros(s["occupancy"].shape)
    if len(ng):
        nx = counts.shape[0]
        res = (x1 - x0) / nx
        keep = (ng[:, 0] >= x0) & (ng[:, 0] < x1) & (ng[:, 1] >= y0) & (ng[:, 1] < y1)
        np.add.at(counts, ((nx - 1 - np.floor((ng[keep, 0] - x0) / res)).astype(int),
                           np.floor((ng[keep, 1] - y0) / res).astype(int)), 1)
    shown = np.ma.masked_where(counts == 0, counts)
    img = bev_ax.imshow(shown, origin="upper", extent=(y0, y1, x0, x1), cmap=_sequential(matplotlib),
                        norm=LogNorm(vmin=1, vmax=max(2, counts.max())), interpolation="nearest")
    fig.colorbar(img, ax=bev_ax, fraction=0.045, pad=0.02).set_label("points per cell", color=INK2)
    bev_ax.set_title("BEV 0.5 m: {:,} occupied cells".format(sm["cells"]), loc="left", fontsize=10)
    for ax in (raw_ax, after_ax, bev_ax):
        for name, corners in s["boxes"]:
            ring = np.vstack([corners, corners[:1]])
            if name in seen:
                ax.plot(ring[:, 1], ring[:, 0], color=INK, lw=1.0)
            else:
                ax.plot(ring[:, 1], ring[:, 0], color="#a8a7a1", lw=0.8, ls=(0, (3, 2)))
        ax.plot(0, 0, marker="^", ms=9, color=cav_color(i, len(stages)), mec=INK, ls="")
        ax.set_xlim(-view, view)
        ax.set_ylim(-view, view)
        ax.set_aspect("equal")
        ax.set_facecolor(SURFACE)
        ax.grid(color=GRID, lw=0.5)
        ax.set_axisbelow(True)
        ax.set_xlabel("y, to the right (m)")
        ax.set_ylabel("x, forward (m)")
    handles = [Line2D([], [], ls="", marker="o", ms=5, color=INK2, label="raw return"),
               Line2D([], [], ls="", marker="o", ms=5, color=MUTED, label="ground (removed)"),
               Line2D([], [], ls="", marker="o", ms=5, color=cav_color(i, len(stages)), label="non-ground (kept)"),
               Line2D([], [], color=INK, lw=1.0, label="CARLA box, seen by {} (>= {} points)".format(cav, SEEN_POINTS)),
               Line2D([], [], color="#a8a7a1", lw=0.8, ls=(0, (3, 2)), label="CARLA box, not seen by {}".format(cav))]
    return _save(fig, plt, "{}: {} before and after filtering (its own frame)".format(_name(run_dir, frame), cav),
                 out_path, dpi, handles, ncol=5)


VIEWS = ("world_raw", "world_filtered", "world_bev", "vehicles")


def view_path(run_dir, frame, which, cav=None):
    name = "{}_before_after.png".format(cav) if which == "cav" else which + ".png"
    return os.path.join(processed_dir(run_dir, frame), name)


def make_view(run_dir, frame, which, cav=None, stages=None, rows=None):
    """One figure of a processed frame ('world_raw', 'world_filtered', 'world_bev', 'vehicles', or 'cav' with
    `cav`), plus vehicles.csv and cavs.csv; returns the figure's path."""
    stages = stages or all_stages(run_dir, frame)
    rows = rows if rows is not None else vehicle_returns(run_dir, frame, stages)
    write_tables(rows, cav_summaries(stages, rows), processed_dir(run_dir, frame))
    out = view_path(run_dir, frame, which, cav)
    if which in ("world_raw", "world_filtered"):
        return world_points_figure(run_dir, frame, out, which == "world_filtered", stages, rows)
    if which == "world_bev":
        return world_bev_figure(run_dir, frame, out, stages, rows)
    if which == "vehicles":
        return vehicles_figure(run_dir, frame, out, stages, rows)
    if which == "cav":
        return cav_figure(run_dir, frame, cav, out, stages, rows)
    raise ValueError("unknown view: {}".format(which))


def compare(run_dir, frame):
    """Every figure (each on its own) and both tables of a processed frame, in processed/<frame>/."""
    folder = processed_dir(run_dir, frame)
    stages = all_stages(run_dir, frame)
    rows = vehicle_returns(run_dir, frame, stages)
    summaries = cav_summaries(stages, rows)
    vehicles_csv, cavs_csv = write_tables(rows, summaries, folder)
    figures = {which: make_view(run_dir, frame, which, None, stages, rows) for which in VIEWS}
    figures.update({cav: make_view(run_dir, frame, "cav", cav, stages, rows) for cav in sorted(stages)})
    return {"figures": figures, "vehicles": vehicles_csv, "summary": cavs_csv, "rows": rows, "summaries": summaries}
