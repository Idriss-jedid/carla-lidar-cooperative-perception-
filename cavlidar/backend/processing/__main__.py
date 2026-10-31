"""Run: python -m cavlidar.backend.processing data/runs/<run_folder>"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from .bev_grid import bev_from_points
from .sectors import bev_sector_grid, sector_metadata, static_sectors
from .ground import estimate_seed_plane, patchwise_ground_mask
from .road_filter import load_drivable_area, road_mask


def _preview(path, raw, nonground, bev, name, sectors=None, metadata=None):
    # Import Matplotlib only when preview PNGs are requested.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    for ax, pts, title in [(axes[0], raw, "Raw LiDAR"),
                           (axes[1], nonground, "Non-ground (prototype)")]:
        if len(pts):
            ax.scatter(pts[:, 1], pts[:, 0], s=0.2, c=pts[:, 2], cmap="viridis")
        ax.set_xlim(bev.xy_bounds[2], bev.xy_bounds[3])
        ax.set_ylim(bev.xy_bounds[0], bev.xy_bounds[1])
        ax.set_aspect("equal")
        ax.set_xlabel("y right (m)")
        ax.set_ylabel("x forward (m)")
        ax.set_title(title)
    axes[2].imshow(bev.occupancy, cmap="gray_r", origin="upper",
                   extent=(bev.xy_bounds[2], bev.xy_bounds[3],
                           bev.xy_bounds[0], bev.xy_bounds[1]))
    axes[2].set_xlabel("y right (m)")
    axes[2].set_ylabel("x forward (m)")
    axes[2].set_title("Non-ground BEV occupancy")
    if sectors:  # sector boundaries (y right, x forward) and each sector's quality
        reach = bev.xy_bounds[1]
        for (start, end), meta in zip(sectors, metadata):
            axes[2].plot([0, reach * 1.5 * math.sin(start)], [0, reach * 1.5 * math.cos(start)],
                         color="tab:red", lw=1)
            mid = (start + end) / 2
            axes[2].text(0.55 * reach * math.sin(mid), 0.55 * reach * math.cos(mid),
                         "s{}\nS~={:.3f}".format(meta["index"], meta["quality"]),
                         color="tab:red", ha="center", va="center", fontsize=8)
        axes[2].set_xlim(bev.xy_bounds[2], bev.xy_bounds[3])
        axes[2].set_ylim(bev.xy_bounds[0], bev.xy_bounds[1])
        axes[2].set_title("Non-ground BEV, {} static sectors".format(len(sectors)))
    fig.suptitle(name)
    fig.tight_layout()
    fig.savefig(str(path), dpi=110)
    plt.close(fig)


def process_frame(run_dir, frame=None, patch_size=8.0, threshold=0.18,
                  iterations=70, resolution=0.5, view_range=60.0, preview=True,
                  max_step=0.5, max_slope=15.0, road_filter=True, road_margin=0.5,
                  sector_count=4, sector_start=math.pi / 4, downsample=0.2):
    """Process one stored multi-CAV frame without connecting to CARLA.

    Original data/runs/.../frames/ is NEVER modified. The output goes to
    data/runs/.../processed/<six-digit-frame>/ .
    """
    root = Path(run_dir)
    frames_dir = root / "frames"
    if not frames_dir.is_dir():
        raise FileNotFoundError("Run folder has no frames/: {}".format(root))
    frames = sorted(int(p.name) for p in frames_dir.iterdir()
                    if p.is_dir() and p.name.isdigit())
    if not frames:
        raise ValueError("No recorded frames found in {}".format(frames_dir))
    frame = frames[-1] if frame is None else int(frame)
    data_dir = frames_dir / "{:06d}".format(frame)
    with open(str(data_dir / "meta.json"), encoding="utf-8") as fh:
        meta = json.load(fh)
    if view_range <= 0:
        raise ValueError("view_range must be positive")
    bounds = (-view_range, view_range, -view_range, view_range)
    out_dir = root / "processed" / "{:06d}".format(frame)
    out_dir.mkdir(parents=True, exist_ok=True)
    area = _drivable_area(meta.get("town"), road_margin) if road_filter else None
    sectors = static_sectors(sector_count, sector_start)
    lidar_range = _lidar_range(root)
    results = {}
    metadata = {"frame": frame, "town": meta.get("town"), "timestamp": meta.get("timestamp"),
                "sectorization": "static", "cavs": {}}
    for name in sorted(meta["cavs"]):
        source = data_dir / "{}.npy".format(name)
        if not source.is_file():
            raise FileNotFoundError("Missing .npy scan for {}: {}".format(name, source))
        raw = np.load(str(source), allow_pickle=False)
        on_road = np.ones(len(raw), dtype=bool)
        if area is not None:  # paper: isolate the drivable road region first
            on_road = road_mask(raw, meta["cavs"][name]["sensor_pose"], area)
            np.save(str(out_dir / "{}_offroad.npy".format(name)), raw[~on_road])
        all_points, raw = raw, raw[on_road]
        seed_plane = estimate_seed_plane(raw, threshold, max_slope_deg=max_slope)
        mask = patchwise_ground_mask(raw, patch_size=patch_size,
                                     residual_threshold=threshold,
                                     iterations=iterations, max_slope_deg=max_slope,
                                     max_step=max_step, seed_plane=seed_plane)
        non_ground = raw[~mask]
        bev = bev_from_points(non_ground, resolution=resolution, xy_bounds=bounds)
        np.save(str(out_dir / "{}_ground.npy".format(name)), raw[mask])
        np.save(str(out_dir / "{}_nonground.npy".format(name)), non_ground)
        np.save(str(out_dir / "{}_bev_occupancy.npy".format(name)), bev.occupancy)
        np.save(str(out_dir / "{}_bev_intensity.npy".format(name)), bev.mean_intensity)
        np.save(str(out_dir / "{}_bev_count.npy".format(name)), bev.counts)
        sector_meta = sector_metadata(all_points, non_ground, sectors, meta["cavs"][name]["sensor_pose"],
                                      lidar_range, downsample)
        np.save(str(out_dir / "{}_bev_sector.npy".format(name)),
                bev_sector_grid(bev.occupancy.shape, bounds, sectors))
        metadata["cavs"][name] = {"vehicle_id": meta["cavs"][name].get("vehicle_id"),
                                  "sensor_pose": meta["cavs"][name]["sensor_pose"], "sectors": sector_meta}
        if preview:
            _preview(out_dir / "{}_preview.png".format(name), all_points, non_ground, bev, name,
                     sectors, sector_meta)
        results[name] = {"raw_points": int(len(all_points)),
                         "offroad_points": int((~on_road).sum()),
                         "ground_points": int(mask.sum()),
                         "non_ground_points": int(len(non_ground)),
                         "occupied_bev_cells": int(bev.occupancy.sum()),
                         "estimated_sensor_height_m": (round(float(-seed_plane[2]), 3)
                                                       if seed_plane is not None else None),
                         "bev_shape": list(bev.occupancy.shape),
                         "sectors": [{k: m[k] for k in ("index", "points", "size_mb", "non_ground_points",
                                                         "downsampled_points", "quality")} for m in sector_meta]}
        print("{}: raw={}, off-road={}, ground={}, non-ground={}, occupied BEV cells={}".format(
            name, len(all_points), (~on_road).sum(), mask.sum(), len(non_ground), bev.occupancy.sum()))
    summary = {
        "frame": frame,
        "method": ("drivable_road_filter(OpenDRIVE 2D lanes)+" if area is not None else "") +
                  "patchwise_local_RANSAC_with_slope_aware_height_prior_PROTOTYPE",
        "important_limitations": [
            ("Drivable region from the map's OpenDRIVE lanes (2D) and the recorded CAV pose; the paper does "
             "not say how it isolates the road." if area is not None else
             "No drivable-road pre-filter (required by paper methodology)."),
            "Unobserved BEV cells are unknown, not proven free.",
            "RANSAC parameters are initial engineering choices, not verified paper hyperparameters.",
            "Sparse patches without accepted neighbours are kept (not removed), e.g. distant slope rings."
        ],
        "settings": {"patch_size_m": patch_size, "ransac_threshold_m": threshold,
                     "ransac_iterations": iterations, "max_step_m": max_step,
                     "max_slope_deg": max_slope, "road_filter": area is not None,
                     "road_margin_m": road_margin, "bev_resolution_m": resolution,
                     "bev_half_range_m": view_range, "sectorization": "static",
                     "sector_count": sector_count, "sector_start_rad": sector_start,
                     "quality_downsample_m": downsample, "footprint_range_m": lidar_range},
        "cavs": results,
    }
    with open(str(out_dir / "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2)
    with open(str(out_dir / "metadata.json"), "w", encoding="utf-8") as fh:  # what goes to the central node
        json.dump(metadata, fh, indent=1)
    return out_dir


def _lidar_range(run_dir, default=120.0):
    """lidar.range of the run (its saved config.yaml), else the paper's 120 m."""
    try:
        with open(str(Path(run_dir) / "config.yaml"), encoding="utf-8") as fh:
            return float(yaml.safe_load(fh)["lidar"]["range"])
    except (OSError, KeyError, TypeError, ValueError):
        return default


def _drivable_area(town, margin):
    """The town's drivable area from its OpenDRIVE file (CARLA folder or the editor's map cache)."""
    from ...api import DEFAULT_CONFIG
    from ...core.config import load_config
    from ...editor import settings
    from ..maps import catalog

    info = catalog.find_map(settings.available_maps(load_config(DEFAULT_CONFIG)), town)
    if info is None:
        raise FileNotFoundError("No OpenDRIVE file for map '{}': set server.carla_root, start CARLA once "
                                "with the editor, or use --no-road-filter".format(town))
    return load_drivable_area(info.name, info.xodr_path, settings.MAPS_CACHE, margin=margin)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline LiDAR ground cleaning + numerical BEV")
    parser.add_argument("run_dir", help="data/runs/<recording-folder>")
    parser.add_argument("--frame", type=int, default=None, help="CARLA frame; default latest")
    parser.add_argument("--patch-size", type=float, default=8.0)
    parser.add_argument("--threshold", type=float, default=0.18)
    parser.add_argument("--iterations", type=int, default=70)
    parser.add_argument("--max-step", type=float, default=0.5,
                        help="largest ground height step between neighbouring patches (m)")
    parser.add_argument("--max-slope", type=float, default=15.0, help="steepest ground plane (degrees)")
    parser.add_argument("--no-road-filter", action="store_true",
                        help="keep the points outside the drivable road region")
    parser.add_argument("--road-margin", type=float, default=0.5, help="metres added around the lanes")
    parser.add_argument("--sectors", type=int, default=4, help="number of static sectors")
    parser.add_argument("--sector-start", type=float, default=45.0,
                        help="start of the first sector, degrees from forward towards the right")
    parser.add_argument("--downsample", type=float, default=0.2,
                        help="voxel size (m) of the averaging downsampling of the quality metric")
    parser.add_argument("--resolution", type=float, default=0.5)
    parser.add_argument("--range", type=float, default=60.0)
    parser.add_argument("--no-preview", action="store_true")
    args = parser.parse_args(argv)
    out_dir = process_frame(args.run_dir, frame=args.frame,
                            patch_size=args.patch_size, threshold=args.threshold,
                            iterations=args.iterations, resolution=args.resolution,
                            view_range=args.range, preview=not args.no_preview,
                            max_step=args.max_step, max_slope=args.max_slope,
                            road_filter=not args.no_road_filter, road_margin=args.road_margin,
                            sector_count=args.sectors, sector_start=math.radians(args.sector_start),
                            downsample=args.downsample)
    print("[SUCCESS] Saved processed data to {}".format(out_dir))


if __name__ == "__main__":
    main()
