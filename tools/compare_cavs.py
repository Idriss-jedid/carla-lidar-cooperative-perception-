"""
Every CAV of a processed frame, before and after filtering.

    python tools/compare_cavs.py data/runs/<run>              # last processed frame
    python tools/compare_cavs.py data/runs/<run> --frame 31264

Writes into <run>/processed/<frame>/, each figure on its own:
    world_raw.png            the raw point clouds of every CAV superposed in the world frame (colour = CAV), CARLA's
                             ground-truth boxes in grey
    world_filtered.png       the non-ground points left after ego mask + road filter + ground removal; a box is solid
                             when a CAV sees the car (>= 10 points), dashed when none does
    world_bev.png            the BEV of all CAVs together (points per 0.5 m cell), with a zoom
    vehicles.png             points on each car before -> after filtering, and which CAVs see it
    cavN_before_after.png    each CAV in its own frame: raw sweep | after filtering | BEV (points per cell)
    vehicles.csv             per vehicle and CAV: distance, points on the body raw / after road filter / after ground
                             removal, BEV cells on its box
    cavs.csv                 per CAV: points at each stage, BEV cells, vehicles seen before / after
and prints both tables. Ground-truth boxes are used only to count and to draw; the processing never reads them.
The frame must be processed first (python main.py process <run>).
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from cavlidar.backend.recording import storage  # noqa: E402
from cavlidar.backend.visualization import pipeline  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description="Every CAV before and after filtering")
    parser.add_argument("run_dir")
    parser.add_argument("--frame", type=int, default=None, help="default: the last processed frame")
    args = parser.parse_args(argv)
    frames = [f for f in storage.list_frames(args.run_dir) if pipeline.is_processed(args.run_dir, f)]
    if not frames:
        sys.exit("No processed frame in {}: run python main.py process {}".format(args.run_dir, args.run_dir))
    frame = frames[-1] if args.frame is None else args.frame
    out = pipeline.compare(args.run_dir, frame)

    print("Frame {}\n".format(frame))
    print("{:5} {:>7} {:>6} {:>8} {:>8} {:>7} {:>10} {:>6} {:>16} {:>14}".format(
        "CAV", "raw", "ego", "off-road", "ground", "non-gr.", "BEV cells", "", "vehicle points", "vehicles seen"))
    for cav, s in sorted(out["summaries"].items()):
        print("{:5} {:>7,} {:>6,} {:>8,} {:>8,} {:>7,} {:>10,} {:>6} {:>7,} -> {:>6,} {:>6} -> {:>2}".format(
            cav, s["raw"], s["ego"], s["offroad"], s["ground"], s["non_ground"], s["cells"], "",
            s["vehicle_points_raw"], s["vehicle_points_after"], s["seen_raw"], s["seen_after"]))
    rows = out["rows"]
    print("\n{:14} {:24} {:>8} {:>8} {:>8} {:>6}  seen by".format("vehicle", "model", "raw", "road", "non-gr.",
                                                                   "cells"))
    for r in rows:
        if r["raw"] == 0:
            continue
        print("{:14} {:24} {:>8,} {:>8,} {:>8,} {:>6,}  {}".format(r["name"], r["model"][:24], r["raw"], r["road"],
                                                                   r["non_ground"], r["cells"],
                                                                   ", ".join(r["seen_by"]) or "-"))
    print("\n{} vehicles; {} seen by at least one CAV (>= {} points after filtering); {} got no point at all".format(
        len(rows), sum(1 for r in rows if r["seen_by"]), pipeline.SEEN_POINTS, sum(1 for r in rows if r["raw"] == 0)))
    for path in out["figures"].values():
        print("->", path)
    print("->", out["vehicles"])
    print("->", out["summary"])


if __name__ == "__main__":
    main()
