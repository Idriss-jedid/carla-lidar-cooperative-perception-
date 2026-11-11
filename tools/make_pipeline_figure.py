"""
One recorded LiDAR sweep through the per-CAV processing chain, in four panels:
raw sweep -> ego mask + drivable-road filter -> ground removal -> BEV occupancy with the static sectors.

    python tools/make_pipeline_figure.py data/runs/<run> --cav cav2
    python tools/make_pipeline_figure.py data/runs/<run> --cav cav0 --frame 31264 --out docs/images/pipeline_stages.png
    python tools/make_pipeline_figure.py data/runs/<run> --cav cav0 --from-processed   # read processed/<frame>/

By default the chain is run again in memory with the processing's default settings (nothing is written into the
run folder); --from-processed draws what `python main.py process` saved instead. Ground-truth boxes are drawn for
orientation only; the processing never reads them. The drivable area needs the map's OpenDRIVE file
(server.carla_root, the editor's CARLA folder or $CARLA_ROOT).
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from cavlidar.backend.visualization import pipeline  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description="Figure of one sweep through the processing chain")
    parser.add_argument("run_dir")
    parser.add_argument("--cav", default="cav0")
    parser.add_argument("--frame", type=int, default=None, help="default: last frame of the run")
    parser.add_argument("--from-processed", action="store_true", help="draw the saved outputs of the processing")
    parser.add_argument("--out", default=os.path.join(ROOT, "docs", "images", "pipeline_stages.png"))
    args = parser.parse_args(argv)
    frames = sorted(int(f) for f in os.listdir(os.path.join(args.run_dir, "frames")) if f.isdigit())
    frame = frames[-1] if args.frame is None else args.frame
    load = pipeline.stages_from_processed if args.from_processed else pipeline.stages_in_memory
    stages = load(args.run_dir, frame, args.cav)
    run = os.path.basename(os.path.normpath(args.run_dir))
    pipeline.save_figure(stages, "One LiDAR sweep through the processing chain ({}, {}, frame {})".format(
        run, args.cav, frame), args.out)
    print("raw {}, ego {}, off-road {}, ground {}, non-ground {}, occupied cells {}".format(
        len(stages["raw"]), len(stages["ego"]), len(stages["offroad"]), len(stages["ground"]),
        len(stages["non_ground"]), int(stages["occupancy"].sum())))
    print("saved", args.out)


if __name__ == "__main__":
    main()
