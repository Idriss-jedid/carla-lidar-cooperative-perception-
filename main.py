"""
Command-line entry point.

    python main.py check
    python main.py drive  --seconds 40
    python main.py record --config config/experiments/e02_paper_10cavs.yaml --name dense
    python main.py record --set lidar.channels=64 --set scenario.num_cavs=6
    python main.py show   data/runs/<run_folder> --view world

--config may be given several times (later files win); default.yaml is
always loaded first. --set overrides any single parameter.
"""
import argparse
import logging
import sys

from cavlidar import CavLidarError, ViewMode, api


def build_parser():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--config", action="append", default=[],
                        help="experiment YAML file (repeatable)")
    common.add_argument("--set", action="append", default=[], metavar="SECTION.KEY=VALUE",
                        help="override one parameter, e.g. lidar.channels=64 (repeatable)")
    common.add_argument("-v", "--verbose", action="store_true")

    parser = argparse.ArgumentParser(description="CARLA LiDAR data collection for CAVs")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", parents=[common], help="test the connection to CARLA")
    drive = sub.add_parser("drive", parents=[common], help="spawn vehicles and watch them drive")
    drive.add_argument("--seconds", type=float, default=40.0)
    record = sub.add_parser("record", parents=[common], help="record LiDAR sweeps + ground truth")
    record.add_argument("--name", default="run", help="suffix of the run folder")
    show = sub.add_parser("show", parents=[common], help="plot a recorded frame (no CARLA needed)")
    show.add_argument("run_dir")
    show.add_argument("--frame", type=int, default=None)
    show.add_argument("--view", default=ViewMode.BOTH.value, choices=[v.value for v in ViewMode])
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="[%(levelname)s] %(message)s")
    try:
        if args.command == "show":
            api.show_frame(args.run_dir, args.frame, args.view)
            return 0
        cfg = api.load_config(*args.config, overrides=args.set)
        if args.command == "check":
            api.check_connection(cfg)
            print("[SUCCESS] Connected to CARLA, map {} ready.".format(cfg.server.town))
        elif args.command == "drive":
            api.drive_cars(cfg, args.seconds)
        elif args.command == "record":
            run_dir = api.record_lidar(cfg, args.name)
            print("[SUCCESS] Run saved in {}".format(run_dir))
        return 0
    except CavLidarError as exc:
        logging.error("%s: %s", type(exc).__name__, exc)
        return 1
    except KeyboardInterrupt:
        logging.warning("Interrupted; CARLA actors were cleaned up.")
        return 130


if __name__ == "__main__":
    sys.exit(main())
