"""
Command-line entry point.

    python main.py check
    python main.py drive  --seconds 40
    python main.py record --config config/experiments/e02_paper_10cavs.yaml --name dense
    python main.py record --set lidar.channels=64 --set scenario.num_cavs=6
    python main.py show   data/runs/<run_folder> --view world
    python main.py editor --map Town05            # place vehicles by hand (window)
    python main.py xosc-import scenario.xosc --name qgis_trial   # OpenSCENARIO -> plan + experiment
    python main.py xosc-export config/spawn_plans/figure2_town03.json figure2.xosc

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
    editor = sub.add_parser("editor", parents=[common], help="place vehicles by hand on any map (window)")
    editor.add_argument("--map", default=None, help="map to open, e.g. Town05 (default: last used)")
    editor.add_argument("--plan", default=None, help="spawn plan to open (.json)")
    xin = sub.add_parser("xosc-import", parents=[common],
                         help="OpenSCENARIO file -> spawn plan + experiment YAML")
    xin.add_argument("xosc")
    xin.add_argument("--name", required=True, help="name of the plan / experiment files")
    xin.add_argument("--extra-traffic", type=int, default=0, help="random vehicles added to the planned ones")
    xin.add_argument("--parked", action="store_true", help="vehicles do not drive")
    xin.add_argument("--frames", type=int, default=10)
    xout = sub.add_parser("xosc-export", parents=[common], help="spawn plan -> OpenSCENARIO file")
    xout.add_argument("plan")
    xout.add_argument("xosc")
    xout.add_argument("--duration", type=float, default=60.0, help="seconds before the scenario stops")
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
        elif args.command == "editor":
            api.open_editor(cfg, args.map, args.plan)
        elif args.command == "xosc-import":
            _, yaml_path, _ = api.import_openscenario(cfg, args.xosc, args.name, args.extra_traffic,
                                                      not args.parked, args.frames)
            print("[SUCCESS] Record it with: python main.py record --config {} --name {}".format(
                yaml_path.replace("\\", "/"), args.name))
        elif args.command == "xosc-export":
            print("[SUCCESS] Saved {}".format(api.export_openscenario(cfg, args.plan, args.xosc, args.duration)))
        return 0
    except CavLidarError as exc:
        logging.error("%s: %s", type(exc).__name__, exc)
        return 1
    except KeyboardInterrupt:
        logging.warning("Interrupted; CARLA actors were cleaned up.")
        return 130


if __name__ == "__main__":
    sys.exit(main())
