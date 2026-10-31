"""
Public API: the only functions a user needs.

    from cavlidar import load_config, check_connection, drive_cars, record_lidar, show_frame

    cfg = load_config("config/experiments/e01_baseline_4cavs.yaml")
    check_connection(cfg)
    run_dir = record_lidar(cfg, name="baseline")
    show_frame(run_dir)

Each function hides the CARLA details implemented in cavlidar/backend/.
Behaviour is controlled only through the Config (YAML files / overrides).
"""
import logging
import os
import time

from .core import config as _config
from .core.constants import CONFIG_FILE, PREVIEW_SENSOR, PREVIEW_WORLD
from .core.enums import ViewMode

log = logging.getLogger("cavlidar")

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              "config", "default.yaml")


def load_config(*experiment_files, overrides=None):
    """
    Load config/default.yaml, then each experiment file, then overrides
    like "lidar.channels=64". Prints a warning for risky combinations.
    """
    base = DEFAULT_CONFIG if os.path.exists(DEFAULT_CONFIG) else None
    cfg = _config.load_config(base, *experiment_files, overrides=overrides)
    for warning in _config.validate(cfg):
        log.warning(warning)
    return cfg


def check_connection(cfg):
    """
    Connect to CARLA and report versions, map and spawn points.
    Returns a dict with that information; raises if anything is wrong.
    """
    from .backend.simulator import connection

    srv = cfg.server
    client = connection.connect(srv.host, srv.port, srv.timeout)
    client_v, server_v = connection.versions(client)
    world = connection.ensure_town(client, srv.town, srv.allow_map_change)
    info = {
        "client_version": client_v,
        "server_version": server_v,
        "map": connection.map_name(world),
        "spawn_points": len(world.get_map().get_spawn_points()),
        "synchronous_mode": world.get_settings().synchronous_mode,
    }
    log.info("Client %s / server %s", client_v, server_v)
    if client_v.split("-")[0] != server_v.split("-")[0]:
        log.warning("Client and server versions differ")
    log.info("Map %s with %d spawn points", info["map"], info["spawn_points"])
    if info["synchronous_mode"]:
        log.warning("World is still in synchronous mode (a previous script crashed?)")
    return info


def drive_cars(cfg, seconds=40.0):
    """
    Spawn the scenario's vehicles with autopilot and let them drive for
    `seconds` (paced to real time so you can watch). No sensors, nothing saved.
    """
    from .backend.simulator.session import SimulationSession

    dt = cfg.simulation.fixed_delta_seconds
    with SimulationSession(cfg) as session:
        session.spawn_fleet()
        session.start_routes()
        for i in range(int(seconds / dt)):
            start = time.perf_counter()
            session.tick()
            session.update_camera()
            if i % int(5.0 / dt) == 0:
                log.info("Driving: %.0f / %.0f s", i * dt, seconds)
            time.sleep(max(0.0, dt - (time.perf_counter() - start)))


def record_lidar(cfg, name="run"):
    """
    Spawn CAVs + traffic, mount a LiDAR on every CAV, record full sweeps with
    ground truth, and return the run folder (data/runs/<date>_<time>_<name>).
    """
    from .backend.recording import recorder, storage
    from .backend.simulator.session import SimulationSession

    run_dir = storage.create_run_dir(cfg.recording.output_dir, name)
    _config.save_config(cfg, os.path.join(run_dir, CONFIG_FILE))
    log.info("Recording into %s", run_dir)
    with SimulationSession(cfg) as session:
        session.spawn_fleet()
        session.attach_lidars()
        summary = recorder.record(session, run_dir)
    log.info("Done: %d frames, %.0f points per CAV scan on average",
             len(summary.frames), summary.mean_points_per_cav)
    if cfg.recording.preview and summary.frames:
        show_frame(run_dir, summary.frames[-1])
    return run_dir


def open_editor(cfg, town=None, plan=None):
    """
    Open the scenario editor window: place CAVs and traffic by hand on any
    CARLA map, then export an experiment and record it. Works without CARLA
    running (maps are read from CARLA's OpenDRIVE files).
    """
    from .editor import app

    app.run(cfg, town, plan)


def import_openscenario(cfg, xosc_path, name, extra_traffic=0, autopilot=True, num_frames=10, save_every=20):
    """
    Turn an OpenSCENARIO file (e.g. made with CARLA's QGIS Traffic Generation
    Editor) into a spawn plan + experiment YAML ready for record_lidar.
    Ego vehicles become CAVs, other vehicles traffic. Returns
    (plan_path, yaml_path, warnings); warnings list what could not be imported.
    """
    from .backend.interop.openscenario import read_xosc
    from .editor import export, settings
    from .editor.openscenario_bridge import plan_from_xosc

    from .core.errors import ConfigError

    town = read_xosc(xosc_path).town
    road_map = settings.road_map(cfg, town)
    if road_map is None:
        raise ConfigError("No OpenDRIVE file for map '{}': set server.carla_root to the CARLA folder".format(town))
    plan, warnings = plan_from_xosc(xosc_path, road_map)
    options = export.ExportOptions(name, extra_traffic, autopilot, num_frames, save_every)
    try:
        plan_path, yaml_path = export.export(plan, options)
    except ValueError as exc:  # no ego vehicle, bad name
        raise ConfigError("{} ({})".format(exc, "; ".join(warnings) or "no warnings"))
    for w in warnings:
        log.warning(w)
    log.info("%s -> %s (%d CAVs, %d traffic) and %s", xosc_path, plan_path, *plan.counts().values(), yaml_path)
    return plan_path, yaml_path, warnings


def export_openscenario(cfg, plan_path, xosc_path, duration=60.0):
    """Write a spawn plan as an OpenSCENARIO file (QGIS editor, ScenarioRunner)."""
    from .editor.model import ScenarioPlan
    from .editor.openscenario_bridge import xosc_from_plan

    plan = ScenarioPlan.load(plan_path)
    xosc_from_plan(plan, xosc_path, cfg.scenario.cav_blueprint, cfg.scenario.traffic_blueprint, duration)
    log.info("%s -> %s (%d vehicles)", plan_path, xosc_path, len(plan.vehicles))
    return xosc_path


def show_frame(run_dir, frame=None, view=ViewMode.BOTH):
    """
    Save BEV plots of one recorded frame (default: the last one) into the run
    folder and return their paths. Works without CARLA running.
    """
    from .backend.recording import storage
    from .backend.visualization import bev as viz

    view = ViewMode.parse(view)
    frames = storage.list_frames(run_dir)
    if not frames:
        raise FileNotFoundError("No frames in {}".format(run_dir))
    frame = frames[-1] if frame is None else frame
    meta, clouds = storage.load_frame(run_dir, frame)
    paths = []
    if view in (ViewMode.SENSOR, ViewMode.BOTH):
        paths.append(viz.plot_sensor_frame(meta, clouds, os.path.join(run_dir, PREVIEW_SENSOR)))
    if view in (ViewMode.WORLD, ViewMode.BOTH):
        paths.append(viz.plot_world_frame(meta, clouds, os.path.join(run_dir, PREVIEW_WORLD)))
    for p in paths:
        log.info("Plot saved: %s", p)
    return paths
