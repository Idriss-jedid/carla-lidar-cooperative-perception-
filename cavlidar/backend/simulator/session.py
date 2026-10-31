"""
SimulationSession: one object that owns everything created in CARLA.

    with SimulationSession(cfg) as session:
        session.spawn_fleet()
        session.attach_lidars()
        session.start_routes()
        frame = session.tick()

On exit (normal end, error or Ctrl+C) the sensors and vehicles are destroyed
and the world goes back to its original asynchronous settings, so CARLA is
always left clean for the next script.
"""
import logging
import random

from ...core.constants import CAV_CLOUD_NAME
from ...core.enums import CameraMode, Role
from ...core.errors import RouteError, SpawnError
from ..actors import route_driver, spawn_plan, vehicles
from ..maps import routes
from . import camera, connection
from ..actors.lidar import Lidar
from .sync import SyncMode

log = logging.getLogger("cavlidar")


class SimulationSession:

    def __init__(self, cfg):
        self.cfg = cfg
        self.rng = random.Random(cfg.simulation.seed)
        self.client = None
        self.world = None
        self.cavs = []
        self.traffic = []
        self.lidars = []
        self.drivers = []   # RouteDriver of every vehicle with a planned route
        self._sync = None

    # ------------------------------------------------------------------ lifecycle
    def __enter__(self):
        srv, sim = self.cfg.server, self.cfg.simulation
        self.client = connection.connect(srv.host, srv.port, srv.timeout)
        self.world = connection.ensure_town(self.client, srv.town, srv.allow_map_change)
        self._sync = SyncMode(self.client, self.world, sim.fixed_delta_seconds,
                              sim.no_rendering, srv.tm_port).__enter__()
        try:
            self.tm.set_random_device_seed(sim.seed)
        except AttributeError:
            pass
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            for lidar in self.lidars:
                lidar.destroy()
            vehicles.destroy(self.client, self.cavs + self.traffic)
            log.info("Destroyed %d sensors and %d vehicles",
                     len(self.lidars), len(self.cavs) + len(self.traffic))
        finally:
            self._sync.__exit__(exc_type, exc, tb)
        return False

    @property
    def tm(self):
        return self._sync.tm

    def tick(self):
        """Steer the routed vehicles, advance the simulation by fixed_delta_seconds; return the frame number."""
        route_driver.step_all(self.drivers, self.cavs + self.traffic, self.cfg.simulation.fixed_delta_seconds)
        return self._sync.tick()

    # ------------------------------------------------------------------ actors
    def spawn_fleet(self):
        """
        Spawn scenario.num_cavs CAVs and scenario.num_traffic other vehicles:
        first the ones fixed by scenario.spawn_plan (if any), then the rest at
        random spawn points.

        Who drives: a planned vehicle with a route follows it (RouteDriver,
        started by start_routes()); one with only a speed drives with the
        Traffic Manager at that speed; the others drive with the Traffic
        Manager if scenario.autopilot, else stay parked.
        """
        sc = self.cfg.scenario
        planned = {Role.CAV: [], Role.TRAFFIC: []}
        if sc.spawn_plan:
            planned = spawn_plan.load_plan(sc.spawn_plan, self.cfg.server.town,
                                           sc.num_cavs, sc.num_traffic, sc.spawn_bbox)
        traced = self._trace_routes(planned)  # before spawning: a bad route stops here
        self.cavs = vehicles.spawn_at_poses(
            self.world, [p.pose for p in planned[Role.CAV]], sc.cav_blueprint, self.rng, Role.CAV)
        self.traffic = vehicles.spawn_at_poses(
            self.world, [p.pose for p in planned[Role.TRAFFIC]], sc.traffic_blueprint, self.rng, Role.TRAFFIC)
        if len(self.cavs) < len(planned[Role.CAV]) or len(self.traffic) < len(planned[Role.TRAFFIC]):
            # Filling the gap at random would silently change the planned scene
            raise SpawnError("Only {}/{} planned CAVs and {}/{} planned traffic vehicles could be "
                             "spawned from {}".format(len(self.cavs), len(planned[Role.CAV]),
                                                     len(self.traffic), len(planned[Role.TRAFFIC]),
                                                     sc.spawn_plan))

        candidates = vehicles.candidate_spawn_points(
            self.world, self.rng, sc.spawn_center, sc.spawn_radius, sc.spawn_bbox)
        self.cavs += vehicles.spawn_vehicles(
            self.world, sc.num_cavs - len(self.cavs), sc.cav_blueprint, candidates, self.rng,
            Role.CAV, first_index=len(self.cavs))
        self.traffic += vehicles.spawn_vehicles(
            self.world, sc.num_traffic - len(self.traffic), sc.traffic_blueprint, candidates, self.rng,
            Role.TRAFFIC, first_index=len(self.traffic))
        if len(self.cavs) < sc.num_cavs or len(self.traffic) < sc.num_traffic:
            # A scenario with fewer vehicles than asked is not the scenario of the config
            raise SpawnError(
                "Only {}/{} CAVs and {}/{} traffic vehicles could be spawned. Lower "
                "scenario.num_traffic or enlarge scenario.spawn_bbox.".format(
                    len(self.cavs), sc.num_cavs, len(self.traffic), sc.num_traffic))
        # planned entries by actor id (the random vehicles have none); planned
        # vehicles are the first ones of each list, in plan order
        plans = {v.id: p for v, p in zip(self.cavs, planned[Role.CAV])}
        plans.update({v.id: p for v, p in zip(self.traffic, planned[Role.TRAFFIC])})
        paths = {(self.cavs if role == Role.CAV else self.traffic)[i].id: route
                 for (role, i), route in traced.items()}
        names = self.role_names()
        everyone = self.cavs + self.traffic
        with_speed = [v for v in everyone if v.id in plans and plans[v.id].speed_kmh and v.id not in paths]
        speed_ids = {v.id for v in with_speed}
        autopilot = [v for v in everyone if v.id not in paths and (sc.autopilot or v.id in speed_ids)]
        vehicles.enable_autopilot(autopilot, self.tm, sc.speed_difference_range, sc.ignore_lights_range, self.rng)
        self.tick()  # make the new actors exist on the server (and know their speed limit)
        for v in with_speed:
            self.tm.vehicle_percentage_speed_difference(v, vehicles.tm_speed_difference(v, plans[v.id].speed_kmh))
        self.drivers = [
            route_driver.RouteDriver(v, names[v.id], paths[v.id], plans[v.id].speed_kmh, plans[v.id].route_end,
                                     route_driver.route_lights(self.world, paths[v.id]), self.tm,
                                     vehicles.tm_speed_difference)
            for v in everyone if v.id in paths]
        log.info("Spawned %d CAVs and %d traffic vehicles (%d on a route, %d with the Traffic Manager, "
                 "%d parked)", len(self.cavs), len(self.traffic), len(self.drivers), len(autopilot),
                 len(everyone) - len(self.drivers) - len(autopilot))

    def _trace_routes(self, planned):
        """{(role, index in the plan): Route} for the planned vehicles that have a route."""
        traced = {}
        carla_map = None
        for role in (Role.CAV, Role.TRAFFIC):
            for i, p in enumerate(planned[role]):
                if not p.route:
                    continue
                carla_map = carla_map or self.world.get_map()
                try:
                    traced[(role, i)] = routes.trace_route(carla_map, p.pose, p.route)
                except RouteError as exc:
                    raise RouteError("{}: {}".format(role.name_of(i), exc))
        return traced

    def start_routes(self):
        """Release the routed vehicles (they hold their planned pose until now) at their planned speed."""
        for driver in self.drivers:
            driver.start()
        if self.drivers:
            log.info("Started %d routed vehicles: %s", len(self.drivers), ", ".join(
                "{} {:.0f} m at {:.0f} km/h".format(d.name, d.route.length, d.speed * 3.6) for d in self.drivers))

    def attach_lidars(self):
        """Mount one LiDAR (config.lidar) on every CAV."""
        self.lidars = [Lidar(self.world, v, self.cfg.lidar, CAV_CLOUD_NAME.format(i))
                       for i, v in enumerate(self.cavs)]
        log.info("Attached %d LiDARs (%d channels, %.0f m)", len(self.lidars),
                 self.cfg.lidar.channels, self.cfg.lidar.range)

    def read_scans(self, frame):
        """The LidarScan of every CAV for `frame`."""
        return [lidar.get(frame) for lidar in self.lidars]

    def role_names(self):
        """{actor_id: 'cav_0' | 'traffic_3' ...} for the ground truth."""
        names = {v.id: Role.CAV.name_of(i) for i, v in enumerate(self.cavs)}
        names.update({v.id: Role.TRAFFIC.name_of(i) for i, v in enumerate(self.traffic)})
        return names

    # ------------------------------------------------------------------ camera
    def update_camera(self):
        rec = self.cfg.recording
        if rec.camera == CameraMode.FOLLOW and self.cavs:
            camera.follow(self.world, self.cavs[0], rec.camera_height)
        elif rec.camera == CameraMode.TOP:
            x, y = self.cfg.scenario.spawn_center
            z = self.cavs[0].get_location().z if self.cavs else 0.0
            camera.top_view(self.world, x, y, z, rec.camera_height)
