"""
Drive a vehicle along its planned route at its planned speed (our own controller).

CARLA 0.9.12's Traffic Manager cannot follow a given path or hold a given speed
(set_path / set_desired_speed came later), so routed vehicles are driven here,
once per tick, before the world advances:

- steering: pure pursuit on the route's path;
- speed: held exactly by CARLA's constant-velocity mode, set every tick to the
  route's speed profile (chosen speed, slower in sharp turns), lowered to stop
  for a red light on the route, behind a vehicle on the path ahead and at the
  last point (or the Traffic Manager takes over there, see RouteEnd).
  Speeding up is limited to ACCEL, braking follows DECEL.

Before start() (i.e. during the recorder's warm-up) the vehicle holds its pose
with the brakes on; start() gives it its initial speed at once, so the first
recorded frame shows the planned scene already moving.
"""
import logging
import math

import carla
import numpy as np

from ...core.enums import RouteEnd
from ..maps.routes import DECEL, speed_profile

log = logging.getLogger("cavlidar")

LOOKAHEAD_MIN, LOOKAHEAD_TIME = 4.0, 0.6   # pure pursuit look-ahead: max(min, time * speed) metres
WINDOW = 20                    # path points searched ahead of the last position
LANE_HALF_WIDTH = 1.8          # a vehicle closer than this to the path is in the way (m)
VEHICLE_GAP = 3.0              # metres kept to the vehicle ahead (bumper to bumper)
STOP_LINE_GAP = 1.0            # metres kept before a red light's stop line
LIGHT_MATCH = 3.0              # a light's stop waypoint closer than this to the path stops the route (m)
ACCEL = 2.5                    # m/s^2 when speeding up
EMERGENCY_DECEL = 8.0          # m/s^2 at most when something appears in front
STOPPED = 0.05                 # m/s: a commanded speed below this means standing still
TRIM_RATE, TRIM_LIMITS = 0.5, (0.95, 1.10)  # slow correction of the ~2 % CARLA keeps below the set speed


def route_lights(world, route):
    """[(distance along the route of the stop line, traffic light)] for the lights on the route."""
    s = route.distances
    found = []
    for light in world.get_actors().filter("traffic.traffic_light*"):
        best = None
        for wp in light.get_stop_waypoints():
            loc = wp.transform.location
            d = np.hypot(route.points[:, 0] - loc.x, route.points[:, 1] - loc.y)
            same_lane = np.array([lane == (wp.road_id, wp.lane_id) for lane in route.lanes])
            d[~same_lane | (np.abs(route.points[:, 2] - loc.z) > 3.0)] = np.inf
            i = int(np.argmin(d))
            if d[i] <= LIGHT_MATCH and (best is None or s[i] < best):
                best = float(s[i])
        if best is not None and best > 1.0:  # a light right behind the start does not apply
            found.append((best, light))
    return sorted(found, key=lambda item: item[0])


class RouteDriver:

    def __init__(self, vehicle, name, route, speed_kmh, end=RouteEnd.STOP, lights=(), tm=None, tm_speed=None):
        self.vehicle, self.name, self.route = vehicle, name, route
        self.end = RouteEnd.parse(end)
        self.speed = speed_kmh / 3.6
        self.s = route.distances
        self.profile = speed_profile(route, self.speed, stop_at_end=self.end == RouteEnd.STOP)
        self.lights = list(lights)
        self.tm, self.tm_speed = tm, tm_speed
        self.index = 0
        self.progress = 0.0     # metres driven along the route
        self.target = 0.0       # highest speed allowed in the last step (m/s)
        self.command = 0.0      # speed held in the last step (m/s)
        self.started = False
        self.finished = False
        self._cruising = False  # constant-velocity mode on
        self._trim = 1.0        # set speed / wanted speed
        self.half_length = vehicle.bounding_box.extent.x
        physics = vehicle.get_physics_control()
        self.max_steer = math.radians(max(w.max_steer_angle for w in physics.wheels)) or math.radians(70.0)
        self.wheelbase = 1.2 * self.half_length  # about 60 % of the length for passenger cars

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        """Release the brakes and give the vehicle its initial speed (lower if it must stop soon)."""
        self.started = True
        self.command = self._allowed(0.0, None)
        if self.command > STOPPED:
            forward = self.vehicle.get_transform().get_forward_vector()
            v = self.command
            self.vehicle.set_target_velocity(carla.Vector3D(forward.x * v, forward.y * v, forward.z * v))
            self._cruise(v, 0.0)

    def _cruise(self, speed, steer):
        speed *= self._trim
        self.vehicle.enable_constant_velocity(carla.Vector3D(speed, 0.0, 0.0))  # vehicle frame: forward
        self.vehicle.apply_control(carla.VehicleControl(steer=steer))
        self._cruising = True

    def _hold(self):
        if self._cruising:
            self.vehicle.disable_constant_velocity()
            self._cruising = False
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))

    def _hand_over(self):
        """End of an AUTOPILOT route: the Traffic Manager drives on at the same speed."""
        self.finished = True
        if self._cruising:
            self.vehicle.disable_constant_velocity()
            self._cruising = False
        self.vehicle.set_autopilot(True, self.tm.get_port())
        if self.tm_speed is not None:
            self.tm.vehicle_percentage_speed_difference(self.vehicle, self.tm_speed(self.vehicle, self.speed * 3.6))
        log.info("%s reached the end of its route: the Traffic Manager takes over", self.name)

    # ------------------------------------------------------------------ control
    def _progress(self, x, y):
        """Distance along the path of the vehicle (never goes back, so loops are followed in order)."""
        lo, hi = self.index, min(len(self.s), self.index + WINDOW)
        d = np.hypot(self.route.points[lo:hi, 0] - x, self.route.points[lo:hi, 1] - y)
        self.index = lo + int(np.argmin(d))
        i = self.index
        if i + 1 < len(self.s):  # project onto the segment i -> i+1
            seg = self.route.points[i + 1, :2] - self.route.points[i, :2]
            t = float(np.dot([x - self.route.points[i, 0], y - self.route.points[i, 1]], seg) / max(seg @ seg, 1e-9))
            return self.s[i] + max(0.0, min(1.0, t)) * (self.s[i + 1] - self.s[i])
        return self.s[i]

    def _point_at(self, s):
        """(x, y) on the path `s` metres from the start; beyond the end, straight on."""
        if s <= self.s[-1] or len(self.s) < 2:
            return np.interp(s, self.s, self.route.points[:, 0]), np.interp(s, self.s, self.route.points[:, 1])
        last = self.route.points[-1, :2] - self.route.points[-2, :2]
        last = last / max(np.linalg.norm(last), 1e-9) * (s - self.s[-1])
        return self.route.points[-1, 0] + last[0], self.route.points[-1, 1] + last[1]

    def _allowed(self, s_now, others):
        """Highest speed (m/s) at s_now: profile, red light ahead, vehicle ahead."""
        allowed = float(np.interp(s_now, self.s, self.profile))
        for s_stop, light in self.lights:
            gap = s_stop - s_now - self.half_length - STOP_LINE_GAP
            if gap < -STOP_LINE_GAP - 1.0:
                continue  # already past this stop line
            state = light.get_state()
            braking = self.command ** 2 / (2.0 * DECEL)
            if state == carla.TrafficLightState.Red or (state == carla.TrafficLightState.Yellow and gap > braking):
                allowed = min(allowed, math.sqrt(2.0 * DECEL * max(0.0, gap)))
            break  # only the next light on the route matters
        if others is not None and len(others):
            ahead = self.command ** 2 / (2.0 * DECEL) + 15.0
            lo = self.index
            hi = min(len(self.s), int(np.searchsorted(self.s, s_now + ahead)) + 1)
            if hi > lo:
                pts = self.route.points[lo:hi]
                d = np.hypot(pts[:, None, 0] - others[None, :, 0], pts[:, None, 1] - others[None, :, 1])
                d[np.abs(pts[:, None, 2] - others[None, :, 2]) > 3.0] = np.inf
                hits = np.where(d.min(axis=1) < LANE_HALF_WIDTH)[0]
                if len(hits):  # centre to centre minus two half lengths = bumper to bumper
                    gap = self.s[lo + hits[0]] - s_now - 2.0 * self.half_length - VEHICLE_GAP
                    allowed = min(allowed, math.sqrt(2.0 * DECEL * max(0.0, gap)))
        return allowed

    def _speed(self):
        v = self.vehicle.get_velocity()
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def step(self, others=None, dt=0.05):
        """Apply this tick's control. `others`: (m, 3) positions of the other vehicles."""
        if self.finished and self.end == RouteEnd.AUTOPILOT:
            return
        if not self.started or self.finished:
            self._hold()
            return
        tf = self.vehicle.get_transform()
        x, y = tf.location.x, tf.location.y
        s_now = self.progress = self._progress(x, y)
        remaining = self.s[-1] - s_now
        if self.end == RouteEnd.AUTOPILOT and remaining < LOOKAHEAD_MIN:
            self._hand_over()
            return

        self.target = self._allowed(s_now, others)
        if abs(self.target - self.command) < 0.1 and self.command > 1.0:  # cruising: correct the set speed
            error = (self.command - self._speed()) / self.command
            self._trim = min(TRIM_LIMITS[1], max(TRIM_LIMITS[0], self._trim + TRIM_RATE * error * dt))
        if self.target > self.command:
            self.command = min(self.target, self.command + ACCEL * dt)
        else:
            self.command = max(self.target, self.command - EMERGENCY_DECEL * dt)
        if self.command < STOPPED:
            self.command = 0.0
            if self.end == RouteEnd.STOP and remaining < 1.0:
                self.finished = True
                log.info("%s stopped at the end of its route (%.1f m before the last point)", self.name, remaining)
            self._hold()  # end of route, red light or vehicle ahead
            return

        # pure pursuit
        lookahead = max(LOOKAHEAD_MIN, LOOKAHEAD_TIME * self.command)
        tx, ty = self._point_at(s_now + lookahead)
        yaw = math.radians(tf.rotation.yaw)
        dx, dy = tx - x, ty - y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy   # CARLA: +y is to the right
        alpha = math.atan2(local_y, local_x)
        angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), max(math.hypot(dx, dy), 1e-3))
        self._cruise(self.command, max(-1.0, min(1.0, angle / self.max_steer)))


def positions(actors):
    """(n, 3) array of the actors' locations."""
    locs = [a.get_location() for a in actors]
    return np.array([[l.x, l.y, l.z] for l in locs]) if locs else np.zeros((0, 3))


def step_all(drivers, actors, dt):
    """One control step of every route driver; each avoids all the other actors."""
    if not drivers:
        return
    pos = positions(actors)
    ids = np.array([a.id for a in actors])
    for driver in drivers:
        driver.step(pos[ids != driver.vehicle.id] if len(pos) else None, dt)
