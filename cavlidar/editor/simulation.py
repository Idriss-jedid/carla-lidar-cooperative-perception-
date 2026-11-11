"""
A live simulation run from the editor: start, watch the vehicles drive in real time, pause, record, stop.

    sim = LiveSimulation(cfg, runs_dir)    # cfg = the experiment the editor exported (plan + settings)
    sim.start()                            # background thread: spawn, then tick at real-time pace
    sim.pause(); sim.resume()              # no tick while paused: CARLA (synchronous mode) waits, the scene freezes
    sim.start_recording("my_scene")        # mount the LiDARs, save one frame every recording.save_every ticks
    sim.stop_recording()                   # LiDARs removed; the run folder appears in sim.pop_finished()

The LiDARs (one per CAV, the experiment's settings) exist only while recording: they take most of the time of a tick,
so without them the vehicles can be watched at real-time speed.
    sim.stop()                             # vehicles and sensors removed, CARLA back to asynchronous mode

Only the simulation thread talks to CARLA. The window reads `sim.view()` (state, simulated time, speed compared with
real time, every vehicle's pose, recording) whenever it redraws. Frames are written exactly as `main.py record` writes
them (LiDAR of the tick + ground truth of the same tick), so a live recording is processed like any other run. It is
not repeatable, though: it starts whenever the button is pressed. Use `main.py record` for experiments.
"""
import collections
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field, replace
from typing import List

from ..core.constants import CONFIG_FILE, SPAWN_FILE

log = logging.getLogger("cavlidar")

STARTING, RUNNING, PAUSED, STOPPING, STOPPED, FAILED = "starting", "running", "paused", "stopping", "stopped", "failed"
ACTIVE = (STARTING, RUNNING, PAUSED, STOPPING)
PAUSE_POLL = 0.05      # seconds between checks while paused
LIDAR_WARMUP_TICKS = 2  # ticks after mounting the LiDARs before the first saved sweep
MAX_LAG = 0.25         # seconds behind the real-time schedule before the schedule is reset


@dataclass
class LiveVehicle:
    name: str
    role: str                 # "cav" or "traffic"
    pose: dict                # x, y, z, roll, pitch, yaw (world)
    extent: tuple             # box half sizes (m)
    center_offset: tuple      # box centre in the vehicle frame (m)
    speed_kmh: float


@dataclass
class LiveView:
    state: str = STARTING
    frame: int = 0
    ticks: int = 0
    sim_time: float = 0.0     # simulated seconds since the start
    realtime: float = 0.0     # simulated seconds per wall-clock second over the last ticks (1.0 = real time)
    vehicles: List[LiveVehicle] = field(default_factory=list)
    recording: str = ""       # run folder being written, "" when not recording
    frames_saved: int = 0
    error: str = ""


def _snapshot_states(session, statics):
    """LiveVehicle of every spawned vehicle from the world snapshot of the last tick (no extra server call)."""
    snapshot = session.world.get_snapshot()
    out = []
    for actor_id, (name, role, extent, offset) in statics.items():
        state = snapshot.find(actor_id)
        if state is None:
            continue
        t, v = state.get_transform(), state.get_velocity()
        pose = {"x": t.location.x, "y": t.location.y, "z": t.location.z,
                "roll": t.rotation.roll, "pitch": t.rotation.pitch, "yaw": t.rotation.yaw}
        out.append(LiveVehicle(name, role, pose, extent, offset, 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)))
    return out


def _statics(session):
    """{actor id: (name, role, box half sizes, box centre)} of the spawned vehicles (they do not change)."""
    names = session.role_names()
    out = {}
    for role, actors in (("cav", session.cavs), ("traffic", session.traffic)):
        for actor in actors:
            bb = actor.bounding_box
            out[actor.id] = (names[actor.id], role, (bb.extent.x, bb.extent.y, bb.extent.z),
                             (bb.location.x, bb.location.y, bb.location.z))
    return out


class LiveSimulation:

    def __init__(self, cfg, runs_dir, max_frames=0, session_factory=None, write_frame=None, states=None,
                 clock=time.perf_counter, sleep=time.sleep):
        """
        max_frames: a recording ends by itself after this many frames (0 = when stop_recording() is called).
        session_factory, write_frame, states: replaceable for tests (default: SimulationSession, the recorder's
        frame writer, the world snapshot).
        """
        self.cfg = cfg
        self.runs_dir = runs_dir
        self.max_frames = int(max_frames)
        self._session_factory = session_factory
        self._write_frame = write_frame
        self._states = states or _snapshot_states
        self._clock, self._sleep = clock, sleep
        self._lock = threading.Lock()
        self._view = LiveView()
        self._paused = False
        self._record_name = None       # requested by start_recording(), taken by the thread
        self._record_stop = False
        self._finished = []            # (run folder, frames saved) of the ended recordings
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ commands (any thread)
    def start(self):
        self._thread = threading.Thread(target=self._run, name="live-simulation", daemon=True)
        self._thread.start()

    def pause(self):
        with self._lock:
            self._paused = True

    def resume(self):
        with self._lock:
            self._paused = False

    def stop(self):
        self._stop.set()

    def start_recording(self, name):
        with self._lock:
            self._record_name, self._record_stop = name, False

    def stop_recording(self):
        with self._lock:
            self._record_stop = True

    def view(self):
        with self._lock:
            return replace(self._view)

    def pop_finished(self):
        with self._lock:
            out, self._finished = self._finished, []
        return out

    def join(self, timeout=None):
        if self._thread is not None:
            self._thread.join(timeout)

    # ------------------------------------------------------------------ simulation thread
    def _set(self, **changes):
        with self._lock:
            self._view = replace(self._view, **changes)

    def _run(self):
        recording = None
        try:
            factory = self._session_factory
            if factory is None:
                from ..backend.simulator.session import SimulationSession as factory
            dt = self.cfg.simulation.fixed_delta_seconds
            with factory(self.cfg) as session:
                session.spawn_fleet()           # no LiDAR yet: they are mounted only while recording
                statics = _statics(session)
                frame = session.tick()
                session.start_routes()
                ticks, starts = 0, collections.deque(maxlen=41)
                self._set(state=RUNNING, frame=frame, vehicles=self._states(session, statics))
                deadline = self._clock()
                while not self._stop.is_set():
                    recording = self._recording_requests(session, recording)
                    with self._lock:
                        paused = self._paused
                    if paused:
                        self._set(state=PAUSED, realtime=0.0)
                        starts.clear()
                        self._sleep(PAUSE_POLL)
                        deadline = self._clock()
                        continue
                    starts.append(self._clock())
                    frame = session.tick()
                    scans = session.read_scans(frame)     # [] without LiDARs; keeps their queues in step
                    session.update_camera()
                    ticks += 1
                    if recording is not None:
                        recording = self._save(session, recording, frame, scans)
                    # Not faster than real time. The pace follows a fixed schedule rather than sleeping
                    # dt - (time of this tick): Windows sleeps in ~15 ms steps, and a schedule makes up for an
                    # overslept tick on the next one. Far behind (LiDARs mounted), it does not try to catch up.
                    deadline += dt
                    delay = deadline - self._clock()
                    if delay > 0:
                        self._sleep(delay)
                    elif delay < -MAX_LAG:
                        deadline = self._clock()
                    realtime = dt * (len(starts) - 1) / (starts[-1] - starts[0]) if len(starts) > 1 else 0.0
                    self._set(state=RUNNING, frame=frame, ticks=ticks, sim_time=ticks * dt, realtime=realtime,
                              vehicles=self._states(session, statics))
                self._set(state=STOPPING)
                recording = self._end_recording(recording)
            self._set(state=STOPPED, realtime=0.0)
        except Exception as exc:  # shown by the window; frames saved so far are kept
            log.exception("Live simulation failed")
            self._end_recording(recording)
            self._set(state=FAILED, error="{}: {}".format(type(exc).__name__, exc), realtime=0.0)

    def _recording_requests(self, session, recording):
        with self._lock:
            name, stop = self._record_name, self._record_stop
            self._record_name, self._record_stop = None, False
        if stop and recording is not None:
            recording = self._end_recording(recording)
        if name and recording is None:
            recording = self._begin_recording(session, name)
        return recording

    def _begin_recording(self, session, name):
        from ..backend.actors import ground_truth
        from ..backend.recording import storage
        from ..core.config import save_config

        run_dir = storage.create_run_dir(self.runs_dir, name)
        save_config(self.cfg, os.path.join(run_dir, CONFIG_FILE))
        storage.save_json(os.path.join(run_dir, SPAWN_FILE),
                          ground_truth.collect_vehicles(session.world, session.role_names()))
        session.attach_lidars()   # LiDARs cost most of the tick time: mounted only while recording
        log.info("Live recording into %s", run_dir)
        self._set(recording=run_dir, frames_saved=0)
        # ticks starts at -WARMUP: the sweeps of the first ticks after mounting are not saved
        return {"dir": run_dir, "ticks": -LIDAR_WARMUP_TICKS, "saved": 0, "session": session}

    def _save(self, session, recording, frame, scans):
        if recording["ticks"] >= 0 and recording["ticks"] % self.cfg.recording.save_every == 0:
            write = self._write_frame
            if write is None:
                from ..backend.recording.recorder import _write_frame as write
            write(session, recording["dir"], frame, scans)
            recording["saved"] += 1
            self._set(frames_saved=recording["saved"])
        recording["ticks"] += 1
        if self.max_frames and recording["saved"] >= self.max_frames:
            return self._end_recording(recording)
        return recording

    def _end_recording(self, recording):
        if recording is None:
            return None
        session = recording["session"]
        try:
            for lidar in session.lidars:
                lidar.destroy()
        except Exception:  # CARLA already gone: SimulationSession has cleaned up or cannot
            log.warning("Could not remove the LiDARs after the recording", exc_info=True)
        session.lidars = []
        log.info("Live recording ended: %d frames in %s", recording["saved"], recording["dir"])
        with self._lock:
            self._finished.append((recording["dir"], recording["saved"]))
            self._view = replace(self._view, recording="")
        return None
