"""The editor's live simulation (start, pause, record, stop) with a fake CARLA session."""
import os
import tempfile
import time
import unittest

from cavlidar.api import DEFAULT_CONFIG
from cavlidar.core.config import load_config
from cavlidar.editor import simulation


class FakeWorld:
    def get_actors(self):
        return self

    def filter(self, _pattern):
        return []


class FakeSession:
    """Ticks a frame counter; fails on demand."""
    instances = []

    def __init__(self, cfg, fail_at=None):
        self.cfg, self.fail_at = cfg, fail_at
        self.world = FakeWorld()
        self.cavs, self.traffic, self.lidars = [], [], []
        self.frame, self.exited = 100, False
        FakeSession.instances.append(self)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.exited = True
        return False

    def spawn_fleet(self):
        pass

    def attach_lidars(self):
        pass

    def start_routes(self):
        pass

    def update_camera(self):
        pass

    def role_names(self):
        return {}

    def tick(self):
        self.frame += 1
        if self.fail_at is not None and self.frame >= self.fail_at:
            raise RuntimeError("server gone")
        return self.frame

    def read_scans(self, frame):
        return []


def wait_for(predicate, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class TestLiveSimulation(unittest.TestCase):

    def setUp(self):
        self.cfg = load_config(DEFAULT_CONFIG, overrides=["simulation.fixed_delta_seconds=0.01",
                                                          "recording.save_every=2"])
        self.tmp = tempfile.TemporaryDirectory()
        self.written = []

    def tearDown(self):
        self.tmp.cleanup()

    def make(self, max_frames=0, **session_kw):
        return simulation.LiveSimulation(
            self.cfg, self.tmp.name, max_frames=max_frames,
            session_factory=lambda cfg: FakeSession(cfg, **session_kw),
            write_frame=lambda session, run_dir, frame, scans: self.written.append((run_dir, frame)),
            states=lambda session, statics: [])

    def test_start_pause_record_stop(self):
        sim = self.make()
        sim.start()
        self.assertTrue(wait_for(lambda: sim.view().state == simulation.RUNNING and sim.view().ticks > 5))
        v = sim.view()
        self.assertAlmostEqual(v.sim_time, v.ticks * 0.01, places=9)
        self.assertGreater(v.realtime, 0.0)
        self.assertLessEqual(v.realtime, 1.2)                      # never much faster than real time

        sim.pause()
        self.assertTrue(wait_for(lambda: sim.view().state == simulation.PAUSED))
        frozen = sim.view().frame
        time.sleep(0.1)
        self.assertEqual(sim.view().frame, frozen)                  # no tick while paused
        sim.resume()
        self.assertTrue(wait_for(lambda: sim.view().frame > frozen + 3))

        sim.start_recording("live test")
        self.assertTrue(wait_for(lambda: sim.view().frames_saved >= 3))
        run_dir = sim.view().recording
        self.assertTrue(os.path.basename(run_dir).endswith("_live test"))
        self.assertTrue(os.path.isfile(os.path.join(run_dir, "config.yaml")))
        self.assertTrue(os.path.isfile(os.path.join(run_dir, "spawn.json")))
        sim.stop_recording()
        self.assertTrue(wait_for(lambda: sim.view().recording == ""))
        finished = sim.pop_finished()
        self.assertEqual(len(finished), 1)
        self.assertEqual(finished[0][0], run_dir)
        saved = [frame for d, frame in self.written if d == run_dir]
        self.assertEqual(finished[0][1], len(saved))
        self.assertTrue(all(b - a == 2 for a, b in zip(saved, saved[1:])))   # one frame every save_every ticks
        self.assertEqual(sim.pop_finished(), [])

        sim.stop()
        sim.join(5)
        self.assertEqual(sim.view().state, simulation.STOPPED)
        self.assertTrue(FakeSession.instances[-1].exited)           # actors cleaned up

    # Each test waits only for states that last: a recording limited to a few frames ends ~80 ms after it starts
    # here, so "still recording with frames saved" is too short to catch on a busy machine.
    def test_recording_ends_after_max_frames(self):
        sim = self.make(max_frames=3)
        sim.start()
        self.assertTrue(wait_for(lambda: sim.view().state == simulation.RUNNING))
        sim.start_recording("short")
        self.assertTrue(wait_for(lambda: len(self.written) == 3 and sim.view().recording == ""))
        self.assertEqual(sim.pop_finished()[0][1], 3)
        self.assertEqual(sim.view().state, simulation.RUNNING)       # the simulation goes on without recording
        sim.stop()
        sim.join(5)
        self.assertEqual(len(self.written), 3)

    def test_recording_is_reported_when_the_simulation_stops(self):
        sim = self.make()                                           # no frame limit: still recording at the stop
        sim.start()
        self.assertTrue(wait_for(lambda: sim.view().state == simulation.RUNNING))
        sim.start_recording("cut")
        self.assertTrue(wait_for(lambda: sim.view().recording.endswith("_cut") and sim.view().frames_saved >= 1))
        sim.stop()
        sim.join(5)
        finished = sim.pop_finished()
        self.assertEqual(len(finished), 1)                          # the cut recording is reported too
        self.assertTrue(finished[0][0].endswith("_cut"))
        self.assertGreaterEqual(finished[0][1], 1)

    def test_failure_is_reported_and_cleaned_up(self):
        sim = self.make(fail_at=110)
        sim.start()
        sim.join(5)
        view = sim.view()
        self.assertEqual(view.state, simulation.FAILED)
        self.assertIn("server gone", view.error)
        self.assertTrue(FakeSession.instances[-1].exited)


class FakeCanvas:
    """Records what the editor draws; 10 pixels per metre, origin at (0, 0)."""
    scale = 10.0

    def __init__(self):
        self.items = []

    def world_to_screen(self, x, y):
        return x * self.scale, y * self.scale

    def create_polygon(self, *coords, **kw):
        self.items.append(("polygon", coords, kw))

    def create_line(self, *coords, **kw):
        self.items.append(("line", coords, kw))

    def create_text(self, *coords, **kw):
        self.items.append(("text", coords, kw))


class TestLiveDrawing(unittest.TestCase):

    def test_vehicles_drawn_with_their_real_box_colour_and_speed(self):
        from cavlidar.editor.app import COLORS, EditorApp
        from cavlidar.core.enums import Role
        pose = {"x": 10.0, "y": 20.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 90.0}
        live = simulation.LiveView(state=simulation.RUNNING, vehicles=[
            simulation.LiveVehicle("cav_0", "cav", pose, (2.4, 1.1, 0.7), (0.0, 0.0, 0.7), 27.3),
            simulation.LiveVehicle("traffic_0", "traffic", dict(pose, x=30.0), (2.0, 1.0, 0.7), (0.0, 0.0, 0.7), 0.0)])
        canvas = FakeCanvas()
        EditorApp._draw_live(None, canvas, live)
        polygons = [i for i in canvas.items if i[0] == "polygon"]
        self.assertEqual([p[2]["fill"] for p in polygons], [COLORS[Role.CAV], COLORS[Role.TRAFFIC]])
        xs, ys = polygons[0][1][0::2], polygons[0][1][1::2]
        # heading 90 deg: the 4.8 m long box lies along y, 2.2 m wide along x
        self.assertAlmostEqual(max(ys) - min(ys), 48.0, places=6)
        self.assertAlmostEqual(max(xs) - min(xs), 22.0, places=6)
        texts = [i[2]["text"] for i in canvas.items if i[0] == "text"]
        self.assertIn("cav_0  27 km/h", texts)
        self.assertIn("traffic_0  0 km/h", texts)


if __name__ == "__main__":
    unittest.main()
