"""Synchronous-mode context manager."""
import logging

log = logging.getLogger("cavlidar")


class SyncMode:
    """
    Run the simulation in synchronous mode inside a `with` block.

    In synchronous mode the server only advances when the client calls tick(),
    by exactly `fixed_delta_seconds`. With fixed_delta_seconds equal to
    1 / lidar rotation_frequency, every tick yields one full 360-degree LiDAR
    sweep. The previous world settings are restored when the block exits.

        with SyncMode(client, world, 0.05) as sync:
            frame = sync.tick()
    """

    def __init__(self, client, world, fixed_delta_seconds, no_rendering=False, tm_port=8000):
        self.client = client
        self.world = world
        self.fixed_delta_seconds = fixed_delta_seconds
        self.no_rendering = no_rendering
        self.tm_port = tm_port
        self.tm = None
        self._original = None

    def __enter__(self):
        self._original = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.fixed_delta_seconds
        settings.no_rendering_mode = self.no_rendering
        self.world.apply_settings(settings)
        self.tm = self.client.get_trafficmanager(self.tm_port)
        self.tm.set_synchronous_mode(True)
        log.info("Synchronous mode on: %.3f s per tick", self.fixed_delta_seconds)
        return self

    def tick(self):
        """Advance the simulation by one step and return the new frame number."""
        return self.world.tick()

    def __exit__(self, exc_type, exc, tb):
        self.tm.set_synchronous_mode(False)
        self.world.apply_settings(self._original)
        log.info("Original (asynchronous) world settings restored")
        return False
