"""
Implementation behind cavlidar.api, grouped by responsibility:

    simulator/      connection, synchronous mode, camera, SimulationSession   (needs carla)
    actors/         vehicles, LiDAR sensor, ground truth                       (needs carla)
    recording/      recording loop (needs carla) and storage (numpy only)
    geometry/       poses and transforms                                       (numpy only)
    visualization/  BEV plots                                                  (numpy + matplotlib)
"""
