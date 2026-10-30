"""The recording loop: tick, read every LiDAR, save clouds + metadata."""
import logging
import os
from dataclasses import dataclass, field
from typing import Dict, List

from ...core.constants import META_FILE, SPAWN_FILE, STATIONARY_KMH
from ...core.enums import Role
from ..actors import ground_truth
from . import storage

log = logging.getLogger("cavlidar")


@dataclass
class RecordSummary:
    run_dir: str
    frames: List[int] = field(default_factory=list)
    mean_points_per_cav: float = 0.0
    mean_cav_speed_kmh: Dict[str, float] = field(default_factory=dict)


def record(session, run_dir):
    """
    Save the initial vehicle poses (spawn.json), run warm-up ticks, then save
    recording.num_frames frames, one every recording.save_every ticks.
    Returns a RecordSummary and warns if every CAV stood still.
    """
    rec = session.cfg.recording
    storage.save_json(os.path.join(run_dir, SPAWN_FILE),
                      ground_truth.collect_vehicles(session.world, session.role_names()))
    for _ in range(rec.warmup_ticks):
        frame = session.tick()
        session.read_scans(frame)  # keep sensor queues in step
        session.update_camera()

    summary = RecordSummary(run_dir)
    total_points, tick = 0, 0
    speeds = {}
    while len(summary.frames) < rec.num_frames:
        frame = session.tick()
        scans = session.read_scans(frame)
        session.update_camera()
        tick += 1
        if (tick - 1) % rec.save_every:
            continue
        meta = _write_frame(session, run_dir, frame, scans)
        for vehicle in meta["vehicles"]:
            if vehicle["role"] == Role.CAV.value:
                speeds.setdefault(vehicle["name"], []).append(vehicle["speed_mps"] * 3.6)
        summary.frames.append(frame)
        total_points += sum(len(s.points) for s in scans)
        log.info("Saved frame %d (%d/%d): %s", frame, len(summary.frames), rec.num_frames,
                 ", ".join("{}={}".format(s.name, len(s.points)) for s in scans))
    if scans:
        summary.mean_points_per_cav = total_points / float(len(summary.frames) * len(scans))
    summary.mean_cav_speed_kmh = {name: sum(v) / len(v) for name, v in speeds.items()}
    _check_motion(summary.mean_cav_speed_kmh)
    return summary


def _check_motion(mean_speed_kmh):
    """Log CAV speeds; warn when none of them moved (e.g. warm-up too short)."""
    if not mean_speed_kmh:
        return
    log.info("Mean CAV speed (km/h): %s", ", ".join(
        "{}={:.1f}".format(n, s) for n, s in sorted(mean_speed_kmh.items())))
    if all(s < STATIONARY_KMH for s in mean_speed_kmh.values()):
        log.warning("Every CAV stayed below %.0f km/h during the recording: the scene is static. "
                    "Increase recording.warmup_ticks or record longer.", STATIONARY_KMH)


def _write_frame(session, run_dir, frame, scans):
    cfg = session.cfg
    directory = storage.frame_dir(run_dir, frame)
    for scan in scans:
        storage.save_points(directory, scan.name, scan.points, cfg.recording.formats)
    meta = {
        "frame": frame,
        "timestamp": scans[0].timestamp if scans else None,
        "town": cfg.server.town,
        "cavs": {
            scan.name: {
                "vehicle_id": lidar.vehicle.id,
                "sensor_pose": scan.sensor_pose,
                "mount_z": cfg.lidar.mount_z,
                "num_points": int(len(scan.points)),
            } for scan, lidar in zip(scans, session.lidars)
        },
        "vehicles": (ground_truth.collect_vehicles(session.world, session.role_names())
                     if cfg.recording.save_ground_truth else []),
    }
    storage.save_json(os.path.join(directory, META_FILE), meta)
    return meta
