"""The recording loop: tick, read every LiDAR, save clouds + metadata."""
import logging
import os
from dataclasses import dataclass, field
from typing import List

from ...core.constants import META_FILE
from ..actors import ground_truth
from . import storage

log = logging.getLogger("cavlidar")


@dataclass
class RecordSummary:
    run_dir: str
    frames: List[int] = field(default_factory=list)
    mean_points_per_cav: float = 0.0


def record(session, run_dir):
    """
    Run warm-up ticks, then save recording.num_frames frames, one every
    recording.save_every ticks. Returns a RecordSummary.
    """
    rec = session.cfg.recording
    for _ in range(rec.warmup_ticks):
        frame = session.tick()
        session.read_scans(frame)  # keep sensor queues in step
        session.update_camera()

    summary = RecordSummary(run_dir)
    total_points, tick = 0, 0
    while len(summary.frames) < rec.num_frames:
        frame = session.tick()
        scans = session.read_scans(frame)
        session.update_camera()
        tick += 1
        if (tick - 1) % rec.save_every:
            continue
        _write_frame(session, run_dir, frame, scans)
        summary.frames.append(frame)
        total_points += sum(len(s.points) for s in scans)
        log.info("Saved frame %d (%d/%d): %s", frame, len(summary.frames), rec.num_frames,
                 ", ".join("{}={}".format(s.name, len(s.points)) for s in scans))
    if scans:
        summary.mean_points_per_cav = total_points / float(len(summary.frames) * len(scans))
    return summary


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
