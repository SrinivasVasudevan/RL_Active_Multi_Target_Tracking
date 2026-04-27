from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import torch

from .real_world_types import RobotReport, TargetDetection


@dataclass
class ReportSnapshot:
    report: RobotReport
    received_sec: float


@dataclass
class TrackObservation:
    robot_name: str
    label: str
    confidence: float
    world_x: float
    world_y: float
    stamp_sec: float
    detection: TargetDetection


@dataclass
class Track:
    track_uid: int
    slot_id: int
    label: str
    x: float
    y: float
    vx: float
    vy: float
    last_update_sec: float
    hits: int = 1
    score: float = 1.0
    cycle_measurements: Dict[str, TrackObservation] = field(default_factory=dict)

    def predicted_position(self, stamp_sec: float) -> Tuple[float, float]:
        dt = max(0.0, stamp_sec - self.last_update_sec)
        return (self.x + self.vx * dt, self.y + self.vy * dt)

    def absorb(self, obs: TrackObservation, alpha: float, beta: float):
        dt = max(1e-3, obs.stamp_sec - self.last_update_sec)

        if self.hits <= 0:
            self.x = obs.world_x
            self.y = obs.world_y
            self.vx = 0.0
            self.vy = 0.0
        else:
            pred_x, pred_y = self.predicted_position(obs.stamp_sec)
            residual_x = obs.world_x - pred_x
            residual_y = obs.world_y - pred_y
            self.x = pred_x + alpha * residual_x
            self.y = pred_y + alpha * residual_y
            self.vx = self.vx + beta * residual_x / dt
            self.vy = self.vy + beta * residual_y / dt

        self.last_update_sec = obs.stamp_sec
        self.hits += 1
        self.score = min(10.0, self.score + max(0.05, obs.confidence))
        self.cycle_measurements[obs.robot_name] = obs


class TrackManager:
    def __init__(
        self,
        max_tracks: int,
        association_distance_m: float,
        track_timeout_sec: float,
        position_alpha: float = 0.7,
        velocity_beta: float = 0.2,
    ):
        self.max_tracks = max(1, int(max_tracks))
        self.association_distance_m = max(0.1, float(association_distance_m))
        self.track_timeout_sec = max(0.1, float(track_timeout_sec))
        self.position_alpha = float(position_alpha)
        self.velocity_beta = float(velocity_beta)
        self._tracks: Dict[int, Track] = {}
        self._next_track_uid = 0

    def update(self, snapshots: Sequence[ReportSnapshot], now_sec: float):
        observations: List[TrackObservation] = []
        for snapshot in snapshots:
            report = snapshot.report
            for det in report.detections:
                observations.append(
                    TrackObservation(
                        robot_name=report.robot_name,
                        label=det.label,
                        confidence=det.confidence,
                        world_x=det.world_x,
                        world_y=det.world_y,
                        stamp_sec=snapshot.received_sec,
                        detection=det,
                    )
                )

        for track in self._tracks.values():
            track.cycle_measurements.clear()
            track.score = max(0.0, track.score - 0.05)

        for obs in sorted(observations, key=lambda item: item.confidence, reverse=True):
            track = self._find_best_track(obs)
            if track is None:
                track = self._create_track(obs)
            if track is None:
                continue
            track.absorb(obs, alpha=self.position_alpha, beta=self.velocity_beta)

        stale_slots = [
            slot_id
            for slot_id, track in self._tracks.items()
            if (now_sec - track.last_update_sec) > self.track_timeout_sec
        ]
        for slot_id in stale_slots:
            del self._tracks[slot_id]

    def active_tracks(self) -> List[Track]:
        return [self._tracks[slot_id] for slot_id in sorted(self._tracks.keys())]

    def export_tensors(
        self, robot_names: Sequence[str]
    ) -> Tuple[List[int], List[str], torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        tracks = self.active_tracks()
        num_robots = len(robot_names)
        num_tracks = len(tracks)

        if num_tracks == 0:
            return (
                [],
                [],
                torch.zeros((0, 2), dtype=torch.float32),
                torch.zeros((0, 2), dtype=torch.float32),
                torch.zeros((num_robots, 0, 2), dtype=torch.float32),
                torch.zeros((num_robots, 0), dtype=torch.bool),
            )

        mu = torch.tensor([[track.x, track.y] for track in tracks], dtype=torch.float32)
        v = torch.tensor([[track.vx, track.vy] for track in tracks], dtype=torch.float32)
        z_world = torch.zeros((num_robots, num_tracks, 2), dtype=torch.float32)
        visible = torch.zeros((num_robots, num_tracks), dtype=torch.bool)

        for t_idx, track in enumerate(tracks):
            for r_idx, robot_name in enumerate(robot_names):
                obs = track.cycle_measurements.get(robot_name)
                if obs is None:
                    continue
                z_world[r_idx, t_idx, 0] = obs.world_x
                z_world[r_idx, t_idx, 1] = obs.world_y
                visible[r_idx, t_idx] = True

        track_ids = [track.track_uid for track in tracks]
        labels = [track.label for track in tracks]
        return track_ids, labels, mu, v, z_world, visible

    def _find_best_track(self, obs: TrackObservation) -> Optional[Track]:
        best_track = None
        best_distance = None

        for track in self._tracks.values():
            if track.label != obs.label:
                continue
            pred_x, pred_y = track.predicted_position(obs.stamp_sec)
            distance = ((pred_x - obs.world_x) ** 2 + (pred_y - obs.world_y) ** 2) ** 0.5
            if distance > self.association_distance_m:
                continue
            if best_distance is None or distance < best_distance:
                best_track = track
                best_distance = distance

        return best_track

    def _create_track(self, obs: TrackObservation) -> Optional[Track]:
        free_slots = [slot for slot in range(self.max_tracks) if slot not in self._tracks]
        if not free_slots:
            return None

        slot_id = free_slots[0]
        track = Track(
            track_uid=self._next_track_uid,
            slot_id=slot_id,
            label=obs.label,
            x=obs.world_x,
            y=obs.world_y,
            vx=0.0,
            vy=0.0,
            last_update_sec=obs.stamp_sec,
            hits=0,
            score=max(1.0, obs.confidence),
        )
        self._next_track_uid += 1
        self._tracks[slot_id] = track
        return track
