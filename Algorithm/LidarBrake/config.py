"""All distances in meters, times in seconds, speeds in m/s."""
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class Config:
    stop_gap: float = 2.5
    time_headway: float = 1.2
    cruise_speed: float = 15.0
    kp: float = 0.7
    kd: float = 1.8
    max_acceleration: float = 1.5
    max_deceleration: float = 8.0
    max_jerk: float = 8.0
    control_period: float = 0.05
    stale_timeout: float = 0.35
    roi_half_width: float = 1.05
    min_height: float = 0.2
    max_height: float = 2.4
    max_distance: float = 100.0
    voxel_size: float = 0.3
    min_cluster_points: int = 6
    distance_alpha: float = 0.7
    velocity_alpha: float = 0.45
    emergency_ttc: float = 1.5
    warning_deceleration: float = 2.0
    warning_duration: float = 0.5
    hold_gap_tolerance: float = 0.05
    hold_speed: float = 0.05

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{field.name} must be finite and positive')
        if not 0 < self.distance_alpha <= 1 or not 0 < self.velocity_alpha <= 1:
            raise ValueError('filter coefficients must be in (0, 1]')
        if self.min_height >= self.max_height or self.max_distance <= self.stop_gap:
            raise ValueError('invalid obstacle region')
