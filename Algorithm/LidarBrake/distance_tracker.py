"""Estimate closing/opening speed from lidar distances, without a lead-car velocity."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Estimate:
    gap: float
    relative_speed: float  # positive: opening, negative: closing
    stamp: float


class DistanceTracker:
    def __init__(self, config):
        self.config = config
        self.value = None
        self.previous_raw = None

    def update(self, gap, stamp):
        if self.value is not None and stamp <= self.value.stamp:
            return self.value
        if gap is None or not math.isfinite(gap) or gap < 0:
            self.value = self.previous_raw = None
            return None
        if self.value is None or stamp-self.value.stamp > self.config.stale_timeout:
            result = Estimate(gap, 0.0, stamp)
        else:
            dt = stamp-self.value.stamp
            rate = max(-35.0, min(35.0, (gap-self.previous_raw)/dt))
            alpha = self.config.distance_alpha
            beta = self.config.velocity_alpha
            result = Estimate(alpha*gap+(1-alpha)*self.value.gap,
                              beta*rate+(1-beta)*self.value.relative_speed, stamp)
        self.previous_raw, self.value = gap, result
        return result
