"""Pure algorithm coordinator: cloud -> tracked gap -> bounded speed command."""
from dataclasses import dataclass
import math
from .front_obstacle import front_gap
from .distance_tracker import DistanceTracker
from .pd_controller import following_acceleration
from .brake_guard import emergency


@dataclass(frozen=True)
class Decision:
    target_speed: float
    acceleration: float
    state: str
    gap: float | None
    relative_speed: float | None
    desired_gap: float
    measurement_stamp: float | None
    ttc: float | None


class LidarBrake:
    def __init__(self, config, front_bumper_x):
        self.config = config
        self.front_bumper_x = front_bumper_x
        self.tracker = DistanceTracker(config)
        self.last_scan = None
        self.last_time = None
        self.last_acceleration = 0.0
        self.warning_time = None
        self.held = False

    def observe(self, points_in_base, stamp):
        if self.last_scan is not None and stamp <= self.last_scan:
            return self.tracker.value
        gap = front_gap(points_in_base, self.config, self.front_bumper_x)
        self.last_scan = stamp
        return self.tracker.update(gap, stamp)

    def warn(self, now):
        self.warning_time = now

    def step(self, ego_speed, now):
        if not math.isfinite(ego_speed) or ego_speed < 0 or not math.isfinite(now):
            raise ValueError('invalid ego speed/time')
        c = self.config
        dt = c.control_period if self.last_time is None else max(.001, min(.1, now-self.last_time))
        self.last_time = now
        value = self.tracker.value
        age = None if self.last_scan is None else now-self.last_scan
        desired = c.stop_gap+c.time_headway*ego_speed
        gap = None if value is None else value.gap
        rate = None if value is None else value.relative_speed
        ttc = None
        if self.held:
            acceleration, state = -c.max_deceleration, 'STOPPED_HOLD'
        elif age is None or age > c.stale_timeout or age < -.05 or value is None:
            # Missing/stale data never means the road is clear.
            acceleration, state = -c.max_deceleration, 'SENSOR_LOST_BRAKE'
        elif (value.gap <= c.stop_gap+c.hold_gap_tolerance
              and ego_speed <= c.hold_speed and abs(value.relative_speed) < .25):
            self.held = True
            acceleration, state = -c.max_deceleration, 'STOPPED_HOLD'
        else:
            acceleration, desired = following_acceleration(value, ego_speed, c)
            urgent, ttc = emergency(value, ego_speed, max(0., age), c)
            state = 'LIDAR_FOLLOW'
            if self.warning_time is not None:
                state = 'V2V_LIDAR_BRAKE'
                if now-self.warning_time < c.warning_duration:
                    acceleration = min(acceleration, -c.warning_deceleration)
            acceleration = max(-c.max_deceleration, min(c.max_acceleration, acceleration))
            if urgent:
                acceleration, state = -c.max_deceleration, 'EMERGENCY_BRAKE'
            else:
                change = c.max_jerk*dt
                acceleration = max(self.last_acceleration-change, min(self.last_acceleration+change, acceleration))
        self.last_acceleration = acceleration
        target = max(0., min(c.cruise_speed, ego_speed+acceleration*c.control_period))
        if self.held:
            target = 0.
        return Decision(target, acceleration, state, gap, rate, desired, self.last_scan,
                        ttc if ttc is not None and math.isfinite(ttc) else None)
