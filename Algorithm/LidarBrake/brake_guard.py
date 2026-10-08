"""Independent TTC and stopping-distance guard, using measured gap/range rate only."""
import math


def emergency(estimate, ego_speed, age, config):
    closing = max(0.0, -estimate.relative_speed)
    ttc = estimate.gap/closing if closing > 0.1 else math.inf
    margin = estimate.gap-config.stop_gap-ego_speed*max(0.0, age)
    relative_stopping_distance = closing*closing/(2*config.max_deceleration)
    imminent = closing > 0.5 and (ttc < config.emergency_ttc or margin < relative_stopping_distance)
    return imminent, ttc
