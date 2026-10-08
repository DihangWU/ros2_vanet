"""Time-headway spacing plus relative-speed damping."""


def following_acceleration(estimate, ego_speed, config):
    desired = config.stop_gap+config.time_headway*ego_speed
    acceleration = config.kp*(estimate.gap-desired)+config.kd*estimate.relative_speed
    return acceleration, desired
