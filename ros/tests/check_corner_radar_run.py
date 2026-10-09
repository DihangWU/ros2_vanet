"""Run while the traffic scene plays: check independent real corner clouds and TF."""
from pathlib import Path
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from cosim_bridge.corner_radar_display import radar_specs

root = Path(__file__).resolve().parents[2]
specs = radar_specs(root) | radar_specs(root, 'side_radar_rig')
rclpy.init()
node = Node('check_corner_radar_run')
buffer = Buffer()
listener = TransformListener(buffer, node)
received = {}
object_seen = set()
stamps = {name: set() for name in specs}


def receive(message, name, colored):
    received[name, colored] = message
    if colored:
        points = point_cloud2.read_points(message)
        if len(points) and specs[name]['rgb'] in set(np.unique(points['rgb'])):
            object_seen.add(name)
    else:
        stamps[name].add((message.header.stamp.sec, message.header.stamp.nanosec))


subscriptions = []
for name in specs:
    for colored, suffix in ((False, 'points'), (True, 'points_colored')):
        subscriptions.append(node.create_subscription(PointCloud2, f'/car_b/radar/{name}/{suffix}',
            lambda m, key=name, color=colored: receive(m, key, color), qos_profile_sensor_data))
try:
    deadline = time.monotonic()+40
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        if (len(received) == 2*len(specs) and object_seen == set(specs)
                and all(len(values) >= 4 for values in stamps.values())
                and all(buffer.can_transform('map', f'car_b/radar/{name}_link', rclpy.time.Time()) for name in specs)):
            break
    assert len(received) == 2*len(specs), f'Missing raw/colored streams: {received.keys()}'
    assert object_seen == set(specs), f'No obstacle return: {set(specs)-object_seen}'
    for name, spec in specs.items():
        raw, cloud = received[name, False], received[name, True]
        frame = f'car_b/radar/{name}_link'
        assert raw.header.frame_id == cloud.header.frame_id == frame
        assert (raw.width, raw.height) == (240, 16)
        assert len(stamps[name]) >= 4, f'{name}: not a moving timestamp stream'
        data = point_cloud2.read_points(cloud)
        assert len(data) > 50
        xyz = np.column_stack([data[axis] for axis in ('x', 'y', 'z')])
        assert np.isfinite(xyz).all() and np.ptp(xyz[:, 2]) > .1
        assert np.linalg.norm(xyz, axis=1).max() <= 30.01
        assert 0x80FF20 in set(np.unique(data['rgb'])), 'Missing ground returns'
        transform = buffer.lookup_transform('car_b/base_link', frame, rclpy.time.Time()).transform
        assert np.allclose([transform.translation.x, transform.translation.y, transform.translation.z], spec['pose'][:3])
        # q and -q represent the same orientation; TF may normalize either way.
        quaternion = np.array([transform.rotation.z, transform.rotation.w])
        expected = np.array([math.sin(spec['pose'][5]/2), math.cos(spec['pose'][5]/2)])
        assert np.allclose(quaternion, expected) or np.allclose(quaternion, -expected)
    print('PASS: six independent real 3D radar clouds, obstacle/ground returns, changing timestamps and mounting TFs.')
finally:
    node.destroy_node()
    rclpy.shutdown()
