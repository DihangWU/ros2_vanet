"""Run in the preview ROS domain: clock, static scene and sensors without traffic nodes."""
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener
from cosim_bridge.preview_scene import world_vehicles

root = Path(__file__).resolve().parents[2]
rclpy.init()
node = Node('check_sensor_preview_run')
buffer = Buffer()
listener = TransformListener(buffer, node)
received, stamps = {}, {}
subscriptions = []


def record(message, key):
    received[key] = message
    stamp = message.clock if key == '/clock' else (message.markers[0].header.stamp if key == '/demo/markers' else message.header.stamp)
    stamps.setdefault(key, set()).add((stamp.sec, stamp.nanosec))


topics = [('/clock', Clock), ('/demo/markers', MarkerArray),
          ('/car_b/camera/surround/image_raw', Image), ('/car_b/lidar/points_colored', PointCloud2)]
topics += [(f'/car_b/radar/{name}/points_colored', PointCloud2)
           for name in ('front_left', 'front_right', 'rear_left', 'rear_right', 'side_left', 'side_right')]
for topic, kind in topics:
    subscriptions.append(node.create_subscription(kind, topic, lambda m, key=topic: record(m, key), qos_profile_sensor_data))
try:
    deadline = time.monotonic()+35
    vehicles = world_vehicles(root, 'traffic')
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        if (len(received) == len(topics) and all(len(values)>=3 for values in stamps.values())
                and all(buffer.can_transform('map', v['id']+'/base_link', rclpy.time.Time()) for v in vehicles)):
            break
    assert len(received) == len(topics), f'Missing topics: {set(t for t,_ in topics)-received.keys()}'
    assert all(len(values)>=3 for values in stamps.values()), 'Clock or sensor frames do not advance'
    assert node.count_publishers('/clock') == 1, 'Expected only Gazebo clock bridge'
    forbidden = {'sumo_bridge', 'gazebo_sync', 'lidar_brake', 'brake_controller'}
    assert not forbidden.intersection(name for name,_ in node.get_node_names_and_namespaces()), 'Traffic/control nodes running'
    for vehicle in vehicles:
        transform = buffer.lookup_transform('map', vehicle['id']+'/base_link', rclpy.time.Time()).transform
        assert np.allclose([transform.translation.x, transform.translation.y, transform.translation.z], vehicle['pose'][:3])
        assert math.isclose(transform.rotation.w, math.cos(vehicle['pose'][5]/2), abs_tol=1e-6)
    namespaces = {m.ns for m in received['/demo/markers'].markers}
    assert all(v['id']+'/model' in namespaces for v in vehicles)
    assert {'preview/road', 'car_b/sensors', 'car_b/radars', 'car_b/side_radars', 'car_b/surround_cameras'} <= namespaces
    preview = received['/car_b/camera/surround/image_raw']
    assert (preview.width, preview.height) == (1280,1176)
    print('PASS: one advancing Gazebo clock, 11 static vehicle TFs/markers, road, six-camera preview and seven live clouds; no SUMO/control nodes.')
finally:
    node.destroy_node()
    rclpy.shutdown()
