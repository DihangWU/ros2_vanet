"""Verify ROS inputs do not smuggle front-car truth into distance feedback."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from collections import deque
import numpy as np
import rclpy.time
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from cosim_interfaces.msg import V2VWarning
from cosim_bridge.lidar_brake_node import LidarBrakeNode

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Algorithm.LidarBrake import Config, LidarBrake


class AdapterTests(unittest.TestCase):
    def adapter(self):
        return SimpleNamespace(
            vehicle='car_b', config=Config(), mount=np.array([.9, 0., 1.85]),
            algorithm=LidarBrake(Config(), 2.5), speed=None, speed_stamp=None,
            pending_clouds=deque(maxlen=20),
            handled=set(), get_clock=lambda: SimpleNamespace(now=lambda: rclpy.time.Time(seconds=1.)),
            log=SimpleNamespace(write=lambda *args, **kwargs: None),
            control=lambda **kwargs: None)

    def test_only_ego_velocity_is_read_from_odom(self):
        node = self.adapter()
        for position in (1., -1e9, float('nan')):
            message = Odometry()
            message.header.stamp = rclpy.time.Time(seconds=1.).to_msg()
            message.pose.pose.position.x = position
            message.twist.twist.linear.x = 12.
            LidarBrakeNode.odom(node, message)
            self.assertEqual(node.speed, 12.)
            self.assertIsNone(node.algorithm.tracker.value)

    def test_cloud_sets_gap_warning_position_does_not(self):
        node = self.adapter()
        points = np.array([[42.5, y, z] for y in np.linspace(-.5, .5, 9)
                           for z in np.linspace(.4, 1.2, 9)])
        cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=rclpy.time.Time(seconds=1.).to_msg(), frame_id='car_b/lidar_link'),
            points-node.mount)
        LidarBrakeNode.cloud(node, cloud)
        self.assertAlmostEqual(node.algorithm.tracker.value.gap, 40., places=4)
        for index, position in enumerate((1., 1e9, float('nan'))):
            warning = V2VWarning()
            warning.header.stamp = cloud.header.stamp
            warning.source_vehicle, warning.target_vehicle = 'car_a', 'car_b'
            warning.event_type, warning.event_id = 'EMERGENCY_BRAKE', str(index)
            warning.position.x, warning.speed_mps = position, float('nan')
            LidarBrakeNode.warning(node, warning)
            self.assertAlmostEqual(node.algorithm.tracker.value.gap, 40., places=4)
            self.assertEqual(node.algorithm.warning_time, 1.)

    def test_future_cloud_waits_for_clock_and_stale_cloud_remains_rejected(self):
        node = self.adapter()
        points = np.array([[42.5,y,z] for y in np.linspace(-.5,.5,9)
                           for z in np.linspace(.4,1.2,9)])
        cloud = point_cloud2.create_cloud_xyz32(Header(
            stamp=rclpy.time.Time(seconds=1.2).to_msg(), frame_id='car_b/lidar_link'), points-node.mount)
        LidarBrakeNode.cloud(node, cloud)
        self.assertEqual(len(node.pending_clouds),1)
        self.assertIsNone(node.algorithm.tracker.value)
        LidarBrakeNode.consume_pending_clouds(node,1.)
        self.assertEqual(len(node.pending_clouds),1)
        LidarBrakeNode.consume_pending_clouds(node,1.2)
        self.assertFalse(node.pending_clouds)
        self.assertAlmostEqual(node.algorithm.tracker.value.gap,40.,places=4)
        rejected = self.adapter()
        rejected.pending_clouds.append(cloud)
        LidarBrakeNode.consume_pending_clouds(rejected,10.)
        self.assertIsNone(rejected.algorithm.tracker.value)


if __name__ == '__main__':
    unittest.main()
