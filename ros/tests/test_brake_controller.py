"""验证有效警告、重复警告、过期警告和错误字段。"""
import json
import tempfile
import unittest
import rclpy
from cosim_interfaces.msg import V2VWarning
from cosim_bridge.brake_controller import BrakeController


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='cosim-controller-test-')
        rclpy.init(args=['--ros-args', '-p', f'project_root:={self.tmp.name}'])
        self.node = BrakeController()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.tmp.cleanup()

    def warning(self):
        msg = V2VWarning()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.event_id = 'test_1'
        msg.source_vehicle, msg.target_vehicle = 'car_a', 'car_b'
        msg.event_type = 'EMERGENCY_BRAKE'
        msg.speed_mps = 15.0
        msg.simulated = True
        return msg

    def events(self):
        return [json.loads(line)['event'] for line in self.node.log.path.read_text().splitlines()]

    def test_valid_then_duplicate(self):
        msg = self.warning()
        self.node.receive(msg)
        self.node.receive(msg)
        self.assertEqual(self.events(), ['warning_received', 'brake_command_published', 'warning_rejected'])

    def test_stale(self):
        msg = self.warning()
        msg.header.stamp.sec -= 2
        self.node.receive(msg)
        self.assertEqual(self.events(), ['warning_rejected'])

    def test_wrong_target(self):
        msg = self.warning()
        msg.target_vehicle = 'unknown_vehicle'
        self.node.receive(msg)
        self.assertEqual(self.events(), ['warning_rejected'])

    def test_nonfinite_speed(self):
        msg = self.warning()
        msg.speed_mps = float('nan')
        self.node.receive(msg)
        self.assertEqual(self.events(), ['warning_rejected'])


if __name__ == '__main__':
    unittest.main()
