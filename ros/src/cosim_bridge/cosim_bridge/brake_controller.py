"""收到有效警告后，生成一次后车制动命令，不直接访问 TraCI。"""
import math
import signal
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from cosim_interfaces.msg import V2VWarning, BrakeCommand
from .event_log import EventLog, seconds


class BrakeController(Node):
    def __init__(self):
        super().__init__('brake_controller')
        self.declare_parameter('project_root', '')
        self.log = EventLog(self.get_parameter('project_root').value, 'brake_controller')
        self.handled = set()
        self.publisher = self.create_publisher(BrakeCommand, '/brake_cmd', 10)
        self.subscription = self.create_subscription(V2VWarning, '/v2v_warning', self.receive, 10)

    def receive(self, warning):
        now = self.get_clock().now()
        age = now.nanoseconds / 1e9 - seconds(warning.header.stamp)
        valid = (warning.event_id and warning.source_vehicle == 'car_a'
                 and warning.target_vehicle == 'car_b'
                 and warning.event_type == 'EMERGENCY_BRAKE'
                 and warning.header.frame_id == 'map'
                 and math.isfinite(warning.speed_mps) and warning.speed_mps >= 0
                 and -0.05 <= age <= 0.5)
        if not valid or warning.event_id in self.handled:
            self.log.write('warning_rejected', now.nanoseconds / 1e9,
                           event_id=warning.event_id, reason='invalid_stale_or_duplicate')
            return
        self.handled.add(warning.event_id)
        self.log.write('warning_received', now.nanoseconds / 1e9,
                       event_id=warning.event_id, source_time_s=seconds(warning.header.stamp),
                       simulated=warning.simulated)
        command = BrakeCommand()
        command.header.stamp = now.to_msg()
        command.header.frame_id = 'map'
        command.event_id = warning.event_id
        command.target_vehicle = warning.target_vehicle
        command.target_speed_mps = 0.0
        command.desired_gap_m = 2.5
        command.simulated = warning.simulated
        self.publisher.publish(command)
        self.log.write('brake_command_published', now.nanoseconds / 1e9,
                       event_id=command.event_id, target_vehicle=command.target_vehicle,
                       target_speed_mps=0.0, desired_gap_m=2.5)
        self.get_logger().info(f'{command.event_id}: 发布后车制动命令，t={now.nanoseconds/1e9:.2f}s')


def main(args=None):
    rclpy.init(args=args)
    node = BrakeController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
