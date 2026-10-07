"""测试输入：将前车急刹事件延迟后发布为警告，不模拟无线网络。"""
import copy
import math
import signal
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rosgraph_msgs.msg import Clock
from cosim_interfaces.msg import V2VWarning
from .event_log import EventLog, seconds


class TestWarningPublisher(Node):
    def __init__(self):
        super().__init__('test_warning_publisher')
        self.declare_parameter('project_root', '')
        self.declare_parameter('warning_delay', 0.1)
        self.delay = float(self.get_parameter('warning_delay').value)
        if not math.isfinite(self.delay) or not 0 <= self.delay <= 0.4:
            raise ValueError('warning_delay 必须在 0 到 0.4 秒之间')
        self.log = EventLog(self.get_parameter('project_root').value, 'test_warning_publisher')
        self.pending = None
        self.sent = set()
        self.publisher = self.create_publisher(V2VWarning, '/v2v_warning', 10)
        self.event_sub = self.create_subscription(V2VWarning, '/demo/front_brake', self.receive, 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.on_clock, 10)

    def receive(self, event):
        if event.event_id not in self.sent:
            self.pending = copy.deepcopy(event)

    def on_clock(self, clock):
        now = seconds(clock.clock)
        if self.pending is None or now + 1e-8 < seconds(self.pending.header.stamp) + self.delay:
            return
        warning = self.pending
        warning.simulated = True
        self.publisher.publish(warning)
        self.sent.add(warning.event_id)
        self.pending = None
        self.log.write('test_warning_published', now, event_id=warning.event_id,
                       source_time_s=seconds(warning.header.stamp), simulated=True)
        self.get_logger().info(f'TEST INPUT: /v2v_warning，t={now:.2f}s（非无线收包）')


def main(args=None):
    rclpy.init(args=args)
    node = TestWarningPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
