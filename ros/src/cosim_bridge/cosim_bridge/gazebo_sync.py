"""Batch vehicle poses before advancing Gazebo's paused sensor world."""
import copy
import time
import signal
import importlib.util
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time as TimeMessage
from gz.transport13 import Node as TransportNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
from rosgraph_msgs.msg import Clock


class GazeboSync(Node):
    def __init__(self):
        super().__init__('gazebo_sync')
        self.latest = {}
        self.latest_steps = {}
        self.actual_steps = 0
        self.pending_steps = None
        self.transport = TransportNode()
        self.clock_sub = self.create_subscription(Clock, '/gazebo/clock', self.receive_clock, 10)
        self.sync_pub = self.create_publisher(TimeMessage, '/gazebo/synced', 10)
        self.declare_parameter('project_root', '')
        self.declare_parameter('scenario', 'traffic')
        root = Path(self.get_parameter('project_root').value)
        spec = importlib.util.spec_from_file_location('traffic_scene', root / 'sumo/scripts/traffic_scene.py')
        scene = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(scene)
        vehicle_ids = [v['id'] for v in scene.vehicles(root, self.get_parameter('scenario').value)]
        self.vehicle_ids = set(vehicle_ids)
        self.subscriptions_list = [self.create_subscription(
            Odometry, f'/{v}/odom', lambda msg, vehicle=v: self.receive(vehicle, msg), 10)
            for v in vehicle_ids]
        self.timer = self.create_timer(0.01, self.sync)
        self.last_warning = 0.0

    def receive_clock(self, message):
        self.actual_steps = round((message.clock.sec + message.clock.nanosec / 1e9) / 0.01)

    def receive(self, vehicle, msg):
        self.latest[vehicle] = copy.deepcopy(msg.pose.pose)
        stamp = msg.header.stamp
        self.latest_steps[vehicle] = round((stamp.sec + stamp.nanosec / 1e9) / 0.01)

    def request(self, service, message):
        success, response = self.transport.request(
            '/world/cosim_demo/' + service, message, type(message), Boolean, 1000)
        if not success or not response.data:
            self.get_logger().warning(f'{service}: transport={success}, accepted={response.data}')
        return success and response.data

    def sync(self):
        if self.pending_steps is not None:
            if self.actual_steps < self.pending_steps:
                return
            self.pending_steps = None
        steps_now = self.actual_steps
        self.sync_pub.publish(TimeMessage(
            sec=steps_now // 100, nanosec=(steps_now % 100) * 10_000_000))
        if set(self.latest_steps) != self.vehicle_ids:
            return
        stamps = set(self.latest_steps.values())
        if len(stamps) != 1:
            return
        target = next(iter(stamps))
        if target <= steps_now:
            return
        poses = Pose_V()
        for vehicle, pose in self.latest.items():
            entry = poses.pose.add()
            entry.name = vehicle
            for axis in ('x', 'y', 'z'):
                setattr(entry.position, axis, getattr(pose.position, axis))
            for axis in ('x', 'y', 'z', 'w'):
                setattr(entry.orientation, axis, getattr(pose.orientation, axis))
        # Nonblocking batch service queues all poses. The blocking variant needs
        # a simulation update and would deadlock while this world is paused.
        if not self.request('set_pose_vector', poses):
            if time.monotonic() - self.last_warning > 5:
                self.get_logger().info('等待 Gazebo 批量姿态服务……')
                self.last_warning = time.monotonic()
            return
        control = WorldControl(pause=False)
        control.run_to_sim_time.sec = target // 100
        control.run_to_sim_time.nsec = (target % 100) * 10_000_000
        if self.request('control', control):
            # A control acknowledgement only accepts the request. Wait for the
            # actual world clock before letting SUMO advance the next frame.
            self.pending_steps = target
        else:
            self.get_logger().error('Gazebo 时间步进请求失败，将重试')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboSync()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
