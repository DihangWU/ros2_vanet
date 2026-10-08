"""将最新的 ROS2 车辆中心位姿写入 Gazebo；随 SUMO 定步推进展示时间，不控制车辆运动或相机。"""
import copy
import time
import signal
import importlib.util
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose, ControlWorld
from ros_gz_interfaces.msg import Entity


class GazeboSync(Node):
    def __init__(self):
        super().__init__('gazebo_sync')
        self.client = self.create_client(SetEntityPose, '/world/cosim_demo/set_pose')
        self.control = self.create_client(ControlWorld, '/world/cosim_demo/control')
        self.control_pending = None
        self.commanded_steps = 0
        self.target_steps = 0
        self.latest = {}
        self.pending = {}
        self.declare_parameter('project_root', '')
        self.declare_parameter('scenario', 'traffic')
        root = Path(self.get_parameter('project_root').value)
        spec = importlib.util.spec_from_file_location('traffic_scene', root / 'sumo/scripts/traffic_scene.py')
        scene = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(scene)
        vehicle_ids = [v['id'] for v in scene.vehicles(root, self.get_parameter('scenario').value)]
        self.subscriptions_list = [self.create_subscription(
            Odometry, f'/{v}/odom', lambda msg, vehicle=v: self.receive(vehicle, msg), 10)
            for v in vehicle_ids]
        self.timer = self.create_timer(0.05, self.sync)
        self.last_warning = 0.0
        self.confirmed = set()

    def receive(self, vehicle, msg):
        self.latest[vehicle] = copy.deepcopy(msg.pose.pose)
        # Gazebo 世界步长 0.01 s，按 SUMO 时间推进。
        stamp = msg.header.stamp
        self.target_steps = max(self.target_steps, round((stamp.sec + stamp.nanosec / 1e9) / 0.01))

    def sync(self):
        if self.control_pending is not None and self.control_pending.done():
            result = self.control_pending.result()
            if result is not None and result.success:
                self.commanded_steps = self.requested_steps
            else:
                self.get_logger().error('Gazebo 时间步进失败，将重试')
            self.control_pending = None
        if (self.control.service_is_ready() and self.control_pending is None
                and self.target_steps > self.commanded_steps):
            request = ControlWorld.Request()
            request.world_control.pause = True
            request.world_control.multi_step = self.target_steps - self.commanded_steps
            self.requested_steps = self.target_steps
            self.control_pending = self.control.call_async(request)
        if not self.client.service_is_ready():
            if time.monotonic() - self.last_warning > 5:
                self.get_logger().info('等待 Gazebo set_pose 服务……')
                self.last_warning = time.monotonic()
            return
        for vehicle, pose in self.latest.items():
            future = self.pending.get(vehicle)
            if future is not None:
                if not future.done():
                    continue  # 每辆车最多一个在途请求，避免积压旧状态。
                response = future.result()
                if response is None or not response.success:
                    self.get_logger().error(f'{vehicle} 姿态同步失败')
                elif vehicle not in self.confirmed:
                    self.get_logger().info(f'{vehicle} 姿态同步成功')
                    self.confirmed.add(vehicle)
            request = SetEntityPose.Request()
            request.entity.name = vehicle
            request.entity.type = Entity.MODEL
            request.pose = pose
            self.pending[vehicle] = self.client.call_async(request)


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
