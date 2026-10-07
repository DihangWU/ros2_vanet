"""读取 SUMO 真值，发布 ROS2 状态；暂不接收 ROS2 制动命令。"""
import math
from pathlib import Path
import shutil
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock as ClockMessage
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from .car_visuals import car_markers


class SumoBridge(Node):
    def __init__(self):
        super().__init__('sumo_bridge')
        self.declare_parameter('project_root', '')
        self.declare_parameter('sumo_gui', True)
        self.declare_parameter('duration', 15.0)
        self.declare_parameter('playback_rate', 1.0)
        self.root = Path(self.get_parameter('project_root').value).resolve()
        self.duration = float(self.get_parameter('duration').value)
        self.rate = float(self.get_parameter('playback_rate').value)
        if not math.isfinite(self.duration) or self.duration < 15:
            raise ValueError('duration 必须为至少 15 秒的有限值')
        if not math.isfinite(self.rate) or self.rate <= 0:
            raise ValueError('playback_rate 必须为有限正数')
        # 复用 SUMO 目录中的加载、路网生成函数，不复制交通配置。
        import importlib.util
        script = self.root / 'sumo/scripts/run_demo.py'
        spec = importlib.util.spec_from_file_location('sumo_demo', script)
        demo = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(demo)
        self.gui = self.get_parameter('sumo_gui').value
        binary = shutil.which('sumo-gui' if self.gui else 'sumo')
        if not binary:
            raise RuntimeError('找不到 SUMO，请检查 PATH')
        self.traci = demo.load_traci(binary)
        demo.build_network()
        self.odom = {v: self.create_publisher(Odometry, f'/{v}/odom', 10)
                     for v in ('car_a', 'car_b')}
        self.path_pub = {v: self.create_publisher(RosPath, f'/{v}/path', 10)
                         for v in self.odom}
        self.paths = {v: RosPath() for v in self.odom}
        self.markers = self.create_publisher(MarkerArray, '/demo/markers', 10)
        self.clock_pub = self.create_publisher(ClockMessage, '/clock', 10)
        self.tf = TransformBroadcaster(self)
        self.braked = False
        self.holding = False
        self.finished = False
        self.closed = False
        self.now = 0.0
        self.states = {}
        cmd = [binary, '-c', str(self.root / 'sumo/config/demo.sumocfg'),
               '--end', str(self.duration), '--seed', '42', '--no-step-log', 'true']
        if self.gui:
            cmd += ['--delay', '0', '--quit-on-end']
        if self.gui:
            self.get_logger().info('SUMO 窗口准备好后，请点击绿色“开始”；演示时间从开始运行计算。')
        self.traci.start(cmd)
        self.started = time.monotonic()
        # 独立于 /clock 的稳态定时器；每个回调推进 0.05 仿真秒。
        self.timer = self.create_timer(0.05 / self.rate, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'开始 {self.rate:g} 倍实时播放；{self.duration:g} 秒后保留画面，Ctrl+C 退出。')

    def tick(self):
        advanced = not self.finished
        if not self.finished:
            if self.now >= 5.0 - 1e-8 and not self.braked:
                self.traci.vehicle.slowDown('car_a', 0.0, 2.0)
                self.braked = True
                self.get_logger().info(
                    f'前车急刹：仿真 {self.now:.2f}s / 现实 {time.monotonic()-self.started:.2f}s')
            if self.now >= 7.0 - 1e-8 and not self.holding:
                self.traci.vehicle.setSpeed('car_a', 0.0)
                self.holding = True
            # SUMO 内部仍以 0.01 秒步长计算，ROS2 每 0.05 秒取样。
            self.traci.simulationStep(min(round(self.now + 0.05, 8), self.duration))
            self.now = self.traci.simulation.getTime()
            if set(self.traci.vehicle.getIDList()) != set(self.odom):
                raise RuntimeError('两辆车未按预期存在于 SUMO 中')
            if self.traci.simulation.getCollidingVehiclesIDList():
                raise RuntimeError('SUMO 检测到碰撞')
            for vehicle in self.odom:
                x, y = self.traci.vehicle.getPosition(vehicle)
                # SUMO 的角度从正北顺时针；ROS yaw 从正东逆时针。
                yaw = math.radians(90 - self.traci.vehicle.getAngle(vehicle))
                # SUMO 返回车头中心；ROS base_link 放在车体中心。
                length = self.traci.vehicle.getLength(vehicle)
                x -= length / 2 * math.cos(yaw)
                y -= length / 2 * math.sin(yaw)
                self.states[vehicle] = (x, y, yaw, self.traci.vehicle.getSpeed(vehicle))
            if self.gui:
                a, b = self.states.values()
                self.traci.gui.setOffset('View #0', (a[0]+b[0])/2, (a[1]+b[1])/2)
            if self.now >= self.duration - 1e-8:
                self.finished = True
                self.get_logger().info(
                    f'仿真结束：{self.now:.2f}s，现实 {time.monotonic()-self.started:.2f}s；保留最终状态。')
        self.publish_state(append_path=advanced)

    def publish_state(self, append_path):
        stamp = rclpy.time.Time(seconds=self.now).to_msg()
        clock = ClockMessage()
        clock.clock = stamp
        self.clock_pub.publish(clock)
        markers = MarkerArray()
        for vehicle, (x, y, yaw, speed) in self.states.items():
            odom = Odometry()
            odom.header.frame_id = 'map'
            odom.header.stamp = stamp
            odom.child_frame_id = f'{vehicle}/base_link'
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = math.sin(yaw / 2)
            odom.pose.pose.orientation.w = math.cos(yaw / 2)
            odom.twist.twist.linear.x = speed
            self.odom[vehicle].publish(odom)
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(transform)
            path = self.paths[vehicle]
            path.header = odom.header
            if append_path:
                pose = PoseStamped()
                pose.header = odom.header
                pose.pose = odom.pose.pose
                path.poses.append(pose)
            self.path_pub[vehicle].publish(path)
            markers.markers.extend(car_markers(vehicle, stamp, x, y, yaw))
            label = self.marker(vehicle, 1, Marker.TEXT_VIEW_FACING, stamp)
            label.pose.position = Point(x=x, y=y, z=3.0)
            label.scale.z = 1.2
            state = 'STOPPED' if speed < 0.05 else (('BRAKING' if vehicle == 'car_a' else 'FOLLOWING') if self.braked else 'DRIVING')
            label.text = f'{vehicle}: {speed:.2f} m/s  {state}'
            label.color.r = label.color.g = label.color.b = 1.0
            markers.markers.append(label)
        road = self.marker('road', 0, Marker.CUBE, stamp)
        road.pose.position = Point(x=125.0, y=-1.6, z=-0.1)
        road.scale.x, road.scale.y, road.scale.z = 250.0, 3.2, 0.1
        road.color.r = road.color.g = road.color.b = 0.18
        markers.markers.append(road)
        lines = self.marker('road', 1, Marker.LINE_LIST, stamp)
        lines.scale.x = 0.16
        lines.color.r = lines.color.g = lines.color.b = 1.0
        for y in (-0.25, -2.95):
            for x in range(0, 250, 8):
                lines.points.extend([Point(x=float(x), y=y, z=0.01), Point(x=float(x+3), y=y, z=0.01)])
        markers.markers.append(lines)
        status = self.marker('status', 0, Marker.TEXT_VIEW_FACING, stamp)
        status.pose.position = Point(x=125.0, y=10.0, z=2.0)
        status.scale.z = 2.0
        status.color.r = status.color.g = status.color.b = 1.0
        status.text = f'SUMO time: {self.now:.2f} s | following baseline (no V2V)'
        markers.markers.append(status)
        self.markers.publish(markers)

    @staticmethod
    def marker(namespace, identifier, kind, stamp):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns, marker.id, marker.type = namespace, identifier, kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        return marker

    def close(self):
        if not self.closed:
            self.closed = True
            # launch 的 SIGINT 也会发给 SUMO，关闭时避免重复中断输出堆栈。
            try:
                self.traci.close()
            except (KeyboardInterrupt, self.traci.exceptions.FatalTraCIError):
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SumoBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
