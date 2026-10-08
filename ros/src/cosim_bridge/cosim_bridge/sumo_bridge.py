"""读取 SUMO 真值，发布 ROS2 状态；执行 ROS2 后车制动命令。"""
import json
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
from builtin_interfaces.msg import Time as TimeMessage
from tf2_ros import TransformBroadcaster
from .car_visuals import car_markers
from .sensor_visuals import sensor_markers
from cosim_interfaces.msg import V2VWarning, BrakeCommand, LongitudinalCommand
from .event_log import EventLog, seconds
from .communication_visuals import CommunicationVisuals


class SumoBridge(Node):
    def __init__(self):
        super().__init__('sumo_bridge')
        self.declare_parameter('project_root', '')
        self.declare_parameter('sumo_gui', True)
        self.declare_parameter('network_mode', False)
        self.declare_parameter('network_gui', False)
        self.declare_parameter('scenario', 'traffic')
        self.declare_parameter('show_background_labels', False)
        self.scenario = self.get_parameter('scenario').value
        self.network_mode = self.get_parameter('network_mode').value
        self.declare_parameter('control_mode', 'sumo')
        self.control_mode = self.get_parameter('control_mode').value
        if self.control_mode not in ('lidar', 'sumo'):
            raise ValueError('control_mode 必须为 lidar 或 sumo')
        self.lidar_mode = self.control_mode == 'lidar'
        self.longitudinal = None
        self.last_control_sequence = 0
        self.gazebo_synced_time = -1.0
        self.sync_wait_started = None
        self.longitudinal_sub = self.create_subscription(LongitudinalCommand, '/lidar_brake/cmd', self.on_longitudinal, 10)
        self.sync_sub = self.create_subscription(TimeMessage, '/gazebo/synced', self.on_gazebo_sync, 10)
        self.declare_parameter('communication_range_m', 100.0)
        radius = float(self.get_parameter('communication_range_m').value)
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('communication_range_m 必须为有限正数')
        self.communication = CommunicationVisuals(radius)
        self.communication.simulated = not self.network_mode
        self.communication_pub = self.create_publisher(MarkerArray, '/v2v/markers', 10)
        self.declare_parameter('duration', 30.0)
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
        scene_spec = importlib.util.spec_from_file_location('traffic_scene', self.root / 'sumo/scripts/traffic_scene.py')
        scene = importlib.util.module_from_spec(scene_spec)
        scene_spec.loader.exec_module(scene)
        scene.build_network(self.root, self.scenario)
        self.road_length = scene.road_length(self.root, self.scenario)
        self.sumo_config = scene.scene_files(self.root, self.scenario)[3]
        self.vehicle_ids = [v['id'] for v in scene.vehicles(self.root, self.scenario)]
        self.odom = {v: self.create_publisher(Odometry, f'/{v}/odom', 10)
                     for v in self.vehicle_ids}
        self.path_pub = {v: self.create_publisher(RosPath, f'/{v}/path', 10)
                         for v in self.odom}
        self.paths = {v: RosPath() for v in self.odom}
        self.markers = self.create_publisher(MarkerArray, '/demo/markers', 10)
        self.clock_pub = self.create_publisher(ClockMessage, '/clock', 10)
        self.tf = TransformBroadcaster(self)
        self.log = EventLog(self.root, 'sumo_bridge')
        self.real_warning_pub = self.create_publisher(V2VWarning, '/v2v_warning', 10)
        self.front_event_pub = self.create_publisher(V2VWarning, '/demo/front_brake', 10)
        self.warning_sub = self.create_subscription(V2VWarning, '/v2v_warning', self.on_warning, 10)
        self.command_sub = self.create_subscription(BrakeCommand, '/brake_cmd', self.on_command, 10)
        self.warning_seen = False
        self.simulated_warning = False
        self.pending_command = None
        self.applied_command = None
        self.command_ids = set()
        self.rear_held = False
        self.response_recorded = False
        self.minimum_gap = float('inf')
        self.trajectory_dir = self.root / 'ros/log/events'
        (self.trajectory_dir / 'summary.json').unlink(missing_ok=True)
        self.trajectory = (self.trajectory_dir / 'trajectory.csv').open('w')
        self.trajectory.write('time_s,car_a_x_m,car_a_speed_mps,car_b_x_m,car_b_speed_mps,gap_m\n')
        self.traffic_trajectory = (self.trajectory_dir / 'traffic_trajectory.csv').open('w')
        self.traffic_trajectory.write('time_s,vehicle_id,x_m,y_m,yaw_rad,speed_mps\n')
        self.braked = False
        self.holding = False
        self.finished = False
        self.closed = False
        self.now = 0.0
        self.states = {}
        cmd = [binary, '-c', str(self.sumo_config),
               '--end', str(self.duration), '--seed', '42', '--no-step-log', 'true']
        if self.gui:
            cmd += ['--delay', '0', '--quit-on-end']
        if self.network_mode:
            from .network_backend import NetworkBackend
            self.get_logger().info('连接真实 Veins + INET 网络；SUMO 打开后请点击开始。')
            self.network_backend = NetworkBackend(self)
        else:
            if self.gui:
                self.get_logger().info('SUMO 窗口准备好后，请点击绿色“开始”；演示时间从开始运行计算。')
            self.traci.start(cmd)
        self.started = time.monotonic()
        # 雷达模式频繁检查同步就绪；真正步进仍每次 0.05 仿真秒，
        # 现实节拍由 started / rate 的绝对时间控制，不累计等待误差。
        self.last_state_wall = 0.0
        self.timer = self.create_timer(.01 if self.lidar_mode else 0.05 / self.rate, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'开始 {self.rate:g} 倍实时播放；{self.duration:g} 秒后保留画面，Ctrl+C 退出。')

    def on_warning(self, warning):
        if warning.source_vehicle == 'car_a' and warning.target_vehicle == 'car_b' and warning.event_type == 'EMERGENCY_BRAKE':
            self.warning_seen = True
            self.simulated_warning = warning.simulated
            if warning.simulated:
                self.communication.event_id = warning.event_id
                self.communication.simulated = True
                self.communication.received = self.now
                self.communication.send_position = (warning.position.x, warning.position.y)
                if 'car_b' in self.states:
                    self.communication.receive_position = self.states['car_b'][:2]
                    self.communication.animation_started = time.monotonic()

    def on_command(self, command):
        age = self.now - seconds(command.header.stamp)
        valid = (not self.finished and bool(command.event_id)
                 and command.header.frame_id == 'map' and command.target_vehicle == 'car_b'
                 and command.target_speed_mps == 0.0
                 and math.isfinite(command.desired_gap_m) and 2.5 <= command.desired_gap_m <= 10.0
                 and -0.05 <= age <= 0.5)
        if not valid or command.event_id in self.command_ids:
            self.log.write('command_rejected', self.now, event_id=command.event_id,
                           reason='invalid_stale_duplicate_or_finished')
            return
        self.command_ids.add(command.event_id)
        if self.applied_command is not None or self.pending_command is not None:
            self.log.write('command_rejected', self.now, event_id=command.event_id, reason='already_braking')
            return
        self.pending_command = command
        self.communication.published = seconds(command.header.stamp)
        self.log.write('brake_command_received', self.now, event_id=command.event_id,
                       published_time_s=seconds(command.header.stamp))

    def on_longitudinal(self, command):
        age = self.now-seconds(command.header.stamp)
        valid = (self.lidar_mode and not self.finished and command.target_vehicle == 'car_b'
                 and command.header.frame_id == 'car_b/base_link'
                 and math.isfinite(command.target_speed_mps) and 0 <= command.target_speed_mps <= 15
                 and math.isfinite(command.acceleration_mps2) and -8 <= command.acceleration_mps2 <= 1.5
                 and -.05 <= age <= .35 and command.sequence > self.last_control_sequence)
        if valid:
            self.longitudinal = command
            self.last_control_sequence = command.sequence

    def on_gazebo_sync(self, stamp):
        self.gazebo_synced_time = max(self.gazebo_synced_time, seconds(stamp))

    def sensor_frame_ready(self):
        if not self.lidar_mode or self.now <= 0:
            return True
        # A 10 Hz lidar may not produce its first scan at t=0.05. Allow only a
        # short startup interval to collect it; keep the initial cruise command.
        control_ready = (self.longitudinal is None and self.now <= .2) or (
            self.longitudinal is not None and seconds(self.longitudinal.header.stamp) >= self.now-1e-8
            and seconds(self.longitudinal.measurement_stamp) >= self.now-.200001)
        ready = self.gazebo_synced_time >= self.now-1e-8 and control_ready
        if ready:
            self.sync_wait_started = None
            return True
        if self.sync_wait_started is None:
            self.sync_wait_started = time.monotonic()
        elif time.monotonic()-self.sync_wait_started > 15:
            raise RuntimeError('等待 Gazebo 同步/雷达控制超时；仿真暂停，请检查传感器与 lidar_brake 日志')
        return False

    def execute_lidar_speed(self):
        command = self.longitudinal
        speed = self.traci.vehicle.getSpeed('car_b')
        target = max(0., speed-8*.05) if command is None or self.now-seconds(command.header.stamp)>.35 else command.target_speed_mps
        if command is None and self.now <= .2:
            target = speed
        self.traci.vehicle.setSpeedMode('car_b', 30)
        self.traci.vehicle.setSpeed('car_b', target)

    def tick(self):
        if self.network_mode:
            if self.communication.failed:
                self.publish_state(False)
                return
            try:
                self.network_backend.tick()
            except (OSError, RuntimeError) as error:
                self.communication.failed = True
                self.get_logger().error(str(error))
                self.publish_state(False)
            return
        advanced = not self.finished
        if not self.finished:
            if self.lidar_mode and self.now > 0 and time.monotonic() < self.started+(self.now+.05)/self.rate:
                self.publish_state(False)
                return
            if not self.sensor_frame_ready():
                self.publish_state(False)
                return
            if self.now >= 5.0 - 1e-8 and not self.braked:
                self.traci.vehicle.slowDown('car_a', 0.0, 2.0)
                self.braked = True
                event = V2VWarning()
                event.header.stamp = rclpy.time.Time(seconds=self.now).to_msg()
                event.header.frame_id = 'map'
                event.event_id = 'front_brake_1'
                event.source_vehicle, event.target_vehicle = 'car_a', 'car_b'
                event.event_type = 'EMERGENCY_BRAKE'
                x, y, _, speed = self.states['car_a']
                event.position = Point(x=x, y=y, z=0.0)
                event.speed_mps = speed
                self.front_event_pub.publish(event)
                self.log.write('front_brake', self.now, event_id=event.event_id)
                self.get_logger().info(
                    f'前车急刹：仿真 {self.now:.2f}s / 现实 {time.monotonic()-self.started:.2f}s')
            if self.now >= 7.0 - 1e-8 and not self.holding:
                self.traci.vehicle.setSpeed('car_a', 0.0)
                self.holding = True
            if self.pending_command is not None:
                command = self.pending_command
                self.applied_command = command
                self.communication.applied = self.now
                self.pending_command = None
                self.response_start_speed = self.traci.vehicle.getSpeed('car_b')
                self.log.write('brake_command_applied', self.now, event_id=command.event_id,
                               speed_before_mps=self.response_start_speed, simulated=command.simulated)
            if self.lidar_mode and self.now > 0:
                self.execute_lidar_speed()
            elif self.applied_command is not None:
                # 根据实时净间距收拢停车位置，SUMO 原有安全跟车约束仍生效。
                front_speed = self.traci.vehicle.getSpeed('car_a')
                rear_speed = self.traci.vehicle.getSpeed('car_b')
                gap = (self.traci.vehicle.getLanePosition('car_a')
                       - self.traci.vehicle.getLength('car_a')
                       - self.traci.vehicle.getLanePosition('car_b'))
                desired_gap = self.applied_command.desired_gap_m
                if front_speed < 0.01 and rear_speed < 0.05 and gap <= desired_gap + 0.05:
                    self.rear_held = True
                target = 0.0 if self.rear_held else min(15.0, max(0.0, front_speed + (gap - desired_gap)))
                self.traci.vehicle.setSpeed('car_b', target)
            # SUMO 内部仍以 0.01 秒步长计算，ROS2 每 0.05 秒取样。
            self.traci.simulationStep(min(round(self.now + 0.05, 8), self.duration))
            self.now = self.traci.simulation.getTime()
            if self.now <= .05:
                self.started = time.monotonic()-self.now/self.rate
            self.communication.connected = True
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
            a, b = self.states['car_a'], self.states['car_b']
            gap = a[0] - b[0] - 5.0
            self.minimum_gap = min(self.minimum_gap, gap)
            self.trajectory.write(f'{self.now:.2f},{a[0]:.4f},{a[3]:.4f},{b[0]:.4f},{b[3]:.4f},{gap:.4f}\n')
            self.trajectory.flush()
            self.record_traffic()
            if self.applied_command is not None and not self.response_recorded and b[3] < self.response_start_speed - 0.01:
                self.response_recorded = True
                self.log.write('rear_speed_response', self.now, event_id=self.applied_command.event_id,
                               speed_mps=b[3])
            if self.gui:
                a, b = self.states['car_a'], self.states['car_b']
                self.traci.gui.setOffset('View #0', (a[0]+b[0])/2, (a[1]+b[1])/2)
            if self.now >= self.duration - 1e-8:
                self.finished = True
                self.trajectory.close()
                self.traffic_trajectory.close()
                summary = dict(end_time_s=self.now, minimum_gap_m=self.minimum_gap,
                               control_mode=self.control_mode, control_sequence=self.last_control_sequence,
                               ros_brake_applied=self.applied_command is not None,
                               warning_seen=self.warning_seen, simulated_warning=self.simulated_warning,
                               scenario=self.scenario, vehicle_count=len(self.states),
                               final_gap_m=gap,
                               final_speed_mps={v: state[3] for v, state in self.states.items()})
                (self.trajectory_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
                self.log.write('simulation_finished', self.now, **summary)
                self.get_logger().info(
                    f'仿真结束：{self.now:.2f}s，现实 {time.monotonic()-self.started:.2f}s；保留最终状态。')
        self.publish_state(append_path=advanced)

    def record_traffic(self):
        for vehicle, (x, y, yaw, speed) in self.states.items():
            self.traffic_trajectory.write(f'{self.now:.2f},{vehicle},{x:.4f},{y:.4f},{yaw:.6f},{speed:.4f}\n')
        self.traffic_trajectory.flush()

    def publish_state(self, append_path):
        wall_now = time.monotonic()
        if not append_path and wall_now-self.last_state_wall < .05:
            return
        self.last_state_wall = wall_now
        stamp = rclpy.time.Time(seconds=self.now).to_msg()
        clock = ClockMessage()
        clock.clock = stamp
        self.clock_pub.publish(clock)
        markers = MarkerArray()
        for vehicle, (x, y, yaw, speed) in self.states.items():
            if vehicle not in self.odom:
                self.odom[vehicle] = self.create_publisher(Odometry, f'/{vehicle}/odom', 10)
                self.path_pub[vehicle] = self.create_publisher(RosPath, f'/{vehicle}/path', 10)
                self.paths[vehicle] = RosPath()
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
            markers.markers.extend(sensor_markers(vehicle, stamp, x, y, yaw, self.root))
            label = self.marker(vehicle, 1, Marker.TEXT_VIEW_FACING, stamp)
            label.pose.position = Point(x=x, y=y, z=3.0)
            label.scale.z = 1.2
            state = 'BACKGROUND' if vehicle.startswith('bg_') else ('STOPPED' if speed < 0.05 else (('BRAKING' if vehicle == 'car_a' else ('ROS BRAKING' if self.applied_command is not None else 'FOLLOWING')) if self.braked else 'DRIVING'))
            label.text = f'{vehicle}: {speed:.2f} m/s  {state}'
            label.color.r = label.color.g = label.color.b = 1.0
            if vehicle.startswith('bg_') and not self.get_parameter('show_background_labels').value:
                label.color.a = 0.0  # Keep the label and its ID; hide only its rendering.
            markers.markers.append(label)
        road = self.marker('road', 0, Marker.CUBE, stamp)
        road_length = self.road_length if self.scenario == 'traffic' else 250.0
        road_width = 9.6 if self.scenario == 'traffic' else 3.2
        road.pose.position = Point(x=road_length/2, y=-road_width/2, z=-0.1)
        road.scale.x, road.scale.y, road.scale.z = road_length, road_width, 0.1
        road.color.r = road.color.g = road.color.b = 0.18
        markers.markers.append(road)
        lines = self.marker('road', 1, Marker.LINE_LIST, stamp)
        lines.scale.x = 0.16
        lines.color.r = lines.color.g = lines.color.b = 1.0
        if self.scenario == 'traffic':
            for y in (-0.15, -9.45):
                lines.points.extend([Point(x=0.0, y=y, z=0.01), Point(x=road_length, y=y, z=0.01)])
            for y in (-3.2, -6.4):
                for x in range(0, int(road_length), 8):
                    lines.points.extend([Point(x=float(x), y=y, z=0.01), Point(x=float(x+3), y=y, z=0.01)])
        else:
            for y in (-0.25, -2.95):
                for x in range(0, 250, 8):
                    lines.points.extend([Point(x=float(x), y=y, z=0.01), Point(x=float(x+3), y=y, z=0.01)])
        markers.markers.append(lines)
        status = self.marker('status', 0, Marker.TEXT_VIEW_FACING, stamp)
        status.pose.position = Point(x=125.0, y=10.0, z=2.0)
        status.scale.z = 2.0
        status.color.r = status.color.g = status.color.b = 1.0
        mode = 'TEST WARNING -> ROS BRAKE' if self.simulated_warning else ('WARNING -> ROS BRAKE' if self.warning_seen else 'following baseline')
        status.text = f'SUMO time: {self.now:.2f} s | {len(self.states)} vehicles | {mode}'
        markers.markers.append(status)
        self.markers.publish(markers)
        self.communication_pub.publish(self.communication.markers(
            self.states, self.now, stamp, self.marker))

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
            if not self.trajectory.closed:
                self.trajectory.close()
            if not self.traffic_trajectory.closed:
                self.traffic_trajectory.close()
            if self.network_mode:
                self.network_backend.close()
                return
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
