"""ROS adapter only: lidar + ego speed + V2V -> pure Algorithm/LidarBrake library."""
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from cosim_interfaces.msg import V2VWarning, LongitudinalCommand
from .event_log import EventLog, seconds
from std_msgs.msg import String
from .vehicle_lifecycle import deleted_ids, SCENE_QOS


class LidarBrakeNode(Node):
    def __init__(self):
        super().__init__('lidar_brake')
        self.declare_parameter('project_root', '')
        self.declare_parameter('vehicle', 'car_b')
        root = Path(self.get_parameter('project_root').value).resolve()
        self.vehicle = self.get_parameter('vehicle').value
        sys.path.insert(0, str(root))
        from Algorithm.LidarBrake import Config, LidarBrake
        self.config = Config(**yaml.safe_load((root/'Algorithm/LidarBrake/config.yaml').read_text()))
        # Mount position and physical bumper come from model geometry, not SUMO state.
        rig = ET.parse(root/'gazebo/models/perception_rig/model.sdf')
        self.mount = np.array(list(map(float, rig.find(".//sensor[@name='lidar']/pose").text.split()))[:3])
        car = ET.parse(root/f'gazebo/models/{self.vehicle}/model.sdf')
        chassis = car.find(".//visual[@name='chassis']")
        bumper_x = float(chassis.findtext('pose').split()[0])+float(chassis.findtext('geometry/box/size').split()[0])/2
        self.algorithm = LidarBrake(self.config, bumper_x)
        self.log = EventLog(root, 'lidar_brake')
        self.publisher = self.create_publisher(LongitudinalCommand, '/lidar_brake/cmd', 10)
        self.markers = self.create_publisher(MarkerArray, '/lidar_brake/markers', 10)
        self.cloud_sub = self.create_subscription(PointCloud2, f'/{self.vehicle}/lidar/points', self.cloud, 10)
        self.odom_sub = self.create_subscription(Odometry, f'/{self.vehicle}/odom', self.odom, 10)
        self.warning_sub = self.create_subscription(V2VWarning, '/v2v_warning', self.warning, 10)
        self.scene_stopped = False
        self.lifecycle_sub = self.create_subscription(String, '/gazebo/vehicle_lifecycle', self.on_lifecycle, SCENE_QOS)
        self.speed = None
        self.speed_stamp = None
        self.last_generated = None
        self.sequence = 0
        self.handled = set()
        self.timer = self.create_timer(.01, self.control, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('LidarBrake: raw lidar distance feedback; ego speed only from odometry')

    def on_lifecycle(self, message):
        if deleted_ids(message) & {'car_a', self.vehicle}:
            self.scene_stopped = True
            clear = Marker()
            clear.action = Marker.DELETEALL
            self.markers.publish(MarkerArray(markers=[clear]))

    def odom(self, message):
        # Deliberately do not access pose, car_a/odom or warning position/speed.
        value = message.twist.twist.linear.x
        if math.isfinite(value) and value >= 0:
            self.speed, self.speed_stamp = value, seconds(message.header.stamp)

    def cloud(self, message):
        if message.header.frame_id != f'{self.vehicle}/lidar_link':
            self.get_logger().error('Unexpected lidar frame; rejecting cloud')
            return
        stamp = seconds(message.header.stamp)
        now = self.get_clock().now().nanoseconds/1e9
        if stamp > now+.05+1e-8 or now-stamp > self.config.stale_timeout:
            self.log.write('lidar_frame_rejected', now, measurement_time_s=stamp,
                           reason='future_or_stale')
            return
        data = point_cloud2.read_points(message, field_names=('x','y','z'), skip_nans=True)
        points = np.column_stack([data[name] for name in ('x','y','z')])+self.mount
        estimate = self.algorithm.observe(points, stamp)
        self.log.write('lidar_measurement', now, measurement_time_s=stamp, point_count=len(points),
                       measured_gap_m=None if estimate is None else estimate.gap,
                       relative_speed_mps=None if estimate is None else estimate.relative_speed)
        self.control(force=True)

    def warning(self, message):
        now = self.get_clock().now().nanoseconds/1e9
        age = now-seconds(message.header.stamp)
        if (message.event_id and message.event_id not in self.handled
                and message.source_vehicle == 'car_a' and message.target_vehicle == self.vehicle
                and message.event_type == 'EMERGENCY_BRAKE' and -.05 <= age <= .5):
            self.handled.add(message.event_id)
            self.algorithm.warn(now)
            self.log.write('v2v_trigger', now, event_id=message.event_id, simulated=message.simulated)
            self.control(force=True)

    def control(self, force=False):
        if getattr(self, 'scene_stopped', False):
            return
        if self.speed is None:
            return
        now = self.get_clock().now().nanoseconds/1e9
        if self.algorithm.last_scan is None and now < .2:
            return  # Brief sensor startup; the coordinator waits rather than faking a range.
        if not force and self.last_generated is not None and now <= self.last_generated+1e-8:
            return  # A paused SUMO clock must not repeatedly integrate acceleration.
        if self.speed_stamp is None or now-self.speed_stamp > self.config.stale_timeout:
            return  # Executor independently brakes when commands expire.
        if abs(self.speed_stamp-now) > 1e-8:
            return  # Wait for this frame's ego speed, not the previous Odom.
        decision = self.algorithm.step(self.speed, now)
        self.last_generated = now
        self.sequence += 1
        command = LongitudinalCommand()
        command.header.frame_id = f'{self.vehicle}/base_link'
        command.header.stamp = rclpy.time.Time(seconds=now).to_msg()
        command.sequence, command.target_vehicle = self.sequence, self.vehicle
        command.target_speed_mps = decision.target_speed
        command.acceleration_mps2 = decision.acceleration
        command.measurement_valid = decision.gap is not None and decision.state != 'SENSOR_LOST_BRAKE'
        command.measured_gap_m = decision.gap if decision.gap is not None else 0.
        if decision.measurement_stamp is not None:
            command.measurement_stamp = rclpy.time.Time(seconds=decision.measurement_stamp).to_msg()
        command.state = decision.state
        self.publisher.publish(command)
        self.log.write('control_published', now, sequence=self.sequence, state=decision.state,
                       measured_gap_m=decision.gap, relative_speed_mps=decision.relative_speed,
                       desired_gap_m=decision.desired_gap, ego_speed_mps=self.speed,
                       target_speed_mps=decision.target_speed, acceleration_mps2=decision.acceleration,
                       measurement_time_s=decision.measurement_stamp, ttc_s=decision.ttc)
        marker = Marker()
        marker.header = command.header
        marker.ns, marker.id, marker.type = 'lidar_brake', 0, Marker.TEXT_VIEW_FACING
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = 0., -5., 3.
        marker.pose.orientation.w = 1.
        marker.scale.z = .65
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0., 1., 1., 1.
        gap_text = '--' if decision.gap is None else f'{decision.gap:.3f} m'
        rate_text = '--' if decision.relative_speed is None else f'{decision.relative_speed:.2f} m/s'
        marker.text = (f'LIDAR BRAKE: {decision.state}\nMEASURED GAP: {gap_text}\n'
                       f'TARGET GAP: {decision.desired_gap:.2f} m\nREL SPEED: {rate_text}\n'
                       f'ACCEL: {decision.acceleration:.2f} m/s^2\nSOURCE: LIDAR')
        self.markers.publish(MarkerArray(markers=[marker]))


def main(args=None):
    rclpy.init(args=args)
    node = LidarBrakeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
