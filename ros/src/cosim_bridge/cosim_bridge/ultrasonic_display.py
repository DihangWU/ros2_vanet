"""Independent bumper/side ultrasonic ray approximations, ranges and display clouds."""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from .cloud_display import CloudDisplay
from .perception_display import colored_cloud
from .vehicle_lifecycle import deleted_ids, SCENE_QOS, empty_cloud


def ultrasonic_specs(root):
    rig = ET.parse(Path(root)/'gazebo/models/ultrasonic_rig/model.sdf')
    specs = {}
    for sensor in rig.findall('.//sensor'):
        scan = sensor.find('lidar/scan/horizontal')
        specs[sensor.get('name')] = dict(
            pose=tuple(map(float, sensor.findtext('pose').split())),
            minimum=float(sensor.findtext('lidar/range/min')),
            maximum=float(sensor.findtext('lidar/range/max')),
            fov=float(scan.findtext('max_angle'))-float(scan.findtext('min_angle')))
    return specs


def range_message(cloud, spec):
    points = point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
    xyz = np.column_stack([points[name] for name in ('x', 'y', 'z')])
    distances = np.linalg.norm(xyz, axis=1)
    valid = distances[np.isfinite(distances) & (distances >= spec['minimum']) & (distances <= spec['maximum'])]
    return Range(header=copy.deepcopy(cloud.header), radiation_type=Range.ULTRASOUND,
                 field_of_view=spec['fov'], min_range=spec['minimum'], max_range=spec['maximum'],
                 range=float(np.min(valid)) if len(valid) else float('inf'))


class UltrasonicDisplay(Node):
    def __init__(self):
        super().__init__('ultrasonic_display')
        self.declare_parameter('vehicle', 'car_b')
        self.declare_parameter('project_root', '')
        self.vehicle = self.get_parameter('vehicle').value
        self.specs = ultrasonic_specs(self.get_parameter('project_root').value)
        self.raw, self.colored, self.ranges, self.subscriptions_list = {}, {}, {}, []
        self.last_clouds, self.removed = {}, False
        self.cloud_display = CloudDisplay(self)
        self.lifecycle_sub = self.create_subscription(String, '/gazebo/vehicle_lifecycle', self.on_lifecycle, SCENE_QOS)
        self.tf = StaticTransformBroadcaster(self)
        transforms = []
        for name, spec in self.specs.items():
            base = f'/{self.vehicle}/ultrasonic/{name}'
            self.raw[name] = self.create_publisher(PointCloud2, base+'/points', 10)
            self.colored[name] = self.create_publisher(PointCloud2, base+'/points_colored', qos_profile_sensor_data)
            self.ranges[name] = self.create_publisher(Range, base+'/range', qos_profile_sensor_data)
            self.subscriptions_list.append(self.create_subscription(PointCloud2,
                f'/{self.vehicle}/ultrasonic/raw/{name}/points', lambda m,key=name:self.cloud(m,key),10))
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = f'{self.vehicle}/base_link'
            transform.child_frame_id = f'{self.vehicle}/ultrasonic/{name}_link'
            x, y, z, roll, pitch, yaw = spec['pose']
            if roll or pitch:
                raise ValueError('Ultrasonic ground coloring requires level mounting')
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.z, transform.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
            transforms.append(transform)
        self.tf.sendTransform(transforms)

    def cloud(self, message, name):
        if self.removed:
            return
        cloud = copy.copy(message)
        cloud.header = copy.deepcopy(message.header)
        cloud.header.frame_id = f'{self.vehicle}/ultrasonic/{name}_link'
        self.last_clouds[name] = cloud
        self.raw[name].publish(cloud)
        self.ranges[name].publish(range_message(cloud, self.specs[name]))
        self.cloud_display.submit(self.colored[name],
            colored_cloud(cloud, self.specs[name]['pose'][2], 0xFFFF00, 0xFFFFFF))

    def on_lifecycle(self, message):
        if self.vehicle not in deleted_ids(message):
            return
        self.removed = True
        for name, cloud in self.last_clouds.items():
            empty = empty_cloud(cloud)
            self.raw[name].publish(empty)
            self.ranges[name].publish(range_message(empty, self.specs[name]))
            self.cloud_display.clear(self.colored[name],
                colored_cloud(cloud, self.specs[name]['pose'][2], 0xFFFF00, 0xFFFFFF))


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicDisplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
