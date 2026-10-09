"""Independent ray-based radar clouds and mounting TFs."""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster
from .perception_display import colored_cloud


def radar_specs(root, rig_name='corner_radar_rig'):
    """Link poses are identity; all mounting parameters live in this SDF."""
    rig = ET.parse(Path(root)/f'gazebo/models/{rig_name}/model.sdf')
    result = {}
    for link in rig.findall('.//link'):
        if any(abs(v) > 1e-9 for v in map(float, link.findtext('pose', '0 0 0 0 0 0').split())):
            raise ValueError('Radar mount links must use identity poses; edit sensor/visual poses instead')
        for sensor in link.findall('sensor'):
            name = sensor.get('name')
            pose = tuple(map(float, sensor.findtext('pose').split()))
            if any(abs(v) > 1e-9 for v in pose[3:5]):
                raise ValueError('Ground coloring currently requires level radar mounting')
            rgb = [round(float(v)*255) for v in link.findtext('visual/material/diffuse').split()[:3]]
            result[name] = dict(link=link.get('name'), pose=pose,
                                rgb=(rgb[0]<<16) | (rgb[1]<<8) | rgb[2])
    return result


class CornerRadarDisplay(Node):
    def __init__(self):
        super().__init__('corner_radar_display')
        self.declare_parameter('vehicle', 'car_b')
        self.declare_parameter('project_root', '')
        self.vehicle = self.get_parameter('vehicle').value
        self.declare_parameter('rig_name', 'corner_radar_rig')
        self.specs = radar_specs(self.get_parameter('project_root').value, self.get_parameter('rig_name').value)
        self.raw_publishers, self.colored_publishers = {}, {}
        self.subscriptions_list = []
        self.received = set()
        self.tf = StaticTransformBroadcaster(self)
        transforms = []
        for name, spec in self.specs.items():
            base = f'/{self.vehicle}/radar/{name}'
            self.raw_publishers[name] = self.create_publisher(PointCloud2, base+'/points', 10)
            self.colored_publishers[name] = self.create_publisher(PointCloud2, base+'/points_colored', qos_profile_sensor_data)
            self.subscriptions_list.append(self.create_subscription(
                PointCloud2, f'/{self.vehicle}/radar/raw/{name}/points',
                lambda message, key=name: self.cloud(message, key), 10))
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = f'{self.vehicle}/base_link'
            transform.child_frame_id = f'{self.vehicle}/radar/{name}_link'
            x, y, z, _, _, yaw = spec['pose']
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.z = math.sin(yaw/2)
            transform.transform.rotation.w = math.cos(yaw/2)
            transforms.append(transform)
        self.tf.sendTransform(transforms)

    def cloud(self, message, name):
        cloud = copy.copy(message)
        cloud.header = copy.deepcopy(message.header)
        cloud.header.frame_id = f'{self.vehicle}/radar/{name}_link'
        self.raw_publishers[name].publish(cloud)
        spec = self.specs[name]
        self.colored_publishers[name].publish(colored_cloud(cloud, spec['pose'][2], spec['rgb']))
        if name not in self.received:
            self.get_logger().info(f'{name}: Gazebo radar ray cloud received')
            self.received.add(name)


def main(args=None):
    rclpy.init(args=args)
    node = CornerRadarDisplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
