"""独立传感器输出、静态 TF、同步三目预览与点云高度着色；不参与制动。"""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import StaticTransformBroadcaster


CAMERAS = ('wide', 'standard', 'tele')
PREVIEW_ORDER = ('tele', 'standard', 'wide')


def colored_cloud(message, lidar_height, obstacle_rgb=0x00FFFF, ground_rgb=0x80FF20):
    """仅着色真实雷达回波；高度阈值不是障碍物语义识别。"""
    points = point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True)
    xyz = np.column_stack([points[name] for name in ('x', 'y', 'z')])
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    output = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    for i, name in enumerate(('x', 'y', 'z')):
        output[name] = xyz[:, i]
    # 亮绿色地面、亮青色高于地面的回波，直接叠在 RViz 车体表面。
    output['rgb'] = np.where(xyz[:, 2] + lidar_height > 0.15, obstacle_rgb, ground_rgb)
    cloud = PointCloud2(header=copy.deepcopy(message.header), height=1, width=len(output),
                        is_bigendian=False, point_step=16, row_step=len(output)*16, is_dense=True)
    cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z'))]
    cloud.fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
    cloud.data = output.tobytes()
    return cloud


from std_msgs.msg import String
from .vehicle_lifecycle import deleted_ids, SCENE_QOS, empty_cloud, black_image
from .cloud_display import CloudDisplay


class PerceptionDisplay(Node):
    def __init__(self):
        super().__init__('perception_display')
        self.declare_parameter('vehicle', 'car_b')
        self.declare_parameter('project_root', '')
        self.vehicle = self.get_parameter('vehicle').value
        root = Path(self.get_parameter('project_root').value)
        rig = ET.parse(root / 'gazebo/models/perception_rig/model.sdf')
        self.sensors = {s.get('name'): s for s in rig.findall('.//sensor')}
        self.lidar_height = float(self.sensors['lidar'].findtext('pose').split()[2])
        self.cv = CvBridge()
        self.frames = {}
        self.first_cloud = True
        self.removed = False
        self.last_cloud = None
        self.cloud_display = CloudDisplay(self)
        self.lifecycle_sub = self.create_subscription(String, '/gazebo/vehicle_lifecycle', self.on_lifecycle, SCENE_QOS)
        self.image_publishers = {}
        self.info_publishers = {}
        self.subscriptions_list = []
        self.montage = self.create_publisher(Image, f'/{self.vehicle}/camera/triple/image_raw', 2)
        for name in CAMERAS:
            self.image_publishers[name] = self.create_publisher(Image, f'/{self.vehicle}/camera/{name}/image_raw', qos_profile_sensor_data)
            self.info_publishers[name] = self.create_publisher(CameraInfo, f'/{self.vehicle}/camera/{name}/camera_info', qos_profile_sensor_data)
            self.subscriptions_list.append(self.create_subscription(
                Image, f'/{self.vehicle}/sensors/raw/{name}/image',
                lambda msg, camera=name: self.image(msg, camera), qos_profile_sensor_data))
            self.subscriptions_list.append(self.create_subscription(
                CameraInfo, f'/{self.vehicle}/sensors/raw/{name}/camera_info',
                lambda msg, camera=name: self.camera_info(msg, camera), qos_profile_sensor_data))
        self.points = self.create_publisher(PointCloud2, f'/{self.vehicle}/lidar/points', 10)
        self.colored = self.create_publisher(PointCloud2, f'/{self.vehicle}/lidar/points_colored', qos_profile_sensor_data)
        self.subscriptions_list.append(self.create_subscription(
            PointCloud2, f'/{self.vehicle}/sensors/raw/lidar/points', self.cloud, 10))
        self.tf = StaticTransformBroadcaster(self)
        self.publish_transforms()

    def on_lifecycle(self, message):
        if self.vehicle not in deleted_ids(message):
            return
        if self.last_cloud is not None:
            self.points.publish(empty_cloud(self.last_cloud))
            self.cloud_display.clear(self.colored, colored_cloud(self.last_cloud, self.lidar_height))
        if not self.removed:
            stamp = self.get_clock().now().to_msg()
            for name, publisher in self.image_publishers.items():
                publisher.publish(black_image(640, 360, f'{self.vehicle}/{name}_optical_frame', stamp))
            self.montage.publish(black_image(640, 1176, 'map', stamp))
            self.frames.clear()
        self.removed = True

    def publish_transforms(self):
        transforms = []
        for name, sensor in self.sensors.items():
            x, y, z, roll, pitch, yaw = map(float, sensor.findtext('pose').split())
            frame = f'{self.vehicle}/{name}_link'
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = f'{self.vehicle}/base_link'
            transform.child_frame_id = frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            # SDF uses Rz(yaw) Ry(pitch) Rx(roll); positive pitch tilts +x down.
            cr, sr = math.cos(roll/2), math.sin(roll/2)
            cp, sp = math.cos(pitch/2), math.sin(pitch/2)
            cy, sy = math.cos(yaw/2), math.sin(yaw/2)
            transform.transform.rotation.x = sr*cp*cy - cr*sp*sy
            transform.transform.rotation.y = cr*sp*cy + sr*cp*sy
            transform.transform.rotation.z = cr*cp*sy - sr*sp*cy
            transform.transform.rotation.w = cr*cp*cy + sr*sp*sy
            transforms.append(transform)
            if name in CAMERAS:
                optical = TransformStamped()
                optical.header = copy.deepcopy(transform.header)
                optical.header.frame_id = frame
                optical.child_frame_id = f'{self.vehicle}/{name}_optical_frame'
                # ROS optical: x 右、y 下、z 前；Gazebo camera: x 前、y 左、z 上。
                optical.transform.rotation.x = -0.5
                optical.transform.rotation.y = 0.5
                optical.transform.rotation.z = -0.5
                optical.transform.rotation.w = 0.5
                transforms.append(optical)
        self.tf.sendTransform(transforms)

    def image(self, msg, camera):
        if self.removed:
            return
        msg.header.frame_id = f'{self.vehicle}/{camera}_optical_frame'
        self.image_publishers[camera].publish(msg)
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        group = self.frames.setdefault(key, {})
        group[camera] = msg
        if all(name in group for name in CAMERAS):
            tiles = []
            for name in PREVIEW_ORDER:
                image = self.cv.imgmsg_to_cv2(group[name], desired_encoding='rgb8')
                tile = np.zeros((image.shape[0]+32, image.shape[1], 3), dtype=np.uint8)
                tile[32:] = image
                degrees = round(math.degrees(float(self.sensors[name].findtext('camera/horizontal_fov'))))
                label = {'tele': 'TELEPHOTO', 'standard': 'MAIN / STANDARD', 'wide': 'WIDE'}[name]
                cv2.putText(tile, f'{label} | {degrees} deg', (12, 23), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 1)
                tiles.append(tile)
            preview = self.cv.cv2_to_imgmsg(np.concatenate(tiles, axis=0), encoding='rgb8')
            preview.header = copy.deepcopy(msg.header)
            # 拼图不是单一相机，不发布虚构 CameraInfo。
            self.montage.publish(preview)
            del self.frames[key]
        while len(self.frames) > 10:
            del self.frames[next(iter(self.frames))]

    def camera_info(self, msg, camera):
        if self.removed:
            return
        msg.header.frame_id = f'{self.vehicle}/{camera}_optical_frame'
        self.info_publishers[camera].publish(msg)

    def cloud(self, msg):
        if self.removed:
            return
        if self.first_cloud:
            self.get_logger().info(f'First raw lidar frame: {msg.header.stamp.sec + msg.header.stamp.nanosec/1e9:.3f}s')
            self.first_cloud = False
        msg.header.frame_id = f'{self.vehicle}/lidar_link'
        self.last_cloud = msg
        self.points.publish(msg)
        self.cloud_display.submit(self.colored, colored_cloud(msg, self.lidar_height))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionDisplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
