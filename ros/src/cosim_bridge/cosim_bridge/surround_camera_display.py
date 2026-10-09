"""Independent side/rear cameras and timestamp-matched single-column camera preview."""
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
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster

SIDE_CAMERAS = ('rear', 'left', 'right')
PREVIEW_ROWS = tuple((name,) for name in ('tele', 'standard', 'wide', 'rear', 'left', 'right'))
LABELS = dict(tele='TELEPHOTO', standard='MAIN / STANDARD', wide='WIDE',
              rear='REAR', left='LEFT', right='RIGHT')


def camera_specs(root, rig):
    model = ET.parse(Path(root)/f'gazebo/models/{rig}/model.sdf')
    return {sensor.get('name'): sensor for sensor in model.findall('.//sensor')
            if sensor.get('type') == 'camera'}


def make_preview(frames, specs, rows, cv):
    """Each tile includes its own title; images retain their original pixels."""
    result = []
    for row in rows:
        tiles = []
        for name in row:
            image = cv.imgmsg_to_cv2(frames[name], desired_encoding='rgb8')
            tile = np.zeros((image.shape[0]+32, image.shape[1], 3), dtype=np.uint8)
            tile[32:] = image
            degrees = round(math.degrees(float(specs[name].findtext('camera/horizontal_fov'))))
            cv2.putText(tile, f'{LABELS[name]} | {degrees} deg', (12, 23),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
            tiles.append(tile)
        result.append(np.concatenate(tiles, axis=1))
    return np.concatenate(result, axis=0)


from std_msgs.msg import String
from .vehicle_lifecycle import deleted_ids, SCENE_QOS, black_image


class SurroundCameraDisplay(Node):
    def __init__(self):
        super().__init__('surround_camera_display')
        self.declare_parameter('vehicle', 'car_b')
        self.declare_parameter('project_root', '')
        self.vehicle = self.get_parameter('vehicle').value
        root = Path(self.get_parameter('project_root').value)
        self.specs = camera_specs(root, 'surround_camera_rig')
        model_name = self.vehicle if (root/f'gazebo/models/{self.vehicle}/model.sdf').is_file() else 'background_car'
        model = ET.parse(root/f'gazebo/models/{model_name}/model.sdf')
        front_installed = any(i.findtext('uri') == 'model://perception_rig' for i in model.findall('.//include'))
        self.rows = PREVIEW_ROWS if front_installed else tuple((name,) for name in SIDE_CAMERAS)
        if front_installed:
            self.specs.update(camera_specs(root, 'perception_rig'))
        self.cv, self.frames = CvBridge(), {}
        self.removed = False
        self.lifecycle_sub = self.create_subscription(String, '/gazebo/vehicle_lifecycle', self.on_lifecycle, SCENE_QOS)
        self.images, self.infos, self.subscriptions_list = {}, {}, []
        self.preview = self.create_publisher(Image, f'/{self.vehicle}/camera/surround/image_raw', 2)
        for name in SIDE_CAMERAS:
            self.images[name] = self.create_publisher(Image, f'/{self.vehicle}/camera/{name}/image_raw', qos_profile_sensor_data)
            self.infos[name] = self.create_publisher(CameraInfo, f'/{self.vehicle}/camera/{name}/camera_info', qos_profile_sensor_data)
            self.subscriptions_list.append(self.create_subscription(
                Image, f'/{self.vehicle}/surround/raw/{name}/image',
                lambda msg, key=name: self.image(msg, key), qos_profile_sensor_data))
            self.subscriptions_list.append(self.create_subscription(
                CameraInfo, f'/{self.vehicle}/surround/raw/{name}/camera_info',
                lambda msg, key=name: self.info(msg, key), qos_profile_sensor_data))
        if front_installed:
            for name in ('tele', 'standard', 'wide'):
                self.subscriptions_list.append(self.create_subscription(
                    Image, f'/{self.vehicle}/camera/{name}/image_raw',
                    lambda msg, key=name: self.record(msg, key), qos_profile_sensor_data))
        self.tf = StaticTransformBroadcaster(self)
        transforms = []
        for name in SIDE_CAMERAS:
            x, y, z, roll, pitch, yaw = map(float, self.specs[name].findtext('pose').split())
            mount = TransformStamped()
            mount.header.stamp = self.get_clock().now().to_msg()
            mount.header.frame_id = f'{self.vehicle}/base_link'
            mount.child_frame_id = f'{self.vehicle}/{name}_link'
            mount.transform.translation.x, mount.transform.translation.y, mount.transform.translation.z = x, y, z
            # SDF uses Rz(yaw) Ry(pitch) Rx(roll); positive pitch tilts +x down.
            cr, sr = math.cos(roll/2), math.sin(roll/2)
            cp, sp = math.cos(pitch/2), math.sin(pitch/2)
            cy, sy = math.cos(yaw/2), math.sin(yaw/2)
            mount.transform.rotation.x = sr*cp*cy - cr*sp*sy
            mount.transform.rotation.y = cr*sp*cy + sr*cp*sy
            mount.transform.rotation.z = cr*cp*sy - sr*sp*cy
            mount.transform.rotation.w = cr*cp*cy + sr*sp*sy
            optical = TransformStamped()
            optical.header = copy.deepcopy(mount.header)
            optical.header.frame_id = mount.child_frame_id
            optical.child_frame_id = f'{self.vehicle}/{name}_optical_frame'
            # Optical axes: right, down, forward; camera link: forward, left, up.
            optical.transform.rotation.x, optical.transform.rotation.y = -.5, .5
            optical.transform.rotation.z, optical.transform.rotation.w = -.5, .5
            transforms.extend((mount, optical))
        self.tf.sendTransform(transforms)

    def on_lifecycle(self, message):
        if self.removed or self.vehicle not in deleted_ids(message):
            return
        self.removed = True
        self.frames.clear()
        stamp = self.get_clock().now().to_msg()
        for name, publisher in self.images.items():
            publisher.publish(black_image(640, 360, f'{self.vehicle}/{name}_optical_frame', stamp))
        self.preview.publish(black_image(640*len(self.rows[0]), 392*len(self.rows), 'map', stamp))

    def image(self, message, name):
        if self.removed:
            return
        message.header.frame_id = f'{self.vehicle}/{name}_optical_frame'
        self.images[name].publish(message)
        self.record(message, name)

    def info(self, message, name):
        if self.removed:
            return
        message.header.frame_id = f'{self.vehicle}/{name}_optical_frame'
        self.infos[name].publish(message)

    def record(self, message, name):
        if self.removed:
            return
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        group = self.frames.setdefault(key, {})
        group[name] = message
        if all(camera in group for row in self.rows for camera in row):
            preview = self.cv.cv2_to_imgmsg(make_preview(group, self.specs, self.rows, self.cv), encoding='rgb8')
            preview.header = copy.deepcopy(message.header)
            preview.header.frame_id = f'{self.vehicle}/base_link'
            self.preview.publish(preview)
            del self.frames[key]
        while len(self.frames) > 10:
            del self.frames[next(iter(self.frames))]


def main(args=None):
    rclpy.init(args=args)
    node = SurroundCameraDisplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
