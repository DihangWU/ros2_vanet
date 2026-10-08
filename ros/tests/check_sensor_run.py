"""演示运行时执行：验证真实图像、独立标定、同步拼图、三维回波及 TF。"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node('check_sensor_run')
received = {}
camera_frames = {}


def record_image(message, name):
    received[name] = message
    key = (message.header.stamp.sec, message.header.stamp.nanosec)
    camera_frames.setdefault(key, {})[name] = message
    while len(camera_frames) > 20:
        del camera_frames[next(iter(camera_frames))]


subscriptions = []
for name in ('wide', 'standard', 'tele', 'triple'):
    subscriptions.append(node.create_subscription(
        Image, f'/car_b/camera/{name}/image_raw',
        lambda m, key=name: record_image(m, key), qos_profile_sensor_data))
for name in ('wide', 'standard', 'tele'):
    subscriptions.append(node.create_subscription(
        CameraInfo, f'/car_b/camera/{name}/camera_info',
        lambda m, key=name: received.__setitem__(key+'_info', m), qos_profile_sensor_data))
subscriptions.append(node.create_subscription(
    PointCloud2, '/car_b/lidar/points_colored', lambda m: received.__setitem__('cloud', m), qos_profile_sensor_data))
buffer = Buffer()
listener = TransformListener(buffer, node)
deadline = time.monotonic()+40
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.2)
        if len(received) == 8 and buffer.can_transform('map', 'car_b/lidar_link', rclpy.time.Time()):
            break
    assert len(received) == 8, f'Missing streams: {received.keys()}'
    for name, degrees in [('wide',120), ('standard',60), ('tele',30)]:
        image, info = received[name], received[name+'_info']
        assert (image.width, image.height) == (640,360)
        assert image.header.frame_id == info.header.frame_id == f'car_b/{name}_optical_frame'
        expected_fx = 320 / math.tan(math.radians(degrees)/2)
        assert abs(info.k[0]-expected_fx) < .1
        assert np.std(np.frombuffer(image.data, dtype=np.uint8)) > 10, 'Empty camera image'
    montage = received['triple']
    assert (montage.width, montage.height) == (640,1176)
    key = (montage.header.stamp.sec, montage.header.stamp.nanosec)
    cv = CvBridge()
    preview = cv.imgmsg_to_cv2(montage, 'rgb8')
    for index, name in enumerate(('tele', 'standard', 'wide')):
        frame = cv.imgmsg_to_cv2(camera_frames[key][name], 'rgb8')
        assert np.array_equal(preview[index*392+32:(index+1)*392], frame), 'Wrong preview order or timestamp'
    cloud = received['cloud']
    assert cloud.header.frame_id == 'car_b/lidar_link'
    points = point_cloud2.read_points(cloud)
    assert len(points) > 1000
    assert np.ptp(points['z']) > .5, 'Cloud must be three-dimensional'
    assert {0x00FFFF, 0x80FF20}.issubset(set(np.unique(points['rgb'])))
    tf = buffer.lookup_transform('car_b/base_link', 'car_b/lidar_link', rclpy.time.Time())
    assert abs(tf.transform.translation.z-1.85)<1e-6
    assert abs(tf.transform.translation.x-.90)<1e-6
    camera_tf = buffer.lookup_transform('car_b/base_link', 'car_b/standard_link', rclpy.time.Time())
    assert abs(camera_tf.transform.translation.x-1.05)<1e-6
    assert abs(camera_tf.transform.translation.z-1.85)<1e-6
    print('PASS: three cameras/calibrations, 640x1176 vertical montage, real 3D ground/obstacle returns and lidar TF.')
finally:
    node.destroy_node()
    rclpy.shutdown()
