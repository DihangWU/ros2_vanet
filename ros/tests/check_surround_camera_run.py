"""Run alongside the scene: independent 6-camera streams, calibrations, TF and preview."""
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from cosim_bridge.surround_camera_display import camera_specs, PREVIEW_ROWS

root = Path(__file__).resolve().parents[2]
specs = camera_specs(root, 'surround_camera_rig')
specs.update(camera_specs(root, 'perception_rig'))
rclpy.init()
node = Node('check_surround_camera_run')
frames, infos, previews = {}, {}, []
buffer = Buffer()
listener = TransformListener(buffer, node)


def image(message, name):
    key = (message.header.stamp.sec, message.header.stamp.nanosec)
    frames.setdefault(key, {})[name] = message
    while len(frames) > 30:
        del frames[next(iter(frames))]


subscriptions = []
for name in specs:
    subscriptions.append(node.create_subscription(Image, f'/car_b/camera/{name}/image_raw',
        lambda m, key=name: image(m, key), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(CameraInfo, f'/car_b/camera/{name}/camera_info',
        lambda m, key=name: infos.update({key: m}), qos_profile_sensor_data))
subscriptions.append(node.create_subscription(Image, '/car_b/camera/surround/image_raw',
    lambda m: previews.append(m), qos_profile_sensor_data))
try:
    deadline = time.monotonic()+35
    match = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        for preview in previews[-15:]:
            key = (preview.header.stamp.sec, preview.header.stamp.nanosec)
            if (set(frames.get(key, {})) == set(specs) and set(infos) == set(specs)
                    and all(buffer.can_transform('car_b/base_link', f'car_b/{n}_optical_frame', rclpy.time.Time()) for n in specs)):
                match = preview, frames[key]
                break
        if match:
            break
        if len(previews) > 30:
            del previews[:-15]
    assert match, f'Missing synchronized preview / streams / TF: info={list(infos)}, preview={len(previews)}'
    preview, group = match
    assert (preview.width, preview.height) == (1280, 1176)
    cv = CvBridge()
    pixels = cv.imgmsg_to_cv2(preview, 'rgb8')
    for row, pair in enumerate(PREVIEW_ROWS):
        for col, name in enumerate(pair):
            message, info = group[name], infos[name]
            assert message.header.frame_id == info.header.frame_id == f'car_b/{name}_optical_frame'
            assert (message.width, message.height) == (640, 360)
            frame = cv.imgmsg_to_cv2(message, 'rgb8')
            assert np.std(frame) > 10, f'{name}: empty scene image'
            assert np.array_equal(pixels[row*392+32:(row+1)*392, col*640:(col+1)*640], frame), f'{name}: wrong tile or timestamp'
            fov = float(specs[name].findtext('camera/horizontal_fov'))
            assert abs(info.k[0]-320/math.tan(fov/2)) < .1, f'{name}: wrong intrinsic calibration'
            transform = buffer.lookup_transform('car_b/base_link', f'car_b/{name}_optical_frame', rclpy.time.Time()).transform
            pose = tuple(map(float, specs[name].findtext('pose').split()))
            assert np.allclose([transform.translation.x, transform.translation.y, transform.translation.z], pose[:3])
            q = transform.rotation
            optical_forward = [2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x), 1-2*(q.x*q.x+q.y*q.y)]
            expected_forward = [math.cos(pose[4])*math.cos(pose[5]),
                                math.cos(pose[4])*math.sin(pose[5]), -math.sin(pose[4])]
            assert np.allclose(optical_forward, expected_forward, atol=1e-6), f'{name}: incorrect viewing TF'
    print('PASS: six independent scene images and calibrations, correct optical TFs, synchronized 2-column x 3-row preview.')
finally:
    node.destroy_node()
    rclpy.shutdown()
