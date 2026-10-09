"""直接复用 Gazebo 独立传感器模型的可视几何，避免两份安装模型漂移。"""
from functools import lru_cache
from pathlib import Path
import math
import xml.etree.ElementTree as ET
from visualization_msgs.msg import Marker


@lru_cache(maxsize=32)
def rig_visuals(project_root, vehicle, rig_name='perception_rig'):
    root = Path(project_root)
    model_name = vehicle if (root/f'gazebo/models/{vehicle}/model.sdf').is_file() else 'background_car'
    model = ET.parse(root / f'gazebo/models/{model_name}/model.sdf')
    if not any(i.findtext('uri') == f'model://{rig_name}' for i in model.findall('.//include')):
        return ()
    rig = ET.parse(root / f'gazebo/models/{rig_name}/model.sdf')
    parts = []
    for visual in rig.findall('.//visual'):
        pose = tuple(map(float, visual.findtext('pose').split()))
        if any(abs(v)>1e-9 for v in pose[3:]):
            raise ValueError('传感器可视几何当前使用与车体平行的安装')
        box = visual.find('geometry/box')
        if box is not None:
            shape, size = Marker.CUBE, tuple(map(float, box.findtext('size').split()))
        else:
            cylinder = visual.find('geometry/cylinder')
            diameter = float(cylinder.findtext('radius'))*2
            shape, size = Marker.CYLINDER, (diameter, diameter, float(cylinder.findtext('length')))
        color = tuple(map(float, visual.findtext('material/diffuse').split()))
        parts.append((pose[:3], shape, size, color))
    return tuple(parts)


def sensor_markers(vehicle, stamp, x, y, yaw, project_root):
    result = []
    for rig_name, namespace in (('perception_rig', 'sensors'), ('corner_radar_rig', 'radars'),
                                ('side_radar_rig', 'side_radars'), ('surround_camera_rig', 'surround_cameras')):
        result.extend(rig_markers(vehicle, stamp, x, y, yaw, project_root, rig_name, namespace))
    return result


def rig_markers(vehicle, stamp, x, y, yaw, project_root, rig_name, namespace):
    result = []
    for index, (offset, shape, size, color) in enumerate(rig_visuals(str(project_root), vehicle, rig_name)):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns, marker.id = vehicle+'/'+namespace, index
        marker.type, marker.action = shape, Marker.ADD
        dx, dy, dz = offset
        marker.pose.position.x = x+dx*math.cos(yaw)-dy*math.sin(yaw)
        marker.pose.position.y = y+dx*math.sin(yaw)+dy*math.cos(yaw)
        marker.pose.position.z = dz
        marker.pose.orientation.z = math.sin(yaw/2)
        marker.pose.orientation.w = math.cos(yaw/2)
        marker.scale.x, marker.scale.y, marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        result.append(marker)
    return result
