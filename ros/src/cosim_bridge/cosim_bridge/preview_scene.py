"""Read preview vehicles and road geometry from Gazebo, without SUMO imports."""
from pathlib import Path
import xml.etree.ElementTree as ET


def world_scene(project_root, scenario):
    if scenario not in ('traffic', 'two_cars'):
        raise ValueError('scenario must be traffic or two_cars')
    return ET.parse(Path(project_root)/f'gazebo/worlds/{scenario}.sdf')


def world_vehicles(project_root, scenario):
    result = []
    for include in world_scene(project_root, scenario).findall('world/include'):
        uri = include.findtext('uri', '')
        if not uri.startswith('model://'):
            raise ValueError('Preview requires local model:// vehicle includes')
        pose = tuple(map(float, include.findtext('pose', '0 0 0 0 0 0').split()))
        if len(pose) != 6 or any(abs(v)>1e-9 for v in pose[2:5]):
            raise ValueError('Vehicle preview supports ground-level vehicles with yaw rotation')
        result.append(dict(id=include.findtext('name', uri[8:]), model=uri[8:], pose=pose))
    return result
