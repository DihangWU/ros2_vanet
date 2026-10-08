"""自动识别车顶与四角传感器模组，逐车建立独立话题与展示节点。"""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def sensor_nodes(context):
    root = Path(LaunchConfiguration('project_root').perform(context))
    scenario = LaunchConfiguration('scenario').perform(context)
    spec = importlib.util.spec_from_file_location('traffic_scene', root / 'sumo/scripts/traffic_scene.py')
    scene = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(scene)
    actions = []
    for vehicle in scene.vehicles(root, scenario):
        name = vehicle['id']
        model_name = name if (root/f'gazebo/models/{name}/model.sdf').is_file() else 'background_car'
        model = ET.parse(root / f'gazebo/models/{model_name}/model.sdf')
        if any(i.findtext('uri') == 'model://corner_radar_rig' for i in model.findall('.//include')):
            actions.extend(corner_radar_nodes(root, name))
        if any(i.findtext('uri') == 'model://surround_camera_rig' for i in model.findall('.//include')):
            actions.extend(surround_camera_nodes(root, name))
        if not any(i.findtext('uri') == 'model://perception_rig' for i in model.findall('.//include')):
            continue
        base = f'/world/cosim_demo/model/{name}/link/sensor_mount/sensor'
        arguments, remappings = [], []
        for camera in ('wide', 'standard', 'tele'):
            for suffix, ros_type, gz_type in [('image', 'Image', 'Image'), ('camera_info', 'CameraInfo', 'CameraInfo')]:
                topic = f'{base}/{camera}/{suffix}'
                arguments.append(f'{topic}@sensor_msgs/msg/{ros_type}[gz.msgs.{gz_type}')
                remappings.append((topic, f'/{name}/sensors/raw/{camera}/{suffix}'))
        topic = f'{base}/lidar/scan/points'
        arguments.append(f'{topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked')
        remappings.append((topic, f'/{name}/sensors/raw/lidar/points'))
        actions.extend([
            Node(package='ros_gz_bridge', executable='parameter_bridge', name=f'{name}_sensor_bridge',
                 arguments=arguments, remappings=remappings, output='screen'),
            Node(package='cosim_bridge', executable='perception_display', name=f'{name}_perception_display',
                 parameters=[{'project_root': str(root), 'vehicle': name, 'use_sim_time': True}], output='screen'),
        ])
    return actions


def corner_radar_nodes(root, vehicle):
    rig = ET.parse(root/'gazebo/models/corner_radar_rig/model.sdf')
    arguments, remappings = [], []
    for link in rig.findall('.//link'):
        for sensor in link.findall('sensor'):
            name = sensor.get('name')
            topic = f"/world/cosim_demo/model/{vehicle}/link/{link.get('name')}/sensor/{name}/scan/points"
            arguments.append(f'{topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked')
            remappings.append((topic, f'/{vehicle}/radar/raw/{name}/points'))
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name=f'{vehicle}_radar_bridge',
             arguments=arguments, remappings=remappings, output='screen'),
        Node(package='cosim_bridge', executable='corner_radar_display', name=f'{vehicle}_corner_radar_display',
             parameters=[{'project_root': str(root), 'vehicle': vehicle, 'use_sim_time': True}], output='screen'),
    ]


def surround_camera_nodes(root, vehicle):
    rig = ET.parse(root/'gazebo/models/surround_camera_rig/model.sdf')
    arguments, remappings = [], []
    for link in rig.findall('.//link'):
        for sensor in link.findall('sensor'):
            name = sensor.get('name')
            base = f"/world/cosim_demo/model/{vehicle}/link/{link.get('name')}/sensor/{name}"
            for suffix, kind in (('image', 'Image'), ('camera_info', 'CameraInfo')):
                topic = f'{base}/{suffix}'
                arguments.append(f'{topic}@sensor_msgs/msg/{kind}[gz.msgs.{kind}')
                remappings.append((topic, f'/{vehicle}/surround/raw/{name}/{suffix}'))
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name=f'{vehicle}_surround_bridge',
             arguments=arguments, remappings=remappings, output='screen'),
        Node(package='cosim_bridge', executable='surround_camera_display', name=f'{vehicle}_surround_camera_display',
             parameters=[{'project_root': str(root), 'vehicle': vehicle, 'use_sim_time': True}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('project_root'), DeclareLaunchArgument('scenario', default_value='traffic'),
        OpaqueFunction(function=sensor_nodes),
    ])
