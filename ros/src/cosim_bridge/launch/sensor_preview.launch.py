"""Gazebo + sensors + static RViz scene; no SUMO, network or brake controller."""
import math
from pathlib import Path
import runpy
import tempfile
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_preview(context):
    root = Path(LaunchConfiguration('project_root').perform(context)).resolve()
    scenario = LaunchConfiguration('scenario').perform(context)
    if scenario not in ('traffic', 'two_cars'):
        raise ValueError('scenario must be traffic or two_cars')
    rate = float(LaunchConfiguration('playback_rate').perform(context))
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('playback_rate must be finite and positive')
    runpy.run_path(str(root/'gazebo/scripts/build_sensor_sync.py'))['build'](root)
    # Normal demo worlds remain externally stepped. Only this temporary preview runs freely.
    world = ET.parse(root/f'gazebo/worlds/{scenario}.sdf')
    world.find('world/physics/real_time_factor').text = str(rate)
    with tempfile.NamedTemporaryFile(prefix='cosim-sensor-preview-', suffix='.sdf', delete=False) as stream:
        world.write(stream, encoding='utf-8', xml_declaration=True)
        world_file = Path(stream.name)
    def cleanup(_context):
        world_file.unlink(missing_ok=True)
        return []
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    command = ['gz', 'sim', '-r', str(world_file)]
    if gui:
        config = 'traffic_gui.config' if scenario == 'traffic' else 'gui.config'
        command.extend(['--gui-config', str(root/f'gazebo/config/{config}')])
    else:
        command.append('-s')
    parameters = [{'project_root': str(root), 'scenario': scenario, 'use_sim_time': True}]
    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(root/'gazebo/models')),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', str(root/'gazebo/build')),
        ExecuteProcess(cmd=command, output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='preview_clock_bridge',
             arguments=['/world/cosim_demo/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             remappings=[('/world/cosim_demo/clock', '/clock')], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(__file__).with_name('sensors.launch.py'))),
             launch_arguments={'project_root': str(root), 'scenario': scenario}.items()),
        Node(package='cosim_bridge', executable='sensor_preview', parameters=parameters, output='screen'),
    ]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', str(root/'rviz/sensor_preview.rviz')],
            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    root = next((p for p in (Path.cwd(), *Path.cwd().parents)
                 if (p/'gazebo/worlds/traffic.sdf').is_file()), Path.cwd())
    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=str(root)),
        DeclareLaunchArgument('scenario', default_value='traffic'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        OpaqueFunction(function=launch_preview),
    ])
