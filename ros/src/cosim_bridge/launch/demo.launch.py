"""在项目根目录或 ros 工作空间执行，先开 RViz，再启动交通仿真。"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = next((p for p in (Path.cwd(), *Path.cwd().parents)
                 if (p / 'sumo/config/demo.sumocfg').is_file()), Path.cwd())
    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=str(root)),
        DeclareLaunchArgument('sumo_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('duration', default_value='15.0'),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([LaunchConfiguration('project_root'), 'rviz', 'two_cars.rviz'])],
             parameters=[{'use_sim_time': True}]),
        TimerAction(period=3.0, actions=[
            Node(package='cosim_bridge', executable='sumo_bridge', output='screen',
                 parameters=[{
                     'project_root': LaunchConfiguration('project_root'),
                     'sumo_gui': ParameterValue(LaunchConfiguration('sumo_gui'), value_type=bool),
                     'duration': ParameterValue(LaunchConfiguration('duration'), value_type=float),
                     'playback_rate': ParameterValue(LaunchConfiguration('playback_rate'), value_type=float),
                 }]),
        ]),
    ])
