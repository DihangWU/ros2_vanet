"""在项目根目录或 ros 工作空间执行，先开三维窗口，再启动交通仿真。"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = next((p for p in (Path.cwd(), *Path.cwd().parents)
                 if (p / 'sumo/config/demo.sumocfg').is_file()), Path.cwd())
    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=str(root)),
        DeclareLaunchArgument('scenario', default_value='traffic'),
        DeclareLaunchArgument('sumo_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gazebo', default_value='true'),
        DeclareLaunchArgument('startup_delay', default_value='8.0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([LaunchConfiguration('project_root'), 'gazebo', 'models'])),
        ExecuteProcess(cmd=['gz', 'sim',
            PathJoinSubstitution([LaunchConfiguration('project_root'), 'gazebo', 'worlds',
                PythonExpression(["'traffic.sdf' if '", LaunchConfiguration('scenario'), "' == 'traffic' else 'two_cars.sdf'"])]),
            '--gui-config', PathJoinSubstitution([LaunchConfiguration('project_root'), 'gazebo', 'config',
                PythonExpression(["'traffic_gui.config' if '", LaunchConfiguration('scenario'), "' == 'traffic' else 'gui.config'"])] )],
            condition=IfCondition(LaunchConfiguration('gazebo')), output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/world/cosim_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                        '/world/cosim_demo/control@ros_gz_interfaces/srv/ControlWorld'],
             condition=IfCondition(LaunchConfiguration('gazebo')), output='screen'),
        Node(package='cosim_bridge', executable='gazebo_sync',
             condition=IfCondition(LaunchConfiguration('gazebo')), output='screen',
             parameters=[{'project_root': LaunchConfiguration('project_root'), 'scenario': LaunchConfiguration('scenario')}]),
        DeclareLaunchArgument('network', default_value='true'),
        DeclareLaunchArgument('network_gui', default_value='false'),
        DeclareLaunchArgument('communication_range_m', default_value='100.0'),
        DeclareLaunchArgument('test_warning', default_value='true'),
        DeclareLaunchArgument('warning_delay', default_value='0.1'),
        Node(package='cosim_bridge', executable='brake_controller', output='screen',
             parameters=[{'project_root': LaunchConfiguration('project_root'), 'use_sim_time': True}]),
        Node(package='cosim_bridge', executable='test_warning_publisher', output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('network'), "' == 'false' and '", LaunchConfiguration('test_warning'), "' == 'true'"])),
             parameters=[{'project_root': LaunchConfiguration('project_root'), 'use_sim_time': True,
                          'warning_delay': ParameterValue(LaunchConfiguration('warning_delay'), value_type=float)}]),
        DeclareLaunchArgument('duration', default_value='30.0'),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([LaunchConfiguration('project_root'), 'rviz',
                 PythonExpression(["'traffic.rviz' if '", LaunchConfiguration('scenario'), "' == 'traffic' else 'two_cars.rviz'"])] )],
             parameters=[{'use_sim_time': True}]),
        TimerAction(period=LaunchConfiguration('startup_delay'), actions=[
            Node(package='cosim_bridge', executable='sumo_bridge', output='screen',
                 parameters=[{
                     'network_mode': ParameterValue(LaunchConfiguration('network'), value_type=bool),
                     'scenario': LaunchConfiguration('scenario'),
                     'network_gui': ParameterValue(LaunchConfiguration('network_gui'), value_type=bool),
                     'communication_range_m': ParameterValue(LaunchConfiguration('communication_range_m'), value_type=float),
                     'project_root': LaunchConfiguration('project_root'),
                     'sumo_gui': ParameterValue(LaunchConfiguration('sumo_gui'), value_type=bool),
                     'duration': ParameterValue(LaunchConfiguration('duration'), value_type=float),
                     'playback_rate': ParameterValue(LaunchConfiguration('playback_rate'), value_type=float),
                 }]),
        ]),
    ])
