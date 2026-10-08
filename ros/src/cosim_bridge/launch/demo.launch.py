"""在项目根目录或 ros 工作空间执行，先开三维窗口，再启动交通仿真。"""
from pathlib import Path
import runpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def validate_control(context):
    mode = LaunchConfiguration('control_mode').perform(context)
    gazebo = LaunchConfiguration('gazebo').perform(context)
    if mode not in ('lidar', 'sumo'):
        raise ValueError('control_mode 必须为 lidar 或 sumo')
    if mode == 'lidar' and gazebo.lower() != 'true':
        raise ValueError('雷达控制需要 Gazebo 生成真实点云；无 Gazebo 对比运行请指定 control_mode:=sumo')
    if gazebo.lower() == 'true':
        root = Path(LaunchConfiguration('project_root').perform(context))
        runpy.run_path(str(root/'gazebo/scripts/build_sensor_sync.py'))['build'](root)
    return []


def generate_launch_description():
    root = next((p for p in (Path.cwd(), *Path.cwd().parents)
                 if (p / 'sumo/config/demo.sumocfg').is_file()), Path.cwd())
    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=str(root)),
        DeclareLaunchArgument('scenario', default_value='traffic'),
        DeclareLaunchArgument('show_background_labels', default_value='false'),
        DeclareLaunchArgument('sumo_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gazebo', default_value='true'),
        DeclareLaunchArgument('control_mode', default_value='lidar'),
        OpaqueFunction(function=validate_control),
        Node(package='cosim_bridge', executable='lidar_brake', output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'lidar'"])),
             parameters=[{'project_root': LaunchConfiguration('project_root'), 'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(__file__).with_name('sensors.launch.py'))),
            launch_arguments={'project_root': LaunchConfiguration('project_root'), 'scenario': LaunchConfiguration('scenario')}.items(),
            condition=IfCondition(LaunchConfiguration('gazebo'))),
        DeclareLaunchArgument('startup_delay', default_value='8.0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([LaunchConfiguration('project_root'), 'gazebo', 'models'])),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', [
            PathJoinSubstitution([LaunchConfiguration('project_root'), 'gazebo', 'build']), ':',
            EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')]),
        ExecuteProcess(cmd=['gz', 'sim',
            PathJoinSubstitution([LaunchConfiguration('project_root'), 'gazebo', 'worlds',
                PythonExpression(["'traffic.sdf' if '", LaunchConfiguration('scenario'), "' == 'traffic' else 'two_cars.sdf'"])]),
            '--gui-config', PathJoinSubstitution([LaunchConfiguration('project_root'), 'gazebo', 'config',
                PythonExpression(["'traffic_gui.config' if '", LaunchConfiguration('scenario'), "' == 'traffic' else 'gui.config'"])] )],
            condition=IfCondition(LaunchConfiguration('gazebo')), output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/world/cosim_demo/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             remappings=[('/world/cosim_demo/clock', '/gazebo/clock')],
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
                     'control_mode': LaunchConfiguration('control_mode'),
                     'scenario': LaunchConfiguration('scenario'),
                     'show_background_labels': ParameterValue(LaunchConfiguration('show_background_labels'), value_type=bool),
                     'network_gui': ParameterValue(LaunchConfiguration('network_gui'), value_type=bool),
                     'communication_range_m': ParameterValue(LaunchConfiguration('communication_range_m'), value_type=float),
                     'project_root': LaunchConfiguration('project_root'),
                     'sumo_gui': ParameterValue(LaunchConfiguration('sumo_gui'), value_type=bool),
                     'duration': ParameterValue(LaunchConfiguration('duration'), value_type=float),
                     'playback_rate': ParameterValue(LaunchConfiguration('playback_rate'), value_type=float),
                 }]),
        ]),
    ])
