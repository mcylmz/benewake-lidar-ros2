from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('frame_id', default_value='laser'),
        DeclareLaunchArgument('output_topic', default_value='scan'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('host_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('sensor_ip', default_value='192.168.198.2'),
        DeclareLaunchArgument('port', default_value='2368'),
        DeclareLaunchArgument('angle_offset', default_value='0'),
        DeclareLaunchArgument('scan_freq', default_value='30'),
        DeclareLaunchArgument('filter', default_value='3'),
        DeclareLaunchArgument('laser_enable', default_value='true'),
        DeclareLaunchArgument('scan_range_start', default_value='45'),
        DeclareLaunchArgument('scan_range_stop', default_value='315'),
        DeclareLaunchArgument('configure_sensor', default_value='false'),
    ]

    lidar_node = Node(
        package='benewake_lidar',
        executable='benewake_lidar_scan_node',
        name='benewake_lidar_scan',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'output_topic': LaunchConfiguration('output_topic'),
            'inverted': LaunchConfiguration('inverted'),
            'host_ip': LaunchConfiguration('host_ip'),
            'sensor_ip': LaunchConfiguration('sensor_ip'),
            'port': LaunchConfiguration('port'),
            'angle_offset': LaunchConfiguration('angle_offset'),
            'scan_freq': ParameterValue(LaunchConfiguration('scan_freq'), value_type=str),
            'filter': ParameterValue(LaunchConfiguration('filter'), value_type=str),
            'laser_enable': ParameterValue(LaunchConfiguration('laser_enable'), value_type=str),
            'scan_range_start': ParameterValue(LaunchConfiguration('scan_range_start'), value_type=str),
            'scan_range_stop': ParameterValue(LaunchConfiguration('scan_range_stop'), value_type=str),
            'configure_sensor': LaunchConfiguration('configure_sensor'),
        }],
        output='screen',
    )

    ld = LaunchDescription()
    for arg in args:
        ld.add_action(arg)
    ld.add_action(lidar_node)
    return ld
