from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('frame_id', default_value='laser'),
        DeclareLaunchArgument('output_topic_0', default_value='scan0'),
        DeclareLaunchArgument('output_topic_1', default_value='scan1'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('host_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('sensor_ip', default_value='192.168.198.2'),
        DeclareLaunchArgument('port_0', default_value='2368'),
        DeclareLaunchArgument('port_1', default_value='2369'),
        DeclareLaunchArgument('angle_offset', default_value='0'),
        DeclareLaunchArgument('scan_freq', default_value='30'),
        DeclareLaunchArgument('filter', default_value='3'),
        DeclareLaunchArgument('laser_enable', default_value='true'),
        DeclareLaunchArgument('scan_range_start', default_value='45'),
        DeclareLaunchArgument('scan_range_stop', default_value='315'),
        DeclareLaunchArgument('configure_sensor', default_value='false'),
    ]

    common_params = {
        'frame_id': LaunchConfiguration('frame_id'),
        'inverted': LaunchConfiguration('inverted'),
        'host_ip': LaunchConfiguration('host_ip'),
        'sensor_ip': LaunchConfiguration('sensor_ip'),
        'angle_offset': LaunchConfiguration('angle_offset'),
        'scan_freq': LaunchConfiguration('scan_freq'),
        'filter': LaunchConfiguration('filter'),
        'laser_enable': LaunchConfiguration('laser_enable'),
        'scan_range_start': LaunchConfiguration('scan_range_start'),
        'scan_range_stop': LaunchConfiguration('scan_range_stop'),
        'configure_sensor': LaunchConfiguration('configure_sensor'),
    }

    params_0 = dict(common_params)
    params_0['output_topic'] = LaunchConfiguration('output_topic_0')
    params_0['port'] = LaunchConfiguration('port_0')

    params_1 = dict(common_params)
    params_1['output_topic'] = LaunchConfiguration('output_topic_1')
    params_1['port'] = LaunchConfiguration('port_1')

    lidar_node_0 = Node(
        package='benewake_lidar',
        executable='benewake_lidar_scan_node',
        name='benewake_lidar_scan_0',
        parameters=[params_0],
        output='screen',
    )

    lidar_node_1 = Node(
        package='benewake_lidar',
        executable='benewake_lidar_scan_node',
        name='benewake_lidar_scan_1',
        parameters=[params_1],
        output='screen',
    )

    ld = LaunchDescription()
    for arg in args:
        ld.add_action(arg)
    ld.add_action(lidar_node_0)
    ld.add_action(lidar_node_1)
    return ld
