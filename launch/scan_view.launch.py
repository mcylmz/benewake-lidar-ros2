import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    lidar_params = {
        'frame_id': LaunchConfiguration('frame_id'),
        'output_topic': LaunchConfiguration('output_topic'),
        'inverted': LaunchConfiguration('inverted'),
        'host_ip': LaunchConfiguration('host_ip'),
        'sensor_ip': LaunchConfiguration('sensor_ip'),
        'port': LaunchConfiguration('port'),
        'angle_offset': LaunchConfiguration('angle_offset'),
        'scan_freq': LaunchConfiguration('scan_freq'),
        'filter': LaunchConfiguration('filter'),
        'laser_enable': LaunchConfiguration('laser_enable'),
        'scan_range_start': LaunchConfiguration('scan_range_start'),
        'scan_range_stop': LaunchConfiguration('scan_range_stop'),
        'configure_sensor': LaunchConfiguration('configure_sensor'),
    }

    lidar_node = Node(
        package='benewake_lidar',
        executable='benewake_lidar_scan_node',
        name='benewake_lidar_scan',
        parameters=[lidar_params],
        output='screen',
    )

    pkg_share = get_package_share_directory('benewake_lidar')
    rviz_config = os.path.join(pkg_share, 'rviz', 'benewake_lidar_scan.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    ld = LaunchDescription()
    for arg in args:
        ld.add_action(arg)
    ld.add_action(lidar_node)
    ld.add_action(rviz_node)
    return ld
