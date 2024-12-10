import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Paket dizinini al
    pkg_dir = get_package_share_directory('obstacle_avoidance')

    # Düğümleri tanımla
    lidar_node = Node(
        package='obstacle_avoidance',
        executable='lidar_node',
        name='lidar_node',
        output='screen'
    )

    obstacle_avoidance_node = Node(
        package='obstacle_avoidance',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen'
    )

    # Launch Description oluştur
    return LaunchDescription([
        lidar_node,
        obstacle_avoidance_node
    ])
