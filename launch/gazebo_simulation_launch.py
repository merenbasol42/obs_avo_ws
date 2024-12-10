import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # TurtleBot3 Gazebo launch dosyasını al
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # TurtleBot3 dünya launch dosyasını dahil et
    turtlebot3_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )
    
    # Kendi engel kaçınma düğümlerimizi başlat
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

    return LaunchDescription([
        turtlebot3_world_launch,
        lidar_node,
        obstacle_avoidance_node
    ])
