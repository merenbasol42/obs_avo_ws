from setuptools import find_packages, setup
import os

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyaları için
        (os.path.join('share', package_name, 'launch'), 
            [os.path.join('launch', 'obstacle_avoidance_launch.py')],
        ),
        (os.path.join('share', package_name, 'launch'), 
            [os.path.join('launch', 'gazebo_simulation_launch.py')]
        ),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ustad',
    maintainer_email='merenbasol@gmail.com',
    description='ROS2 Otonom Engel Tespiti ve Kaçınma Sistemi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = obstacle_avoidance.lidar_node:main',
            'obstacle_avoidance_node = obstacle_avoidance.obstacle_avoidance_node:main',
        ],
    },
)
