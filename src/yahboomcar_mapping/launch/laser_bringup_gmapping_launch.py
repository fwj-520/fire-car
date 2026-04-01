from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 1. 激光雷达驱动
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'baudrate': 115200,
                'angle_min': -2.0,
                'angle_max': 2.0,
                'range_min': 0.1,
                'range_max': 12.0,
                'frequency': 10.0,
                'ignore_array': ''
            }]
        ),
        
        # 2. 电机驱动 (提供odom -> base_link)
        Node(
            package='stm32_driver',
            executable='stm32_motor_node',
            name='hiwonder_motor_driver',
            parameters=[{'serial_port': '/dev/ttyUSB1'}]
        ),
        
        # 3. 静态TF：base_link -> laser_frame (补偿左偏90度)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '1.57', '0', '0', 'base_link', 'laser_frame'],
            name='base_link_to_laser'
        ),
    ])
