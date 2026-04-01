from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 激光雷达启动
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'baudrate': 115200,
                'angle_min': -180.0,
                'angle_max': 180.0,
                'range_min': 0.1,
                'range_max': 12.0,
                'frequency': 10.0
            }]
        ),
        
        # 电机驱动启动
        Node(
            package='stm32_driver',
            executable='stm32_motor_node',
            name='hiwonder_motor_driver',
            parameters=[{'serial_port': '/dev/ttyUSB1'}],
            output='screen'
        ),
        
        # 静态TF：base_link -> laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            name='static_tf_publisher'
        ),
        
        # 静态TF：odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            name='odom_to_base_link'
        ),
    ])
