from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 激光雷达
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'baudrate': 115200,
                'frequency': 10.0
            }]
        ),
        
        # 静态TF：odom -> base_link（无偏移）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            name='odom_to_base'
        ),
        
        # 静态TF：base_link -> laser_frame（添加雷达偏移补偿）
        # 参数：x y z yaw pitch roll parent child
        # x=0.07 (前方7cm), y=0.0075 (右侧0.75cm), z=0.024 (上方2.4cm)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.07', '0.0075', '0.024', '0', '0', '0', 'base_link', 'laser_frame'],
            name='base_to_laser'
        ),
        
        # slam_toolbox 建图
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'max_laser_range': 12.0
            }]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
