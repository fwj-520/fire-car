from launch import LaunchDescription
from launch_ros.actions import Node

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
        
        # 静态TF：base_link -> laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            name='base_to_laser'
        ),
        
        # 静态TF：odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            name='odom_to_base'
        ),
        
        # slam_gmapping
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[{
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'delta': 0.05,
                'maxUrange': 5.0,
                'maxRange': 6.0,
                'minimumScore': 50,
                'map_update_interval': 1.0,
                'particles': 30,
                'linearUpdate': 0.2,
                'angularUpdate': 0.2
            }]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
