from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # 获取工作区路径
    ws_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

    return LaunchDescription([
        # 温度传感器驱动节点
        Node(
            package='mlx90614_driver',
            executable='mlx90614_node',
            name='mlx90614_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB2',
                'baudrate': 115200,
                'publish_rate': 2.0
            }]
        ),

        # 温度可视化标记节点
        Node(
            package='yahboomcar_mapping',
            executable='temperature_marker',
            name='temperature_marker_node',
            output='screen'
        ),

        # 激光雷达（如果需要SLAM）
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

        # 静态TF转换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            name='base_to_laser'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            name='odom_to_base'
        ),

        # 启动RViz（自动加载温度配置）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                os.path.join(ws_path, 'src', 'yahboomcar_mapping', 'rviz', 'temperature.rviz')
            ],
            output='screen'
        )
    ])
