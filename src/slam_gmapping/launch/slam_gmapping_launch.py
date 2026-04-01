from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time'),
        
        # slam_gmapping节点
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
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
                'angularUpdate': 0.2,
                'xmin': -15.0,
                'ymin': -15.0,
                'xmax': 15.0,
                'ymax': 15.0,
                'srr': 0.01,
                'srt': 0.02,
                'str': 0.01,
                'stt': 0.02
            }]
        )
    ])
