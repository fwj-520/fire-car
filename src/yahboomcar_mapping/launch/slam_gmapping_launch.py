from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[{
                'use_sim_time': False,
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
                'srr': 0.1,
                'srt': 0.2,
                'str': 0.1,
                'stt': 0.2,
                'xmin': -15.0,
                'ymin': -15.0,
                'xmax': 15.0,
                'ymax': 15.0
            }],
            remappings=[('/scan', '/scan')]
        )
    ])
