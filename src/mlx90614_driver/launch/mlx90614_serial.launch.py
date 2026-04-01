from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mlx90614_driver'),
        'config',
        'mlx90614_serial.yaml'
    )

    return LaunchDescription([
        Node(
            package='mlx90614_driver',
            executable='mlx90614_node',
            name='mlx90614_node',
            output='screen',
            parameters=[config]
        )
    ])
