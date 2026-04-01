from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('stm32_driver'),
        'config',
        'stm32_motor.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='stm32_driver',
            executable='stm32_motor_node',
            name='hiwonder_motor_driver',
            output='screen',
            parameters=[config]
        )
    ])
