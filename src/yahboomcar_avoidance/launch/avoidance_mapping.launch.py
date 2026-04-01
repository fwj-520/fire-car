from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数文件路径
    ydlidar_params = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'params',
        'X2.yaml'
    )
    
    # RViz配置文件路径
    rviz_config = os.path.join(
        get_package_share_directory('yahboomcar_avoidance'),
        'rviz',
        'mapping.rviz'
    )
    
    return LaunchDescription([
        # 1. 激光雷达
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[ydlidar_params]
        ),
        
        # 2. STM32电机驱动
        Node(
            package='stm32_driver',
            executable='stm32_motor_node',
            name='hiwonder_motor_driver',
            parameters=[{'serial_port': '/dev/ttyUSB1'}],
            output='screen'
        ),
        
        # 3. 避障+建图融合节点（延迟2秒启动）
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='yahboomcar_avoidance',
                    executable='avoidance_mapping',
                    name='avoidance_mapping',
                    output='screen'
                )
            ]
        ),
        
        # 4. RViz可视化（延迟3秒启动）
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen'
                )
            ]
        )
    ])
