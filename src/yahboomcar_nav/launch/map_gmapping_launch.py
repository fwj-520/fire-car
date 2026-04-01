from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 激光雷达和底盘启动
    laser_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_nav'), 
                         'launch', 'laser_bringup_gmapping_launch.py')
        ])
    )
    
    # gmapping建图启动
    slam_gmapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_gmapping'), 
                         'launch', 'slam_gmapping_launch.py')
        ])
    )
    
    # RViz可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('yahboomcar_nav'), 
                                       'rviz', 'map_view.rviz')],
        output='screen'
    )
    
    return LaunchDescription([
        laser_bringup,
        slam_gmapping,
        rviz_node
    ])
