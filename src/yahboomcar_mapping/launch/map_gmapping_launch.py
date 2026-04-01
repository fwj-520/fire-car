from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 1. 激光雷达+底盘启动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('yahboomcar_mapping'), 
                'launch', 'laser_bringup_gmapping_launch.py')
            ])
        ),
        
        # 2. gmapping建图
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('yahboomcar_mapping'), 
                'launch', 'slam_gmapping_launch.py')
            ])
        ),
        
        # 3. RViz可视化
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('yahboomcar_mapping'), 
                'launch', 'rviz_launch.py')
            ])
        )
    ])
