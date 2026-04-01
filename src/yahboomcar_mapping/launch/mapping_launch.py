from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('yahboomcar_mapping')
    rviz_config = os.path.join(pkg_share, 'rviz', 'mapping.rviz')
    
    return LaunchDescription([
        # 1. 静态TF：odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            name='odom_to_base_link'
        ),
        
        # 2. 静态TF：base_link -> laser_frame (补偿左偏90度)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '1.57', '0', '0', 'base_link', 'laser_frame'],
            name='base_link_to_laser'
        ),
        
        # 3. slam_gmapping建图 - 优化参数使建图更平滑
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
                'maxUrange': 4.0,              # 减小最大使用距离，减少噪声
                'maxRange': 5.0,                 # 减小最大激光距离
                'minimumScore': 100,              # 提高匹配分数要求
                'map_update_interval': 1.5,       # 降低地图更新频率
                'particles': 40,                   # 增加粒子数提高稳定性
                'linearUpdate': 0.1,               # 移动0.1米更新（更频繁）
                'angularUpdate': 0.05,             # 转动0.05弧度更新（更频繁）
                'srr': 0.05,                        # 降低旋转误差
                'srt': 0.1,                         # 降低旋转平移误差
                'str': 0.05,                        # 降低平移旋转误差
                'stt': 0.1,                         # 降低平移误差
                'xmin': -15.0,
                'ymin': -15.0,
                'xmax': 15.0,
                'ymax': 15.0
            }]
        ),
        
        # 4. 速度平滑器 - 使电机响应更平滑
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='speed_smoother',
            parameters=[{
                'scale_linear': 0.3,               # 降低最大线速度
                'scale_angular': 0.5,               # 降低最大角速度
                'scale_linear_turbo': 0.5,
                'scale_angular_turbo': 0.8,
            }]
        ),
        
        # 5. RViz可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
