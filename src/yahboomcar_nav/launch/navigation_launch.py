from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_nav')
    config_dir = os.path.join(package_path, 'config')
    map_dir = os.path.join(package_path, 'maps')

    return LaunchDescription([
        # 地图服务器启动
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': os.path.join(map_dir, 'map.yaml')}]
        ),

        # 定位启动（AMCL）
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(config_dir, 'navigation.yaml')]
        ),

        # 控制器服务器启动
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'navigation.yaml')]
        ),

        # 规划器服务器启动
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'navigation.yaml')]
        ),

        # 行为树导航器启动
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(config_dir, 'navigation.yaml')]
        ),

        # 生命周期管理器启动
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': False},
                       {'autostart': True},
                       {'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator']}]
        ),

        # RViz可视化启动
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_path, 'rviz', 'map_view.rviz')],
            output='screen'
        )
    ])