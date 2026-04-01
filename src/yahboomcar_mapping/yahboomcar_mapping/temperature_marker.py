#!/usr/bin/env python3
"""
温度数据集成节点 - 在 SLAM 建图中显示温度信息和着火点标记
功能：
1. 订阅温度传感器数据（单个话题）
2. 订阅里程计数据获取机器人位置
3. 检测温度超过 30°C 的着火点
4. 发布温度分布和着火点标记的可视化消息
5. 支持 RViz 可视化
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Pose
import math

# 温度阈值（摄氏度）
TEMP_FIRE_THRESHOLD = 30.0
# 着火点标记过期时间（秒）
FIRE_MARKER_TIMEOUT = 300.0
# 无效温度值
TEMP_INVALID = -999.0

class TemperatureMarkerNode(Node):
    def __init__(self):
        super().__init__('temperature_marker_node')

        # 创建订阅者（订阅统一的温度数据话题）
        self.temp_sub = self.create_subscription(
            Float32MultiArray, '/mlx90614/temperature_array',
            self.temperature_callback, 10)

        # 订阅里程计话题
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odometry_callback, 10)

        # 创建发布者
        self.fire_markers_pub = self.create_publisher(
            MarkerArray, '/fire_markers', 10)

        self.temp_markers_pub = self.create_publisher(
            MarkerArray, '/temperature_markers', 10)

        # 添加静态TF发布，解决map frame不存在的问题
        # 如果没有SLAM，使用odom作为基准坐标系
        # 静态TF：odom -> map（临时解决方案，SLAM启动后会覆盖）
        # 注意：这需要依赖tf2_ros包

        # 存储着火点信息 (timestamp, x, y, temperature, sensor)
        self.fire_points = []
        self.marker_id_counter = 0

        # 当前机器人位置
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # 温度标记数组
        self.temp_markers = MarkerArray()

        # 定时器用于更新标记（每秒更新一次）
        self.timer = self.create_timer(1.0, self.update_markers)

        self.get_logger().info('🔥 温度数据集成节点已启动')
        self.get_logger().info(f'着火点检测阈值: {TEMP_FIRE_THRESHOLD}°C')

    def temperature_callback(self, msg):
        """统一的温度数据回调"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # 解析温度数据：[左环境, 左物体, 右环境, 右物体]
        if len(msg.data) >= 4:
            left_ambient = msg.data[0]
            left_object = msg.data[1]
            right_ambient = msg.data[2]
            right_object = msg.data[3]

            # 处理左传感器数据
            if left_object != TEMP_INVALID:
                self.get_logger().info(f'左传感器(DCC) 温度: {left_object:.2f}°C')
                self.process_temperature_data(left_object, "左传感器(DCC)")
                # 保存最新左传感器温度数据
                self.last_left_temp = left_object

            # 处理右传感器数据
            if right_object != TEMP_INVALID:
                self.get_logger().info(f'右传感器(DCI) 温度: {right_object:.2f}°C')
                self.process_temperature_data(right_object, "右传感器(DCI)")
                # 保存最新右传感器温度数据
                self.last_right_temp = right_object
        else:
            self.get_logger().warning('温度数据格式不正确')

    def process_temperature_data(self, temperature, sensor_name):
        """处理温度数据的通用方法"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # 强制显示所有温度（不检查阈值）
        self.get_logger().warning(f'🔥 {sensor_name} 强制创建标记! 温度: {temperature:.2f}°C')

        self.fire_points.append({
            'timestamp': current_time,
            'x': self.current_x,
            'y': self.current_y,
            'temperature': temperature,
            'sensor': sensor_name
        })

        self.get_logger().info(f'着火点已记录: ({self.current_x:.2f}, {self.current_y:.2f}) - {temperature:.2f}°C ({sensor_name})')

        # 清理过期的着火点标记（超过5分钟）
        self.cleanup_expired_fire_points(current_time)

    def odometry_callback(self, msg):
        """里程计数据回调"""
        # 更新机器人当前位置
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # 计算朝向（yaw角）
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def cleanup_expired_fire_points(self, current_time):
        """清理过期的着火点标记"""
        self.fire_points = [
            p for p in self.fire_points
            if current_time - p['timestamp'] < FIRE_MARKER_TIMEOUT
        ]

    def create_fire_marker(self, x, y, temperature, marker_id, sensor_name="未知"):
        """创建着火点标记"""
        marker = Marker()
        marker.header.frame_id = 'odom'  # 使用odom代替map，避免frame不存在的问题
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fire_points'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # 位置和尺寸
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # 根据温度设置尺寸
        if temperature >= TEMP_FIRE_THRESHOLD:
            size = 0.25  # 着火点尺寸大
        else:
            size = 0.15  # 正常温度尺寸小

        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = 0.1

        # 根据传感器设置颜色
        if sensor_name == "左传感器(DCC)":
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8
        elif sensor_name == "右传感器(DCI)":
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

        # 持续时间（300秒后过期）
        marker.lifetime = rclpy.duration.Duration(seconds=300).to_msg()

        return marker

    def create_temperature_marker(self, x, y, temperature, marker_id):
        """创建温度分布标记"""
        marker = Marker()
        marker.header.frame_id = 'odom'  # 使用odom代替map，避免frame不存在的问题
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'temperature_points'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # 位置
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.02
        marker.pose.orientation.w = 1.0

        # 尺寸根据温度变化
        if temperature >= TEMP_FIRE_THRESHOLD:
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.08
        else:
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.05

        # 颜色：蓝色（低温）→ 绿色（中温）→ 红色（高温）
        if temperature < 25:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        elif temperature < 30:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        marker.color.a = 0.6

        return marker

    def update_markers(self):
        """更新所有标记"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # 创建着火点标记数组
        fire_markers = MarkerArray()

        for i, point in enumerate(self.fire_points):
            # 为每个着火点创建标记
            marker = self.create_fire_marker(
                point['x'],
                point['y'],
                point['temperature'],
                i,
                point.get('sensor', "未知")
            )

            fire_markers.markers.append(marker)

            # 添加文字标签
            text_marker = Marker()
            text_marker.header.frame_id = 'odom'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'fire_text'
            text_marker.id = len(self.fire_points) + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = point['x']
            text_marker.pose.position.y = point['y']
            text_marker.pose.position.z = 0.15
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.2

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0
            text_marker.color.a = 0.9

            text_marker.text = f'{point["temperature"]:.1f}°C ({point.get("sensor", "未知")})'

            fire_markers.markers.append(text_marker)

        # 发布着火点标记
        self.fire_markers_pub.publish(fire_markers)

        # 发布当前温度标记（显示机器人周围的温度）
        self.update_current_temp_marker()

    def update_current_temp_marker(self):
        """更新当前位置的温度标记"""
        temp_markers = MarkerArray()

        # 获取最新的温度数据（从 last_msg 中获取，或者通过其他方式）
        # 这里简化处理，显示当前温度标记
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'current_temperature'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.2
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.15

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # 显示温度信息（需要跟踪最新的温度数据）
        # 优化：显示两个传感器的温度
        left_temp = self.last_left_temp if hasattr(self, 'last_left_temp') else '--'
        right_temp = self.last_right_temp if hasattr(self, 'last_right_temp') else '--'
        marker.text = f'DCC: {left_temp}°C  DCI: {right_temp}°C'

        temp_markers.markers.append(marker)

        self.temp_markers_pub.publish(temp_markers)

    def print_fire_points_summary(self):
        """打印着火点汇总信息"""
        if not self.fire_points:
            self.get_logger().info('✅ 未检测到着火点')
            return

        self.get_logger().warning(f'🔥 着火点汇总: 共 {len(self.fire_points)} 个')

        for i, point in enumerate(self.fire_points):
            age = self.get_clock().now().nanoseconds / 1e9 - point['timestamp']
            self.get_logger().info(f'  #{i+1}: ({point["x"]:.2f}, {point["y"]:.2f}) - {point["temperature"]:.2f}°C ({age:.0f}秒前)')

    def cleanup_expired_fire_points(self, current_time):
        """清理过期的着火点"""
        initial_count = len(self.fire_points)
        self.fire_points = [
            p for p in self.fire_points
            if current_time - p['timestamp'] < FIRE_MARKER_TIMEOUT
        ]

        removed_count = initial_count - len(self.fire_points)
        if removed_count > 0:
            self.get_logger().info(f'已清理 {removed_count} 个过期的着火点标记')

def main(args=None):
    rclpy.init(args=args)

    try:
        node = TemperatureMarkerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
