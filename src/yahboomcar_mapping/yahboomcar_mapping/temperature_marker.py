#!/usr/bin/env python3
"""
温度数据集成节点 - 在 SLAM 建图中显示着火点标记
功能：
1. 订阅温度传感器数据
2. 通过 TF2 获取机器人在 map 坐标系的位置
3. 检测温度超过 30°C 的着火点
4. 发布着火点标记的可视化消息
5. 支持 RViz 可视化
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

# 温度阈值（摄氏度）
TEMP_FIRE_THRESHOLD = 25.0
# 着火点标记过期时间（秒）
FIRE_MARKER_TIMEOUT = 300.0

class TemperatureMarkerNode(Node):
    def __init__(self):
        super().__init__('temperature_marker')

        self.get_logger().info('🔥 温度标记节点已启动')
        self.get_logger().info(f'着火点检测阈值: {TEMP_FIRE_THRESHOLD}°C')

        # 创建订阅者
        self.temp_sub = self.create_subscription(
            Float32MultiArray, '/mlx90614/temperature_array',
            self.temperature_callback, 10)

        # 创建发布者
        self.fire_markers_pub = self.create_publisher(
            MarkerArray, '/fire_markers', 10)

        # TF2 监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 存储着火点信息 (timestamp, x, y, temperature)
        self.fire_points = []
        self.marker_id_counter = 0

        # 定时器用于更新标记（提高频率到10Hz）
        self.timer = self.create_timer(0.1, self.update_markers)

    def get_robot_position(self):
        """通过 TF2 获取机器人在 map 坐标系的位置"""
        try:
            # 获取 base_link 到 map 的坐标变换
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # 计算朝向（yaw角）
            orientation = transform.transform.rotation
            siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            return x, y, theta
        except Exception as e:
            self.get_logger().debug(f"无法获取TF变换: {e}")
            return None, None, None

    def temperature_callback(self, msg):
        """统一的温度数据回调"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # 解析温度数据：[左环境, 左物体, 右环境, 右物体]
        if len(msg.data) >= 4:
            left_ambient = msg.data[0]
            left_object = msg.data[1]
            right_ambient = msg.data[2]
            right_object = msg.data[3]

            # 获取当前机器人位置
            robot_x, robot_y, robot_theta = self.get_robot_position()

            if robot_x is not None and robot_y is not None:
                # 处理左传感器数据（物体温度）
                if left_object >= TEMP_FIRE_THRESHOLD:
                    self.fire_points.append({
                        'timestamp': current_time,
                        'x': robot_x,
                        'y': robot_y,
                        'temperature': left_object,
                        'sensor': "左传感器(DCC)"
                    })
                    self.get_logger().info(
                        f'🔥 左传感器检测到着火点: '
                        f'({robot_x:.2f}, {robot_y:.2f}) - {left_object:.2f}°C'
                    )

                # 处理右传感器数据（物体温度）
                if right_object >= TEMP_FIRE_THRESHOLD:
                    self.fire_points.append({
                        'timestamp': current_time,
                        'x': robot_x,
                        'y': robot_y,
                        'temperature': right_object,
                        'sensor': "右传感器(DCI)"
                    })
                    self.get_logger().info(
                        f'🔥 右传感器检测到着火点: '
                        f'({robot_x:.2f}, {robot_y:.2f}) - {right_object:.2f}°C'
                    )

        # 清理过期的着火点标记（超过5分钟）
        self.cleanup_expired_fire_points(current_time)

    def cleanup_expired_fire_points(self, current_time):
        """清理过期的着火点标记"""
        self.fire_points = [
            p for p in self.fire_points
            if current_time - p['timestamp'] < FIRE_MARKER_TIMEOUT
        ]

    def create_fire_marker(self, x, y, temperature, marker_id, sensor_name="未知"):
        """创建着火点标记"""
        marker = Marker()
        marker.header.frame_id = 'map'
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
        marker.scale.x = 0.3
        marker.scale.y = 0.3
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

        # 发布着火点标记
        self.fire_markers_pub.publish(fire_markers)

        # 清理过期的标记
        self.cleanup_expired_fire_points(current_time)

    def print_fire_points_summary(self):
        """打印着火点汇总信息"""
        if not self.fire_points:
            self.get_logger().info('✅ 未检测到着火点')
            return

        self.get_logger().warning(f'🔥 着火点汇总: 共 {len(self.fire_points)} 个')

        for i, point in enumerate(self.fire_points):
            age = self.get_clock().now().nanoseconds / 1e9 - point['timestamp']
            self.get_logger().info(
                f'  #{i+1}: ({point["x"]:.2f}, {point["y"]:.2f}) '
                f'- {point["temperature"]:.2f}°C ({age:.0f}秒前)'
            )

def main(args=None):
    rclpy.init(args=args)

    try:
        node = TemperatureMarkerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 节点已停止')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
