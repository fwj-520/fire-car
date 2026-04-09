#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import time

class TestFireSourceNode(Node):
    def __init__(self):
        super().__init__('test_fire_source')

        # 创建订阅者，订阅火源定位节点发布的信息
        self.pose_sub = self.create_subscription(
            PoseStamped, '/fire_source_pose',
            self.pose_callback, 10)

        # 创建发布者，模拟温度传感器发布的信息
        self.temp_pub = self.create_publisher(
            Float32MultiArray, '/mlx90614/temperature_array', 10)

        self.get_logger().info('测试节点已启动')

        # 发送模拟温度数据
        self.send_test_data()

    def pose_callback(self, msg):
        self.get_logger().info(f'收到火源位置: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def send_test_data(self):
        # 模拟发送多个高温区域的温度数据
        import random

        # 随机选择一个温度模式
        mode = random.randint(0, 2)

        if mode == 0:
            # 左传感器检测到异常温度（45°C），右传感器正常（25°C）
            msg = Float32MultiArray()
            msg.data = [25.0, 45.0, 25.0, 25.0]  # [左环境, 左物体, 右环境, 右物体]
            self.temp_pub.publish(msg)
            self.get_logger().info('已发送模拟温度数据: 左物体温度 = 45°C, 右物体温度 = 25°C')
        elif mode == 1:
            # 右传感器检测到异常温度（45°C），左传感器正常（25°C）
            msg = Float32MultiArray()
            msg.data = [25.0, 25.0, 25.0, 45.0]  # [左环境, 左物体, 右环境, 右物体]
            self.temp_pub.publish(msg)
            self.get_logger().info('已发送模拟温度数据: 左物体温度 = 25°C, 右物体温度 = 45°C')
        elif mode == 2:
            # 两个传感器都检测到异常温度（45°C）
            msg = Float32MultiArray()
            msg.data = [25.0, 45.0, 25.0, 45.0]  # [左环境, 左物体, 右环境, 右物体]
            self.temp_pub.publish(msg)
            self.get_logger().info('已发送模拟温度数据: 左物体温度 = 45°C, 右物体温度 = 45°C')

        # 定时器，用于定时发送数据
        self.timer = self.create_timer(3.0, self.send_test_data)

def main():
    rclpy.init()

    try:
        node = TestFireSourceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
