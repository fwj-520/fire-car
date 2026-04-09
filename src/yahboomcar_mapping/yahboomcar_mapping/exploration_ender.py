#!/usr/bin/env python3
"""
探索结束触发节点 - 判断何时停止探索并开始返航
功能：
1. 检测是否找到火源
2. 判断是否需要停止探索
3. 发送探索结束信号
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math

# 触发参数
FIRE_DETECTION_RADIUS = 1.0  # 检测到火源的距离阈值
EXPLORATION_TIME_LIMIT = 300  # 探索时间限制（秒）
MAX_PATH_POINTS = 1000  # 路径点数量限制

class ExplorationEnderNode(Node):
    def __init__(self):
        super().__init__('exploration_ender')

        self.get_logger().info('👁️ 探索结束触发节点已启动')

        # 探索状态
        self.exploration_ended = False
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # 创建订阅者
        self.fire_pose_sub = self.create_subscription(
            PoseStamped, '/fire_source_pose', self.fire_pose_callback, 10)

        # 创建发布者
        self.exploration_end_pub = self.create_publisher(
            Bool, '/exploration_end', 10)

        # 定时器用于检查探索时间
        self.timer = self.create_timer(1.0, self.check_exploration_time)

        # 记录是否找到火源
        self.fire_found = False

        # 探索时间限制（从300秒缩短到30秒，便于测试）
        self.exploration_time_limit = 30

        # 探索进度跟踪
        self.loop_count = 0

    def fire_pose_callback(self, msg):
        """收到火源位置，判断是否已接近火源"""
        if self.exploration_ended:
            return

        # 获取火源位置
        fire_x = msg.pose.position.x
        fire_y = msg.pose.position.y

        # 简单判断：如果收到火源位置，认为已找到火源
        self.fire_found = True
        self.get_logger().info(f'🔥 检测到火源位置: ({fire_x:.2f}, {fire_y:.2f})')

        # 发送探索结束信号
        self.send_exploration_end_signal()

    def check_exploration_time(self):
        """检查探索时间是否超过限制"""
        if self.exploration_ended:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time

        # 打印探索进度信息（每分钟一次）
        self.loop_count += 1
        if self.loop_count % 60 == 0:
            self.get_logger().info(f'⏱️  探索已持续 {elapsed_time:.0f} 秒')

        if elapsed_time > self.exploration_time_limit:
            self.get_logger().info('⏰ 探索时间已超过限制')
            self.send_exploration_end_signal()

    def send_exploration_end_signal(self):
        """发送探索结束信号"""
        if not self.exploration_ended:
            self.exploration_ended = True
            msg = Bool()
            msg.data = True
            self.exploration_end_pub.publish(msg)
            self.get_logger().info('🚩 探索结束，开始返航')

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ExplorationEnderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            try:
                node.get_logger().info('🛑 节点已停止')
            except:
                pass
    except Exception as e:
        if node:
            try:
                node.get_logger().error(f'节点异常: {e}')
            except:
                print(f'节点异常: {e}')
    finally:
        if node:
            try:
                node.destroy_node()
            except:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
