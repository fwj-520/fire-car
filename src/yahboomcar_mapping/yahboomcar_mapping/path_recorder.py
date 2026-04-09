#!/usr/bin/env python3
"""
路径记录节点 - 记录小车走过的所有路径点
功能：
1. 定时通过 TF2 读取小车在 map 坐标系下的 (x,y)
2. 把坐标按顺序存到一个列表里（路径点序列）
3. 从启动开始就一直存，直到找到火源或判定探索结束
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math

class PathRecorderNode(Node):
    def __init__(self):
        super().__init__('path_recorder')

        self.get_logger().info('📝 路径记录节点已启动')

        # 创建 TF2 监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 路径点列表
        self.path_points = []
        self.recording = True

        # 定时器用于读取路径点（每秒2次）
        self.timer = self.create_timer(0.5, self.read_path_point)

        # 订阅探索结束信号
        self.exploration_end_sub = self.create_subscription(
            Bool, '/exploration_end', self.exploration_end_callback, 10)

        # 发布路径点话题
        self.path_pub = self.create_publisher(
            PoseStamped, '/path_point', 10)

        # 最小距离阈值，用于避免记录过于接近的点
        self.min_distance = 0.2

    def read_path_point(self):
        """定时读取小车位置并记录"""
        if not self.recording:
            return

        try:
            # 获取机器人在 map 坐标系的位置
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # 检查是否需要记录这个点
            if self.should_record_point(x, y):
                self.path_points.append((x, y))
                self.get_logger().debug(
                    f'📌 记录路径点: ({x:.2f}, {y:.2f}) - 总点数: {len(self.path_points)}')

                # 发布路径点
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = 'map'
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = 0.0
                pose_msg.pose.orientation.w = 1.0
                self.path_pub.publish(pose_msg)

        except Exception as e:
            try:
                self.get_logger().debug(f"无法获取TF变换: {e}")
            except:
                pass

    def should_record_point(self, x, y):
        """判断是否需要记录这个点"""
        if len(self.path_points) == 0:
            return True

        # 计算与最后一个点的距离
        last_x, last_y = self.path_points[-1]
        distance = math.sqrt((x - last_x)**2 + (y - last_y)**2)

        return distance > self.min_distance

    def exploration_end_callback(self, msg):
        """探索结束回调函数"""
        if msg.data and self.recording:
            self.recording = False
            self.get_logger().info(
                f'🛑 探索结束，停止记录路径。总记录点数: {len(self.path_points)}')

    def get_full_path(self):
        """返回完整路径"""
        return self.path_points

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = PathRecorderNode()
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
        # 确保先销毁节点
        if node:
            try:
                node.destroy_node()
            except:
                pass
        # 检查 rclpy 是否还在运行，避免双重 shutdown 错误
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
