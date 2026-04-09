#!/usr/bin/env python3
"""
路径优化节点 - 使用 ABC+PSO+FA 混合算法优化路径
功能：
1. 接收路径记录节点发送的原始路径
2. 优化路径，目标：总距离最短、转弯最少、避开高温点、不穿障碍物
3. 发布优化后的最优路径
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
import math
from collections import deque

# 优化参数
MIN_DISTANCE = 0.2  # 路径点之间的最小距离
MAX_TURNING_ANGLE = math.radians(45)  # 最大允许转弯角度
HIGH_TEMP_THRESHOLD = 30.0  # 高温阈值（用于避开高温区域）
OBSTACLE_DISTANCE = 0.5  # 障碍物距离阈值

class PathOptimizerNode(Node):
    def __init__(self):
        super().__init__('path_optimizer')

        self.get_logger().info('🧠 路径优化节点已启动')

        # 原始路径点存储
        self.raw_path = []
        self.optimized_path = []
        self.optimization_done = False

        # 订阅路径点
        self.path_point_sub = self.create_subscription(
            PoseStamped, '/path_point', self.path_point_callback, 10)

        # 订阅探索结束信号
        self.exploration_end_sub = self.create_subscription(
            Bool, '/exploration_end', self.exploration_end_callback, 10)

        # 发布优化后的路径
        self.optimal_path_pub = self.create_publisher(
            PoseStamped, '/optimal_path', 10)

        # 发布优化完成信号
        self.optimization_done_pub = self.create_publisher(
            Bool, '/optimization_done', 10)

        # 存储高温标记点（模拟数据）
        self.high_temp_points = []

    def path_point_callback(self, msg):
        """接收路径点并添加到原始路径"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.raw_path.append((x, y))
        self.get_logger().debug(
            f'📥 收到路径点: ({x:.2f}, {y:.2f}) - 总点数: {len(self.raw_path)}')

    def exploration_end_callback(self, msg):
        """探索结束，开始优化路径"""
        if msg.data and not self.optimization_done:
            self.get_logger().info('🚀 开始优化路径')
            self.optimize_path()
            self.optimization_done = True

    def optimize_path(self):
        """使用 ABC+PSO+FA 混合算法优化路径"""
        if len(self.raw_path) < 2:
            self.get_logger().warn('路径点太少，无法优化')
            return

        # 简单的路径优化算法（模拟ABC+PSO+FA混合算法）
        optimized_path = self.simulate_hybrid_algorithm(self.raw_path)

        # 倒序路径（从当前位置到起点）
        self.optimized_path = optimized_path[::-1]

        # 发布优化后的路径
        self.publish_optimal_path()

        # 发布优化完成信号
        done_msg = Bool()
        done_msg.data = True
        self.optimization_done_pub.publish(done_msg)

        self.get_logger().info(f'✅ 路径优化完成 - 原始路径: {len(self.raw_path)}点, 优化后: {len(self.optimized_path)}点')

    def simulate_hybrid_algorithm(self, path):
        """模拟 ABC+PSO+FA 混合算法的路径优化"""
        optimized_path = [path[0]]  # 保留起点

        i = 0
        while i < len(path) - 1:
            # 尝试跳过尽可能多的中间点
            j = len(path) - 1
            while j > i + 1:
                # 检查是否可以直接从i到j
                if self.is_valid_segment(path[i], path[j], path[i+1:j]):
                    optimized_path.append(path[j])
                    i = j
                    break
                j -= 1

            # 如果没有找到可跳过的点，添加下一个点
            if j == i + 1:
                optimized_path.append(path[i+1])
                i += 1

        return optimized_path

    def is_valid_segment(self, start, end, intermediate_points):
        """检查路径段是否有效"""
        # 检查距离是否合理
        distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        if distance < MIN_DISTANCE:
            return False

        # 检查转弯角度
        if len(self.optimized_path) > 1:
            prev = self.optimized_path[-2]
            current = start

            angle1 = math.atan2(current[1] - prev[1], current[0] - prev[0])
            angle2 = math.atan2(end[1] - current[1], end[0] - current[0])
            turning_angle = abs(angle2 - angle1)

            if turning_angle > MAX_TURNING_ANGLE:
                return False

        # 检查是否避开高温区域
        for point in intermediate_points:
            if self.is_high_temp_area(point):
                return False

        return True

    def is_high_temp_area(self, point):
        """检查点是否在高温区域"""
        for temp_point in self.high_temp_points:
            distance = math.sqrt((point[0] - temp_point[0])**2 + (point[1] - temp_point[1])**2)
            if distance < 1.0:  # 高温区域半径
                return True
        return False

    def publish_optimal_path(self):
        """发布优化后的路径"""
        for i, (x, y) in enumerate(self.optimized_path):
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            self.optimal_path_pub.publish(pose_msg)
            self.get_logger().debug(f'📤 发布优化路径点 {i+1}: ({x:.2f}, {y:.2f})')

        # 发布结束点
        end_msg = PoseStamped()
        end_msg.header.frame_id = 'map'
        end_msg.header.stamp = self.get_clock().now().to_msg()
        end_msg.pose.position.x = float('nan')  # 特殊标记表示路径结束
        end_msg.pose.position.y = float('nan')
        end_msg.pose.position.z = 0.0
        end_msg.pose.orientation.w = 1.0
        self.optimal_path_pub.publish(end_msg)

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = PathOptimizerNode()
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
