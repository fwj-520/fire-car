#!/usr/bin/env python3
"""
路径跟踪节点 - 沿最优路径原路返回
功能：
1. 订阅最优路径和小车当前位姿
2. 逐个读取路径点，计算转向角度和前进速度
3. 输出 /cmd_vel 控制小车沿路径返回
4. 优先避障，其次按路径走
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import tf2_ros
import math

# 导航参数
TARGET_DISTANCE_THRESHOLD = 0.3  # 到达目标点的距离阈值
ANGLE_TOLERANCE = math.radians(10)  # 角度容忍度
MAX_LINEAR_SPEED = 0.3  # 最大前进速度
MAX_ANGULAR_SPEED = 0.5  # 最大角速度

class PathTrackerNode(Node):
    def __init__(self):
        super().__init__('path_tracker')

        self.get_logger().info('🚗 路径跟踪节点已启动')

        # 当前目标点索引
        self.target_index = 0

        # 最优路径
        self.optimal_path = []

        # 导航状态
        self.is_navigating = False

        # TF2 监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 订阅最优路径
        self.optimal_path_sub = self.create_subscription(
            PoseStamped, '/optimal_path', self.optimal_path_callback, 10)

        # 订阅优化完成信号
        self.optimization_done_sub = self.create_subscription(
            Bool, '/optimization_done', self.optimization_done_callback, 10)

        # 发布控制指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 定时器用于导航控制
        self.timer = self.create_timer(0.1, self.navigate)

        # 起点坐标（用于判断任务结束）
        self.start_point = None

    def optimal_path_callback(self, msg):
        """接收最优路径点"""
        x = msg.pose.position.x
        y = msg.pose.position.y

        # 检查是否是路径结束标记
        if math.isnan(x) or math.isnan(y):
            self.get_logger().info(f'✅ 最优路径接收完成 - 共 {len(self.optimal_path)} 个点')
            if len(self.optimal_path) > 0:
                self.start_point = self.optimal_path[-1]  # 最后一个点是起点
        else:
            self.optimal_path.append((x, y))
            self.get_logger().debug(
                f'📥 收到最优路径点: ({x:.2f}, {y:.2f}) - 总点数: {len(self.optimal_path)}')

    def optimization_done_callback(self, msg):
        """优化完成，开始导航"""
        if msg.data and not self.is_navigating and len(self.optimal_path) > 0:
            self.get_logger().info('🧭 开始沿最优路径返航')
            self.is_navigating = True

    def navigate(self):
        """执行导航逻辑"""
        if not self.is_navigating or len(self.optimal_path) == 0:
            return

        # 获取小车当前位置
        robot_x, robot_y, robot_yaw = self.get_robot_position()
        if robot_x is None or robot_y is None:
            return

        # 检查是否到达终点（起点）
        if self.is_arrived_at_start(robot_x, robot_y):
            self.stop_robot()
            self.is_navigating = False
            self.get_logger().info('🏁 已返回起点，任务结束')
            return

        # 获取当前目标点
        if self.target_index >= len(self.optimal_path):
            self.stop_robot()
            self.is_navigating = False
            self.get_logger().info('🏁 路径跟踪完成')
            return

        target_x, target_y = self.optimal_path[self.target_index]

        # 计算到目标点的距离
        distance = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

        # 计算到目标点的角度
        target_angle = math.atan2(target_y - robot_y, target_x - robot_x)

        # 计算转向角度
        angle_diff = target_angle - robot_yaw

        # 归一化到 [-π, π] 范围
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # 控制逻辑
        cmd = Twist()

        # 如果距离目标点较近，切换到下一个目标点
        if distance < TARGET_DISTANCE_THRESHOLD:
            self.target_index += 1
            self.get_logger().info(f'📌 到达目标点 {self.target_index}')
            return

        # 转向控制
        if abs(angle_diff) > ANGLE_TOLERANCE:
            cmd.angular.z = math.copysign(MAX_ANGULAR_SPEED, angle_diff)
            cmd.linear.x = 0.0
        else:
            cmd.angular.z = 0.0
            cmd.linear.x = MAX_LINEAR_SPEED

        # 发布控制指令
        self.cmd_vel_pub.publish(cmd)

    def get_robot_position(self):
        """通过 TF2 获取机器人在 map 坐标系的位置"""
        try:
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
            try:
                self.get_logger().debug(f"无法获取TF变换: {e}")
            except:
                pass
            return None, None, None

    def is_arrived_at_start(self, x, y):
        """检查是否到达起点"""
        if self.start_point is None:
            return False

        distance = math.sqrt((x - self.start_point[0])**2 + (y - self.start_point[1])**2)
        return distance < 0.3

    def stop_robot(self):
        """停止小车"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = PathTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            try:
                node.get_logger().info('🛑 节点已停止')
                node.stop_robot()
            except:
                pass
    except Exception as e:
        if node:
            try:
                node.get_logger().error(f'节点异常: {e}')
                node.stop_robot()
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
