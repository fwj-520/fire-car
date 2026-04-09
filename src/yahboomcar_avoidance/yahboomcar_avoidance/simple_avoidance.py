#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
import signal
import sys
import time

class SimpleAvoidance(Node):
    def __init__(self):
        super().__init__('simple_avoidance')

        laser_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=5
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, laser_qos)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 5)

        # 参数（为 SLAM 建图优化）
        self.safe_distance = 0.4    # 安全距离（调整到0.4米）
        self.danger_distance = 0.35 # 危险距离（调整到0.35米）
        self.clear_distance = 0.7   # 清除距离（调整到0.7米）
        self.max_speed = 0.55       # 最大速度（保持0.55 m/s）
        self.turn_speed = 0.6       # 转向速度（提高到0.6 rad/s）
        self.back_speed = -0.4      # 后退速度

        self.laser_x_offset = 0.07
        self.laser_y_offset = 0.0075
        self.laser_z_offset = 0.024

        self.is_turning = False
        self.turn_direction = 0

        self.scan_received = False
        self.last_cmd = Twist()
        self.is_stop_command = False  # 新增停止命令标志

        self.create_timer(0.2, self.timer_callback)

        self.get_logger().info('='*50)
        self.get_logger().info('🚗 避障节点 (平滑转向+偏移补偿)')
        self.get_logger().info('='*50)

        # 注册信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.get_logger().info('🛑 收到 Ctrl+C，正在停止避障...')
        # 发送停止指令
        stop_cmd = Twist()
        for i in range(5):
            self.cmd_pub.publish(stop_cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('✅ 小车已停止')
        # 关闭节点
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def scan_callback(self, scan):
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info('✅ 成功接收激光数据')

        n = len(scan.ranges)

        # 定义检测区域（补偿雷达偏移后，实际是车体周围的空间）
        # 由于雷达偏右0.75cm，前方7cm，需要调整检测区域

        # 前方区域：-30° ~ 30°（雷达前方，但因为雷达在车前7cm，需要适当调整）
        front_start = int((math.radians(-30) - scan.angle_min) / scan.angle_increment)
        front_end = int((math.radians(30) - scan.angle_min) / scan.angle_increment)

        # 左前方：30° ~ 80°
        left_start = int((math.radians(30) - scan.angle_min) / scan.angle_increment)
        left_end = int((math.radians(80) - scan.angle_min) / scan.angle_increment)

        # 右前方：-80° ~ -30°
        right_start = int((math.radians(-80) - scan.angle_min) / scan.angle_increment)
        right_end = int((math.radians(-30) - scan.angle_min) / scan.angle_increment)

        # 确保索引有效
        front_start = max(0, min(front_start, n-1))
        front_end = max(0, min(front_end, n-1))
        left_start = max(0, min(left_start, n-1))
        left_end = max(0, min(left_end, n-1))
        right_start = max(0, min(right_start, n-1))
        right_end = max(0, min(right_end, n-1))

        def get_min_distance(start, end):
            if start >= end:
                return float('inf')
            distances = []
            for i in range(start, end):
                d = scan.ranges[i]
                if not math.isinf(d) and not math.isnan(d) and d > 0.1:
                    # 补偿雷达偏移：实际距离 = 测量距离 - 雷达突出距离
                    # 雷达在车前7cm，所以实际到车体的距离要减去7cm
                    actual_d = d - self.laser_x_offset
                    if actual_d > 0.1:
                        distances.append(actual_d)
            return min(distances) if distances else float('inf')

        front_dist = get_min_distance(front_start, front_end)
        left_dist = get_min_distance(left_start, left_end)
        right_dist = get_min_distance(right_start, right_end)

        # 决策逻辑 - 优化避障行为：后退到转弯过程更平滑，添加转弯间隔
        cmd = Twist()

        if front_dist < self.danger_distance:  # 危险距离
            if not self.is_turning:  # 第一次遇到危险，先后退
                cmd.linear.x = self.back_speed
                self.is_turning = True
                self.back_time = time.time()
                self.turn_direction = 1 if left_dist > right_dist else -1
                self.get_logger().info(f'🔥 开始后退 (前:{front_dist:.2f}m)')
            else:
                # 优化后的避障行为：简化逻辑，确保后退后正确转弯
                if time.time() - self.back_time < 0.6:  # 后退时间缩短到0.6秒
                    cmd.linear.x = self.back_speed
                    self.get_logger().info(f'🔥 继续后退 (前:{front_dist:.2f}m)')
                elif time.time() - self.back_time < 2.0:  # 直接原地转弯1.4秒
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.turn_speed * self.turn_direction
                    self.get_logger().info(f'🔄 原地转弯 (方向:{"左" if self.turn_direction == 1 else "右"}, 前:{front_dist:.2f}m)')
                else:  # 转弯完成
                    self.is_turning = False
                    self.get_logger().info(f'✅ 转弯完成，准备重新开始')

        else:  # 安全距离，直行
            self.is_turning = False
            if front_dist < self.safe_distance + 0.3:
                cmd.linear.x = 0.45  # 最低速度（满足硬件要求）
            else:
                cmd.linear.x = min(0.55, front_dist * 0.5)  # 速度与距离成正比
            self.get_logger().info(f'✅ 直行 (前:{front_dist:.2f}m, 速:{cmd.linear.x:.2f}m/s)')

        self.last_cmd = cmd

    def timer_callback(self):
        # 如果收到停止命令，发送停止指令
        if self.is_stop_command:
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            return

        self.cmd_pub.publish(self.last_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 正在停止...')
        stop_cmd = Twist()
        for i in range(5):
            node.cmd_pub.publish(stop_cmd)
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
