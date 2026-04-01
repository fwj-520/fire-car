#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Imu
import tf2_ros
import math
import signal
import sys
from tf_transformations import euler_from_quaternion

class SimpleOdom(Node):
    def __init__(self):
        super().__init__('simple_odom')

        # 订阅速度指令
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # 订阅 IMU 数据
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # 发布里程计
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 定时器 20Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        # 状态变量
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()

        # IMU 数据
        self.imu_theta = 0.0
        self.imu_gyro_z = 0.0
        self.imu_received = False

        self.get_logger().info('简易里程计已启动（速度积分+IMU辅助）')

        # 注册信号处理（优雅退出）
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        self.get_logger().info('收到停止信号，正在清理...')
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
        
    def imu_callback(self, msg):
        """处理 IMU 数据"""
        # 从四元数转换为欧拉角（偏航角）
        orientation = msg.orientation
        _, _, self.imu_theta = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.imu_gyro_z = msg.angular_velocity.z
        self.imu_received = True

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt > 0:
            if self.imu_received:
                # 使用 IMU 数据修正偏航角（减少累积误差）
                # 混合使用速度指令积分和 IMU 数据
                # 0.7 权重给 IMU，0.3 权重给速度指令积分
                imu_weight = 0.7

                if abs(self.w) < 0.01:
                    # 近似直行（小转弯）
                    self.x += self.v * dt * math.cos(self.theta)
                    self.y += self.v * dt * math.sin(self.theta)

                    # 使用 IMU 偏航角修正
                    delta_theta = self.imu_theta - self.theta
                    # 确保角度在 [-pi, pi] 范围内
                    if delta_theta > math.pi:
                        delta_theta -= 2 * math.pi
                    if delta_theta < -math.pi:
                        delta_theta += 2 * math.pi

                    self.theta += delta_theta * imu_weight
                    self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
                else:
                    # 转弯运动的精确积分
                    radius = self.v / self.w
                    theta0 = self.theta
                    delta_theta = self.w * dt
                    self.theta += delta_theta
                    self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
                    self.x += radius * (math.sin(self.theta) - math.sin(theta0))
                    self.y += radius * (math.cos(theta0) - math.cos(self.theta))

                    # 使用 IMU 偏航角进行微调
                    delta_theta = self.imu_theta - self.theta
                    if delta_theta > math.pi:
                        delta_theta -= 2 * math.pi
                    if delta_theta < -math.pi:
                        delta_theta += 2 * math.pi

                    self.theta += delta_theta * imu_weight * 0.5
                    self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            else:
                # 没有 IMU 数据时，使用原始的速度积分方法
                if abs(self.w) < 0.01:
                    self.x += self.v * dt * math.cos(self.theta)
                    self.y += self.v * dt * math.sin(self.theta)
                else:
                    radius = self.v / self.w
                    theta0 = self.theta
                    delta_theta = self.w * dt
                    self.theta += delta_theta
                    self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
                    self.x += radius * (math.sin(self.theta) - math.sin(theta0))
                    self.y += radius * (math.cos(theta0) - math.cos(self.theta))

        # 发布里程计
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # 添加姿态协方差，帮助 SLAM 更好地进行激光匹配
        odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e9, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e9, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e9, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.5
        ]

        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.w

        # 添加速度协方差
        odom_msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e9, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e9, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e9, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e9, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2
        ]

        self.odom_pub.publish(odom_msg)
        
        # 发布TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
