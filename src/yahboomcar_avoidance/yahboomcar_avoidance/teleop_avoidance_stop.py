#!/usr/bin/env python3
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopAvoidanceStop(Node):
    def __init__(self):
        super().__init__('teleop_avoidance_stop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # 按键映射
        self.key_bindings = {
            'i': (0.5, 0.0),    # 前进
            ',': (-0.5, 0.0),   # 后退
            'j': (0.0, 0.6),    # 左转
            'l': (0.0, -0.6),   # 右转
            'k': (0.0, 0.0),    # 停止
            'q': None           # 退出
        }

        self.get_logger().info('='*60)
        self.get_logger().info('🚗 避障停止键盘控制器')
        self.get_logger().info('='*60)
        self.get_logger().info('控制键：')
        self.get_logger().info('  i : 前进')
        self.get_logger().info('  , : 后退')
        self.get_logger().info('  j : 左转')
        self.get_logger().info('  l : 右转')
        self.get_logger().info('  k : 立即停止（发送停止指令）')
        self.get_logger().info('  q : 退出程序')
        self.get_logger().info('='*60)

        self.running = True
        self.last_cmd = Twist()

    def get_key(self):
        """读取键盘输入"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        """主运行循环"""
        while self.running and rclpy.ok():
            key = self.get_key()

            if key in self.key_bindings:
                if key == 'q':
                    self.get_logger().info('🛑 收到退出指令')
                    self.running = False
                    break

                elif key == 'k':
                    # 停止避障（发送停止指令）
                    self.get_logger().info('⏹️  发送停止指令')
                    stop_cmd = Twist()
                    self.publisher_.publish(stop_cmd)
                    self.last_cmd = stop_cmd

                elif self.key_bindings[key] is not None:
                    # 发送运动指令
                    cmd = Twist()
                    cmd.linear.x, cmd.angular.z = self.key_bindings[key]
                    self.publisher_.publish(cmd)
                    self.last_cmd = cmd

    def cleanup(self):
        """清理资源"""
        self.get_logger().info('🔄 正在清理资源...')
        stop_cmd = Twist()
        self.publisher_.publish(stop_cmd)
        self.get_logger().info('✅ 避障已停止')

def main(args=None):
    rclpy.init(args=args)

    node = TeleopAvoidanceStop()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('🛑 收到中断信号')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
