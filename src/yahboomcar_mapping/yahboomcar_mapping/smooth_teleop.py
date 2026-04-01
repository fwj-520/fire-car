#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import threading

class SmoothTeleop(Node):
    def __init__(self):
        super().__init__('smooth_teleop')
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz平滑更新
        
        # 速度参数（降低建图速度）
        self.max_linear = 0.15      # 最大线速度 0.15 m/s
        self.max_angular = 0.3       # 最大角速度 0.3 rad/s
        self.acceleration = 0.03     # 加速度
        
        # 当前目标速度
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # 当前实际速度（用于平滑过渡）
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.get_logger().info('🎮 平滑键盘控制已启动')
        self.get_logger().info('   前进: i, 后退: ,, 左转: j, 右转: l, 停止: k')
        self.get_logger().info(f'   最大速度: {self.max_linear} m/s, {self.max_angular} rad/s')
        
        # 键盘输入线程
        self.running = True
        self.key_thread = threading.Thread(target=self.keyboard_loop)
        self.key_thread.start()
        
    def keyboard_loop(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        try:
            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self.handle_key(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def handle_key(self, key):
        # 设置目标速度
        if key == 'i':      # 前进
            self.target_linear = self.max_linear
        elif key == ',':    # 后退
            self.target_linear = -self.max_linear
        elif key == 'j':    # 左转
            self.target_angular = self.max_angular
        elif key == 'l':    # 右转
            self.target_angular = -self.max_angular
        elif key == 'k':    # 停止
            self.target_linear = 0.0
            self.target_angular = 0.0
        elif key == 'q':    # 退出
            self.running = False
    
    def timer_callback(self):
        # 平滑过渡到目标速度
        if self.current_linear < self.target_linear:
            self.current_linear = min(self.target_linear, self.current_linear + self.acceleration)
        elif self.current_linear > self.target_linear:
            self.current_linear = max(self.target_linear, self.current_linear - self.acceleration)
        
        if self.current_angular < self.target_angular:
            self.current_angular = min(self.target_angular, self.current_angular + self.acceleration)
        elif self.current_angular > self.target_angular:
            self.current_angular = max(self.target_angular, self.current_angular - self.acceleration)
        
        # 发布速度指令
        cmd = Twist()
        cmd.linear.x = self.current_linear
        cmd.angular.z = self.current_angular
        
        if abs(cmd.linear.x) > 0.01 or abs(cmd.angular.z) > 0.01:
            self.get_logger().debug(f'速度: {cmd.linear.x:.2f} m/s, {cmd.angular.z:.2f} rad/s')
        
        self.pub.publish(cmd)
    
    def destroy_node(self):
        self.running = False
        if self.key_thread.is_alive():
            self.key_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SmoothTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
