#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def stop(self):
        print('🛑 紧急停止中...')
        cmd = Twist()
        for i in range(20):
            self.pub.publish(cmd)
            time.sleep(0.05)
        print('✅ 停止完成')

def main():
    rclpy.init()
    node = EmergencyStop()
    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
