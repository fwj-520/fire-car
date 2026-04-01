#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class DebugSTM32MotorDriver(Node):
    def __init__(self):
        super().__init__('debug_stm32_motor_driver')

        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            self.get_logger().info(f'✅ 串口连接成功: {port}')
        except Exception as e:
            self.get_logger().error(f'❌ 串口连接失败: {e}')
            return

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 5)

        self.create_timer(0.1, self.timer_callback)
        self.last_cmd = Twist()
        self.last_sent_time = 0
        self.sent_count = 0

        # 记录发送历史
        self.command_history = []
        self.max_history = 10

        self.get_logger().info('🚗 STM32电机驱动(调试版)已启动')
        self.get_logger().info('📊 详细记录指令发送和速度计算过程')
        self.get_logger().info('='*50)

    def cmd_callback(self, msg):
        """直接处理接收到的指令，确保立即响应"""
        self.last_cmd = msg
        self.send_to_stm32(msg)
        self.last_sent_time = time.time()

    def timer_callback(self):
        """定时发送指令，确保电机持续接收信号"""
        current_time = time.time()
        if current_time - self.last_sent_time > 0.1:
            self.get_logger().debug('⏰ 定时器超时，发送最后指令')
            self.send_to_stm32(self.last_cmd)
            self.last_sent_time = current_time

    def send_to_stm32(self, msg):
        try:
            LEFT_WHEEL_CALIBRATION = 1.0
            RIGHT_WHEEL_CALIBRATION = 0.98

            left_speed = int(msg.linear.x * 10 * LEFT_WHEEL_CALIBRATION)
            right_speed = int(msg.linear.x * 10 * RIGHT_WHEEL_CALIBRATION)

            if msg.angular.z != 0:
                base_linear = msg.linear.x * 10
                angular_term = msg.angular.z * 8
                left_speed = int(base_linear * LEFT_WHEEL_CALIBRATION - angular_term)
                right_speed = int(base_linear * RIGHT_WHEEL_CALIBRATION + angular_term)

            left_speed = max(-10, min(10, left_speed))
            right_speed = max(-10, min(10, right_speed))

            data = bytearray([0xAA, 0x55, 0x02, 0x04])
            data.append((left_speed >> 8) & 0xFF)
            data.append(left_speed & 0xFF)
            data.append((right_speed >> 8) & 0xFF)
            data.append(right_speed & 0xFF)

            checksum = sum(data[2:8]) & 0xFF
            data.append(checksum)

            self.ser.write(data)
            self.sent_count += 1

            # 记录指令历史
            self.command_history.append({
                'time': time.time(),
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z,
                'left_speed': left_speed,
                'right_speed': right_speed
            })

            if len(self.command_history) > self.max_history:
                self.command_history.pop(0)

            # 详细的调试信息
            debug_msg = (
                f"📤 [第{self.sent_count:3d}次] 发送命令"
                f" | 速度: {msg.linear.x:.3f}m/s"
                f" | 转向: {msg.angular.z:.3f}rad/s"
                f" | 左速: {left_speed:3d}"
                f" | 右速: {right_speed:3d}"
            )

            # 如果检测到速度异常波动，用红色警告显示
            if len(self.command_history) > 1:
                prev_cmd = self.command_history[-2]
                left_diff = abs(left_speed - prev_cmd['left_speed'])
                right_diff = abs(right_speed - prev_cmd['right_speed'])

                if left_diff > 2 or right_diff > 2:
                    self.get_logger().warning(f"⚠️ {debug_msg} | 速度突变!")
                else:
                    self.get_logger().info(debug_msg)
            else:
                self.get_logger().info(debug_msg)

        except Exception as e:
            self.get_logger().error(f'❌ 发送失败: {e}')

    def destroy_node(self):
        if hasattr(self, 'ser'):
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DebugSTM32MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
