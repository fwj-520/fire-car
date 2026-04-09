#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class STM32MotorDriver(Node):
    def __init__(self):
        super().__init__('hiwonder_motor_driver')

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

        self.create_timer(0.5, self.timer_callback)
        self.last_cmd = Twist()
        self.last_sent_time = 0

        self.get_logger().info('🚗 STM32电机驱动已启动')

    def cmd_callback(self, msg):
        """存储最后收到的指令"""
        self.last_cmd = msg

    def timer_callback(self):
        """定时发送指令，确保电机持续接收信号"""
        current_time = time.time()
        if current_time - self.last_sent_time > 0.3:
            # 检查是否是停止状态
            if abs(self.last_cmd.linear.x) < 0.01 and abs(self.last_cmd.angular.z) < 0.01:
                # 发送停止命令
                self.send_stop_command()
            else:
                self.send_to_stm32(self.last_cmd)
            self.last_sent_time = current_time

    def send_stop_command(self):
        """直接发送停止命令给 STM32"""
        try:
            stop_data = bytearray([0xAA, 0x55, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x04])
            self.ser.write(stop_data)
            self.get_logger().debug('✅ 发送停止命令')
        except Exception as e:
            self.get_logger().error(f'❌ 发送停止命令失败: {e}')

    def send_to_stm32(self, msg):
        """发送指令到 STM32 - 优化稳定性"""
        try:
            # 检查是否是停止命令
            is_stop_command = (abs(msg.linear.x) < 0.01 and abs(msg.angular.z) < 0.01)

            if is_stop_command:
                # 发送停止命令
                left_speed = 0
                right_speed = 0
                data = bytearray([0xAA, 0x55, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x04])
                self.ser.write(data)
                self.get_logger().debug('发送停止命令')
                return

            # 打印接收到的原始指令（使用 debug 级别，减少日志噪声）
            self.get_logger().debug(f'接收到指令: 线速度={msg.linear.x:.2f}, 角速度={msg.angular.z:.2f}')

            # 电机校准参数：用于修正左右履带速度差异
            LEFT_WHEEL_CALIBRATION = 0.998  # 左履带速度修正系数（降低 0.2%）
            RIGHT_WHEEL_CALIBRATION = 1.002 # 右履带速度修正系数（提高 0.2%）

            # 针对低速和高速分别优化（起步时更精细）
            if abs(msg.linear.x) < 0.2:
                LEFT_WHEEL_CALIBRATION = 0.995  # 左履带 -0.5%
                RIGHT_WHEEL_CALIBRATION = 1.005 # 右履带 +0.5%
            elif abs(msg.linear.x) < 0.4:
                LEFT_WHEEL_CALIBRATION = 0.997  # 左履带 -0.3%
                RIGHT_WHEEL_CALIBRATION = 1.003 # 右履带 +0.3%

            left_speed = int(msg.linear.x * 10 * LEFT_WHEEL_CALIBRATION)
            right_speed = int(msg.linear.x * 10 * RIGHT_WHEEL_CALIBRATION)

            if msg.angular.z != 0:
                # 优化转向：对于大角度转弯，实现单履带运动
                if abs(msg.angular.z) > 0.6:  # 大角度转弯，单履带运动
                    if msg.angular.z > 0:  # 左转弯 - 左履带后退，右履带停止
                        left_speed = int(-0.3 * 10 * LEFT_WHEEL_CALIBRATION)
                        right_speed = 0
                    else:  # 右转弯 - 右履带后退，左履带停止
                        left_speed = 0
                        right_speed = int(-0.3 * 10 * RIGHT_WHEEL_CALIBRATION)
                    self.get_logger().debug(f'单履带转弯: 左={left_speed}, 右={right_speed}')
                else:
                    # 小角度转向：通过速度差实现偏转，避免大角度转弯导致漂移
                    if abs(msg.angular.z) < 0.2:
                        speed_diff = msg.angular.z * 3
                    elif abs(msg.angular.z) < 0.4:
                        speed_diff = msg.angular.z * 5
                    else:
                        speed_diff = msg.angular.z * 8

                    base_linear = msg.linear.x * 10
                    left_speed = int(base_linear * LEFT_WHEEL_CALIBRATION - speed_diff)
                    right_speed = int(base_linear * RIGHT_WHEEL_CALIBRATION + speed_diff)

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
            self.get_logger().debug(f'发送: L={left_speed:3d}, R={right_speed:3d}, 数据: {data.hex()}')

        except Exception as e:
            self.get_logger().error(f'发送失败: {e}')
            # 发生错误时，发送停止命令
            try:
                self.send_stop_command()
            except:
                pass

    def send_to_stm32(self, msg):
        try:
            # 检查是否是停止命令
            is_stop_command = (abs(msg.linear.x) < 0.01 and abs(msg.angular.z) < 0.01)

            if is_stop_command:
                # 发送停止命令
                left_speed = 0
                right_speed = 0
                data = bytearray([0xAA, 0x55, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x04])
                self.ser.write(data)
                self.get_logger().debug('发送停止命令')
                return

            # 打印接收到的原始指令（使用 debug 级别，减少日志噪声）
            self.get_logger().debug(f'接收到指令: 线速度={msg.linear.x:.2f}, 角速度={msg.angular.z:.2f}')

            # 电机校准参数：用于修正左右履带速度差异
            LEFT_WHEEL_CALIBRATION = 0.998  # 左履带速度修正系数（降低 0.2%）
            RIGHT_WHEEL_CALIBRATION = 1.002 # 右履带速度修正系数（提高 0.2%）

            # 针对低速和高速分别优化（起步时更精细）
            if abs(msg.linear.x) < 0.2:
                LEFT_WHEEL_CALIBRATION = 0.995  # 左履带 -0.5%
                RIGHT_WHEEL_CALIBRATION = 1.005 # 右履带 +0.5%
            elif abs(msg.linear.x) < 0.4:
                LEFT_WHEEL_CALIBRATION = 0.997  # 左履带 -0.3%
                RIGHT_WHEEL_CALIBRATION = 1.003 # 右履带 +0.3%

            left_speed = int(msg.linear.x * 10 * LEFT_WHEEL_CALIBRATION)
            right_speed = int(msg.linear.x * 10 * RIGHT_WHEEL_CALIBRATION)

            if msg.angular.z != 0:
                # 优化转向：对于大角度转弯，实现单履带运动
                if abs(msg.angular.z) > 0.6:  # 大角度转弯，单履带运动
                    if msg.angular.z > 0:  # 左转弯 - 左履带后退，右履带停止
                        left_speed = int(-0.3 * 10 * LEFT_WHEEL_CALIBRATION)
                        right_speed = 0
                    else:  # 右转弯 - 右履带后退，左履带停止
                        left_speed = 0
                        right_speed = int(-0.3 * 10 * RIGHT_WHEEL_CALIBRATION)
                    self.get_logger().debug(f'单履带转弯: 左={left_speed}, 右={right_speed}')
                else:
                    # 小角度转向：通过速度差实现偏转，避免大角度转弯导致漂移
                    if abs(msg.angular.z) < 0.2:
                        speed_diff = msg.angular.z * 3
                    elif abs(msg.angular.z) < 0.4:
                        speed_diff = msg.angular.z * 5
                    else:
                        speed_diff = msg.angular.z * 8

                    base_linear = msg.linear.x * 10
                    left_speed = int(base_linear * LEFT_WHEEL_CALIBRATION - speed_diff)
                    right_speed = int(base_linear * RIGHT_WHEEL_CALIBRATION + speed_diff)

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
            self.get_logger().debug(f'发送: L={left_speed:3d}, R={right_speed:3d}, 数据: {data.hex()}')

        except Exception as e:
            self.get_logger().error(f'发送失败: {e}')

    def destroy_node(self):
        """节点被销毁时发送停止命令"""
        try:
            # 直接发送停止命令给 STM32（不依赖 ROS 系统）
            print('🔴 正在发送停止命令给STM32...')
            stop_data = bytearray([0xAA, 0x55, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x04])
            for i in range(10):  # 多次发送确保停止命令被接收
                self.ser.write(stop_data)
                time.sleep(0.05)
            print('✅ 停止命令已发送')
        except Exception as e:
            print(f'❌ 发送停止命令失败: {e}')

        if hasattr(self, 'ser'):
            self.ser.close()

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STM32MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
