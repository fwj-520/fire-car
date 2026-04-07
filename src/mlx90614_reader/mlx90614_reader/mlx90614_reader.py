#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import re
from std_msgs.msg import Float32MultiArray

# 串口配置 - 根据实际设备修改
SERIAL_PORT = '/dev/ttyUSB2'  # 温度传感器实际串口（指定的设备）
BAUD_RATE = 115200

# 温度范围配置
MIN_TEMP = -40.0  # 最小温度（°C）
MAX_TEMP = 85.0  # 最大温度（°C）

def is_temp_valid(temp):
    """判断温度是否在有效范围内"""
    return MIN_TEMP <= temp <= MAX_TEMP

def is_data_valid(ambient, object_temp, diff):
    """判断数据是否有效"""
    if not (is_temp_valid(ambient) and is_temp_valid(object_temp)):
        return False
    return True

def parse_serial_data(data):
    """解析串口数据"""
    left_ambient = None
    left_object = None
    right_ambient = None
    right_object = None

    lines = data.split('\n')
    for line in lines:
        if 'Left' in line or 'Right' in line:
            # 匹配数据格式：数字  Left/Right DCC/DCI  XX.XX  YY.YY  ZZ.ZZ ...
            match = re.search(r'(Left|Right)\s+(?:DCC|DCI)\s+(\d+\.\d+)\s+(\d+\.\d+)\s+(-?\d+\.\d+)', line)
            if match:
                side = match.group(1)
                ambient = float(match.group(2))
                object_temp = float(match.group(3))
                diff = float(match.group(4))

                if is_data_valid(ambient, object_temp, diff):
                    if side == 'Left':
                        left_ambient = ambient
                        left_object = object_temp
                    elif side == 'Right':
                        right_ambient = ambient
                        right_object = object_temp

    return left_ambient, left_object, right_ambient, right_object

class Mlx90614ReaderNode(Node):
    def __init__(self):
        super().__init__('mlx90614_reader')
        self.get_logger().info(f"尝试连接到串口 {SERIAL_PORT} ...")

        # 创建话题发布者
        self.temp_publisher = self.create_publisher(Float32MultiArray, '/mlx90614/temperature_array', 10)

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1, write_timeout=1)
            self.get_logger().info(f"成功连接到串口 {SERIAL_PORT}")

            # 创建定时器，降低频率以避免IO错误
            self.timer = self.create_timer(0.5, self.read_mlx90614_data)
        except Exception as e:
            self.get_logger().error(f"无法打开串口: {e}")
            rclpy.shutdown()

    def read_mlx90614_data(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                left_ambient, left_object, right_ambient, right_object = parse_serial_data(data)

                # 发布温度数据话题 - 仅在解析到有效数据时发布
                if left_ambient is not None or right_ambient is not None:
                    temp_msg = Float32MultiArray()
                    temp_msg.data = [
                        left_ambient if left_ambient is not None else -999.0,
                        left_object if left_object is not None else -999.0,
                        right_ambient if right_ambient is not None else -999.0,
                        right_object if right_object is not None else -999.0
                    ]
                    self.temp_publisher.publish(temp_msg)

                    # 显示数据
                    self.get_logger().info("\n=== MLX90614 传感器数据 ===")
                    if left_ambient is not None and left_object is not None:
                        self.get_logger().info(f"左传感器: 环境温度: {left_ambient:.2f} °C, 物体温度: {left_object:.2f} °C")
                    if right_ambient is not None and right_object is not None:
                        self.get_logger().info(f"右传感器: 环境温度: {right_ambient:.2f} °C, 物体温度: {right_object:.2f} °C")

        except Exception as e:
            # 更健壮的错误处理，尝试重新连接
            self.get_logger().warn(f"读取数据失败: {e}")
            self.reconnect_serial()

    def reconnect_serial(self):
        """尝试重新连接串口"""
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
            # 尝试重新连接
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"成功重新连接到串口 {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"无法重新连接串口: {e}")
            time.sleep(1.0)


    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info(f"串口 {SERIAL_PORT} 已关闭")

def main(args=None):
    rclpy.init(args=args)
    node = Mlx90614ReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程序被用户中断，正在退出...")
    finally:
        if hasattr(node, 'timer'):
            node.destroy_timer(node.timer)
        if hasattr(node, 'stop_timer'):
            node.destroy_timer(node.stop_timer)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
