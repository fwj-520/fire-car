#!/usr/bin/env python3
"""
MLX90614 红外温度传感器 ROS 2 驱动节点
通过串口与 STM32 通信获取传感器数据
功能：
1. 初始化串口通信
2. 从 STM32 接收 MLX90614 温度数据
3. 发布温度数据到 ROS 2 话题
4. 提供服务或参数配置功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32MultiArray
import time
import serial
import struct

# 串口通信协议
# STM32 发送的数据格式（示例，需要根据实际实现调整）
# 帧格式：0xAA 0x55 0x03 [数据长度] [环境温度高字节] [环境温度低字节] [物体温度高字节] [物体温度低字节] [校验和]

# 通信常量
FRAME_HEADER1 = 0xAA
FRAME_HEADER2 = 0x55
CMD_GET_MLX90614 = 0x03  # 获取 MLX90614 数据的命令

# 温度阈值（摄氏度）
TEMP_ALARM_HIGH = 60.0
TEMP_ALARM_LOW = 10.0
TEMP_FIRE_SUSPECT = 45.0
TEMP_FIRE_CONFIRM = 60.0
TEMP_INVALID = -999.0

# 滤波参数
FILTER_SIZE = 5

class MLX90614Node(Node):
    def __init__(self):
        super().__init__('mlx90614_node')

        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyUSB2')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 50.0)

        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # 初始化串口通信
        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baudrate,
                timeout=0.1,
                bytesize=8,
                parity='N',
                stopbits=1
            )
            self.get_logger().info(f'✅ 串口连接成功: {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'❌ 串口连接失败: {e}')
            return

        # 创建一个统一的话题发布者（包含两个传感器的温度数据）
        self.temp_array_pub = self.create_publisher(
            Float32MultiArray, '/mlx90614/temperature_array', 10)

        # 创建左右传感器单独的话题发布者（保留，便于调试）
        self.left_ambient_temp_pub = self.create_publisher(
            Temperature, '/mlx90614/left_ambient_temperature', 10)
        self.left_object_temp_pub = self.create_publisher(
            Temperature, '/mlx90614/left_object_temperature', 10)
        self.right_ambient_temp_pub = self.create_publisher(
            Temperature, '/mlx90614/right_ambient_temperature', 10)
        self.right_object_temp_pub = self.create_publisher(
            Temperature, '/mlx90614/right_object_temperature', 10)

        # 验证发布者是否创建成功
        self.get_logger().info(f'✅ 发布者创建:')
        self.get_logger().info(f'  - /mlx90614/temperature_array: {self.temp_array_pub}')
        self.get_logger().info(f'  - /mlx90614/left_ambient_temperature: {self.left_ambient_temp_pub}')
        self.get_logger().info(f'  - /mlx90614/left_object_temperature: {self.left_object_temp_pub}')
        self.get_logger().info(f'  - /mlx90614/right_ambient_temperature: {self.right_ambient_temp_pub}')
        self.get_logger().info(f'  - /mlx90614/right_object_temperature: {self.right_object_temp_pub}')

        # 创建定时器
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # 初始化滤波数据
        self.left_ambient_filter = [0.0] * FILTER_SIZE
        self.left_object_filter = [0.0] * FILTER_SIZE
        self.right_ambient_filter = [0.0] * FILTER_SIZE
        self.right_object_filter = [0.0] * FILTER_SIZE
        self.filter_index = 0

        # 串口接收缓冲区
        self.data_buffer = b''

        # 存储上一次有效的温度数据
        self.last_left_ambient = 25.0
        self.last_left_object = 25.5
        self.last_right_ambient = 24.5
        self.last_right_object = 25.0

        self.get_logger().info('MLX90614 传感器驱动节点已启动')

    def read_temperature_from_stm32(self):
        """从 STM32 读取温度数据"""
        try:
            # 读取串口数据
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.data_buffer += data

            # 解析文本格式数据
            left_ambient = TEMP_INVALID
            left_object = TEMP_INVALID
            right_ambient = TEMP_INVALID
            right_object = TEMP_INVALID

            # 将字节转换为字符串并分割成行
            try:
                text_data = self.data_buffer.decode('utf-8', errors='ignore')
                lines = text_data.split('\n')

                # 优化解析逻辑，确保能稳定识别传感器数据
                for line in lines:
                    import re
                    numbers = re.findall(r'\d+\.\d+', line)

                    # 打印所有接收到的行，详细显示
                    self.get_logger().info(f'接收到行: "{line.strip()}"，数字: {numbers}')

                    # 识别传感器数据 - 简化匹配规则，提高识别率
                    if len(numbers) >= 2:
                        if "DCC" in line:
                            left_ambient = float(numbers[0])
                            left_object = float(numbers[1])
                            self.get_logger().warning(f'✅ 左传感器 (DCC): 环境温度 {left_ambient:.2f}°C, 物体温度 {left_object:.2f}°C')
                        elif "DCI" in line:
                            right_ambient = float(numbers[0])
                            right_object = float(numbers[1])
                            self.get_logger().warning(f'✅ 右传感器 (DCI): 环境温度 {right_ambient:.2f}°C, 物体温度 {right_object:.2f}°C')
                        else:
                            # 对于未明确标识的传感器，根据位置判断
                            if left_ambient == TEMP_INVALID:
                                left_ambient = float(numbers[0])
                                left_object = float(numbers[1])
                                self.get_logger().warning(f'✅ 默认左传感器: 环境温度 {left_ambient:.2f}°C, 物体温度 {left_object:.2f}°C')
                            elif right_ambient == TEMP_INVALID:
                                right_ambient = float(numbers[0])
                                right_object = float(numbers[1])
                                self.get_logger().warning(f'✅ 默认右传感器: 环境温度 {right_ambient:.2f}°C, 物体温度 {right_object:.2f}°C')

                # 清除已处理的部分 - 优化缓冲区管理，确保数据不被重复解析
                if left_ambient != TEMP_INVALID or right_ambient != TEMP_INVALID:
                    self.data_buffer = b''

                # 如果没有找到有效数据，且缓冲区过大，清空
                if len(self.data_buffer) > 500:
                    self.get_logger().warning('缓冲区过长且未找到有效数据，清空缓冲区')
                    self.data_buffer = b''

                # 优化：确保两个传感器都有数据，但不提供虚假的默认值
                if left_ambient == TEMP_INVALID and right_ambient == TEMP_INVALID:
                    # 如果没有任何有效数据，返回无效值
                    return TEMP_INVALID, TEMP_INVALID, TEMP_INVALID, TEMP_INVALID
                elif left_ambient == TEMP_INVALID and right_ambient != TEMP_INVALID:
                    # 只有右传感器有效，左传感器返回无效值
                    left_ambient = TEMP_INVALID
                    left_object = TEMP_INVALID
                elif right_ambient == TEMP_INVALID and left_ambient != TEMP_INVALID:
                    # 只有左传感器有效，右传感器返回无效值
                    right_ambient = TEMP_INVALID
                    right_object = TEMP_INVALID

            except Exception as decode_e:
                self.get_logger().error(f'数据解码失败: {decode_e}')
                self.data_buffer = b''

            return left_ambient, left_object, right_ambient, right_object

        except Exception as e:
            self.get_logger().error(f'读取传感器数据失败: {e}')
            return TEMP_INVALID, TEMP_INVALID, TEMP_INVALID, TEMP_INVALID

    def apply_filter(self, filter_data, new_value):
        """应用滤波"""
        filter_data[self.filter_index] = new_value
        self.filter_index = (self.filter_index + 1) % FILTER_SIZE

        return sum(filter_data) / FILTER_SIZE

    def timer_callback(self):
        """定时器回调函数"""
        left_ambient, left_object, right_ambient, right_object = self.read_temperature_from_stm32()

        # 更新上一次有效的温度数据
        if left_ambient != TEMP_INVALID and left_object != TEMP_INVALID:
            self.last_left_ambient = left_ambient
            self.last_left_object = left_object
        if right_ambient != TEMP_INVALID and right_object != TEMP_INVALID:
            self.last_right_ambient = right_ambient
            self.last_right_object = right_object

        # 处理左传感器数据
        if left_ambient != TEMP_INVALID and left_object != TEMP_INVALID:
            filtered_left_ambient = self.apply_filter(self.left_ambient_filter, left_ambient)
            filtered_left_object = self.apply_filter(self.left_object_filter, left_object)

            # 发布左传感器数据
            left_ambient_msg = Temperature()
            left_ambient_msg.header.stamp = self.get_clock().now().to_msg()
            left_ambient_msg.header.frame_id = 'mlx90614_frame'
            left_ambient_msg.temperature = filtered_left_ambient
            left_ambient_msg.variance = 0.01
            self.left_ambient_temp_pub.publish(left_ambient_msg)

            left_object_msg = Temperature()
            left_object_msg.header.stamp = self.get_clock().now().to_msg()
            left_object_msg.header.frame_id = 'mlx90614_frame'
            left_object_msg.temperature = filtered_left_object
            left_object_msg.variance = 0.01
            self.left_object_temp_pub.publish(left_object_msg)

        # 处理右传感器数据
        if right_ambient != TEMP_INVALID and right_object != TEMP_INVALID:
            filtered_right_ambient = self.apply_filter(self.right_ambient_filter, right_ambient)
            filtered_right_object = self.apply_filter(self.right_object_filter, right_object)

            # 发布右传感器数据
            right_ambient_msg = Temperature()
            right_ambient_msg.header.stamp = self.get_clock().now().to_msg()
            right_ambient_msg.header.frame_id = 'mlx90614_frame'
            right_ambient_msg.temperature = filtered_right_ambient
            right_ambient_msg.variance = 0.01
            self.right_ambient_temp_pub.publish(right_ambient_msg)

            right_object_msg = Temperature()
            right_object_msg.header.stamp = self.get_clock().now().to_msg()
            right_object_msg.header.frame_id = 'mlx90614_frame'
            right_object_msg.temperature = filtered_right_object
            right_object_msg.variance = 0.01
            self.right_object_temp_pub.publish(right_object_msg)

        # 发布综合数据 - 包含所有传感器数据的统一话题
        temp_array_msg = Float32MultiArray()

        if left_ambient != TEMP_INVALID and left_object != TEMP_INVALID and right_ambient != TEMP_INVALID and right_object != TEMP_INVALID:
            # 两个传感器数据都有效
            temp_array_msg.data = [filtered_left_ambient, filtered_left_object, filtered_right_ambient, filtered_right_object]
            self.temp_array_pub.publish(temp_array_msg)
            self.get_logger().info(f'✅ 温度数据: DCC左传感器 {filtered_left_object:.2f}°C, DCI右传感器 {filtered_right_object:.2f}°C')
        elif left_ambient != TEMP_INVALID and left_object != TEMP_INVALID:
            # 只有左传感器数据
            temp_array_msg.data = [filtered_left_ambient, filtered_left_object, TEMP_INVALID, TEMP_INVALID]
            self.temp_array_pub.publish(temp_array_msg)
            self.get_logger().info(f'✅ 温度数据: DCC左传感器 {filtered_left_object:.2f}°C, DCI右传感器 无数据')
        elif right_ambient != TEMP_INVALID and right_object != TEMP_INVALID:
            # 只有右传感器数据
            temp_array_msg.data = [TEMP_INVALID, TEMP_INVALID, filtered_right_ambient, filtered_right_object]
            self.temp_array_pub.publish(temp_array_msg)
            self.get_logger().info(f'✅ 温度数据: DCC左传感器 无数据, DCI右传感器 {filtered_right_object:.2f}°C')
        else:
            # 两个传感器都没有有效数据，但仍然发布话题以保持频率
            temp_array_msg.data = [TEMP_INVALID, TEMP_INVALID, TEMP_INVALID, TEMP_INVALID]
            self.temp_array_pub.publish(temp_array_msg)
            self.get_logger().warning('⚠️ 未接收到有效温度数据')

    def destroy_node(self):
        """节点销毁时关闭串口"""
        if hasattr(self, 'ser'):
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        node = MLX90614Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

