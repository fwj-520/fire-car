#!/usr/bin/env python3
import serial
import time
import re

# 串口配置
SERIAL_PORT = '/dev/ttyUSB2'  # 修改为实际使用的串口
BAUD_RATE = 115200

def parse_serial_data(data):
    """解析串口数据"""
    left_ambient = None
    left_object = None
    right_ambient = None
    right_object = None

    lines = data.split('\n')
    for line in lines:
        if 'Left' in line or 'Right' in line:
            match = re.search(r'(\d+\.\d+)  (\d+\.\d+)  (\d+\.\d+)', line)
            if match:
                ambient = float(match.group(1))
                object_temp = float(match.group(2))
                diff = float(match.group(3))

                if 'Left' in line:
                    left_ambient = ambient
                    left_object = object_temp
                elif 'Right' in line:
                    right_ambient = ambient
                    right_object = object_temp

    return left_ambient, left_object, right_ambient, right_object

def read_mlx90614_data():
    """读取并显示MLX90614传感器数据"""
    print(f"尝试连接到串口 {SERIAL_PORT} ...")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"成功连接到串口 {SERIAL_PORT}")

        while True:
            try:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    left_ambient, left_object, right_ambient, right_object = parse_serial_data(data)

                    if left_ambient is not None or right_ambient is not None:
                        print("\n=== MLX90614 传感器数据 ===")

                        if left_ambient is not None and left_object is not None:
                            print(f"左传感器:")
                            print(f"  环境温度: {left_ambient:.2f} °C")
                            print(f"  物体温度: {left_object:.2f} °C")

                        if right_ambient is not None and right_object is not None:
                            print(f"右传感器:")
                            print(f"  环境温度: {right_ambient:.2f} °C")
                            print(f"  物体温度: {right_object:.2f} °C")

                        if left_ambient is not None and right_ambient is not None:
                            avg_ambient = (left_ambient + right_ambient) / 2
                            print(f"平均环境温度: {avg_ambient:.2f} °C")

                        if left_object is not None and right_object is not None:
                            avg_object = (left_object + right_object) / 2
                            print(f"平均物体温度: {avg_object:.2f} °C")

                time.sleep(0.1)

            except Exception as e:
                print(f"读取或解析数据时出错: {e}")
                time.sleep(1)

    except Exception as e:
        print(f"无法打开串口: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"串口 {SERIAL_PORT} 已关闭")

if __name__ == '__main__':
    try:
        read_mlx90614_data()
    except KeyboardInterrupt:
        print("\n程序被用户中断，正在退出...")
