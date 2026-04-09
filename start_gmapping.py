#!/usr/bin/env python3
import subprocess
import time
import signal
import sys
import os

# 进程列表
processes = []
process_groups = []  # 存储进程组ID，用于杀死整个进程树

def send_stop_command_to_stm32():
    """直接通过串口发送停止命令给 STM32 - 最直接的硬件停止方法"""
    try:
        import serial
        import time
        print("[STM32] 直接连接到控制板...")
        ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.5)

        # 清空输入缓冲区
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)

        print("[STM32] 发送停止命令...")
        stop_command = bytearray([0xAA, 0x55, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x04])

        # 多次发送停止命令，确保 STM32 收到
        for i in range(20):
            ser.write(stop_command)
            time.sleep(0.03)

        ser.close()
        print("[STM32] 停止命令已发送")
    except Exception as e:
        print(f"[Warn] 直接发送停止命令失败: {e}")

def send_stop_command():
    """发送停止命令给小车 - 最直接、最强制的方法"""
    print("[Car] 正在强制停止小车...")

    # 方法1：最直接的硬件停止方法（完全绕过 ROS 系统）
    send_stop_command_to_stm32()

    # 方法2：强制停止电机驱动节点
    try:
        subprocess.run("pkill -f 'hiwonder_motor_driver' 2>/dev/null", shell=True)
        subprocess.run("pkill -9 -f 'hiwonder_motor_driver' 2>/dev/null", shell=True)
        print("[OK] 电机驱动节点已停止")
    except Exception as e:
        print(f"[Warn] 停止电机节点失败: {e}")

    # 方法3：清空最后发送的指令（防止重启后继续执行旧指令）
    try:
        # 创建临时的停止命令并发送
        stop_cmd = "ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --no-daemon 2>/dev/null"
        subprocess.run(stop_cmd, shell=True, timeout=0.5)
    except Exception:
        pass

    # 短暂延迟确保硬件响应
    time.sleep(0.5)
    print("[OK] 小车已停止")

def kill_processes():
    """杀死所有子进程及它们的子进程 - 确保彻底清理所有进程"""
    # 先强制停止所有 ROS2 相关进程（使用系统命令）
    try:
        targets = ['ydlidar_ros2_driver_node', 'slam_gmapping', 'simple_odom', 'simple_avoidance', 'rviz2', 'rf2o_laser_odometry', 'hiwonder_motor_driver', 'temperature_marker', 'fire_source_localization', 'path_recorder', 'path_optimizer', 'path_tracker', 'exploration_ender']
        for target in targets:
            for i in range(2):
                subprocess.run(f"pkill -f '{target}' 2>/dev/null", shell=True)
                subprocess.run(f"pkill -9 -f '{target}' 2>/dev/null", shell=True)
                time.sleep(0.1)
    except Exception as e:
        print(f"[Warn] 清理系统进程失败: {e}")

    # 然后杀死所有子进程
    for i, p in enumerate(processes):
        if p and p.poll() is None:
            try:
                if i < len(process_groups) and process_groups[i] != 0:
                    try:
                        os.killpg(process_groups[i], signal.SIGKILL)
                        try:
                            p.wait(timeout=0.2)
                        except:
                            pass
                    except:
                        pass
                else:
                    try:
                        p.kill()
                        try:
                            p.wait(timeout=0.2)
                        except:
                            pass
                    except:
                        pass
            except Exception as e:
                print(f"[Warn] 杀死进程失败: {e}")

    # 最终清理所有残留进程
    try:
        subprocess.run("pkill -9 -f 'ros2' 2>/dev/null", shell=True)
        subprocess.run("pkill -9 -f 'ydlidar' 2>/dev/null", shell=True)
        subprocess.run("pkill -9 -f 'stm32' 2>/dev/null", shell=True)
        time.sleep(0.3)
        print("[OK] 所有进程已彻底清理")
    except Exception as e:
        print(f"[Warn] 最终清理残留进程失败: {e}")

def signal_handler(sig, frame):
    print("\n[Stop] 正在停止小车...")

    # 1. 优先发送停止命令给小车（确保小车立即停止）
    send_stop_command()

    # 2. 然后终止所有进程
    print("[Stop] 正在停止所有节点...")
    kill_processes()

    print("[Stop] 所有节点已关闭")
    os._exit(0)

def run_command(cmd, name, use_setsid=True):
    """启动命令"""
    print(f"启动 {name}: {cmd}")
    try:
        if use_setsid:
            proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
            process_groups.append(proc.pid)
        else:
            proc = subprocess.Popen(cmd, shell=True)
        processes.append(proc)
        return proc
    except Exception as e:
        print(f"启动 {name} 失败: {e}")
        return None

def main():
    global processes, process_groups
    processes = []
    process_groups = []

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 50)
    print("[Car] 启动手持建图系统（RF2O激光里程计）")
    print("=" * 50)

    # 快速清理残留节点
    print("[Tools] 清理残留节点...")
    subprocess.run("pkill -f 'ydlidar_ros2_driver_node' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'slam_gmapping' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'hiwonder_motor_driver' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'simple_odom' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'simple_avoidance' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'rviz2' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'rf2o_laser_odometry' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'temperature_marker' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'fire_source_localization' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'path_recorder' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'path_optimizer' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'path_tracker' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'exploration_ender' 2>/dev/null", shell=True)
    time.sleep(0.5)

    # 1. 激光雷达
    run_command("ros2 launch ydlidar_ros2_driver ydlidar_launch.py", "激光雷达")
    time.sleep(2)

    # 2. 静态TF：base_footprint -> base_link
    run_command("ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link", "base_footprint -> base_link TF")
    time.sleep(0.5)

    # 3. 静态TF：base_link -> laser_frame（正常安装）
    run_command("ros2 run tf2_ros static_transform_publisher 0.07 0.0075 0.024 0 0 0 base_link laser_frame", "雷达TF")
    time.sleep(0.5)

    # 4. RF2O 激光里程计（替代 simple_odom）
    run_command("ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py", "RF2O 激光里程计")
    time.sleep(2)

    # 5. 电机驱动
    run_command("ros2 run stm32_driver hiwonder_motor_driver", "电机驱动")
    time.sleep(1)

    # 6. 避障节点
    run_command("ros2 run yahboomcar_avoidance simple_avoidance", "避障节点")
    time.sleep(1)

    # 7. GMapping 建图节点
    run_command("ros2 launch slam_gmapping slam_gmapping_launch.py", "GMapping 建图")
    time.sleep(2)

    # 8. 温度传感器节点
    run_command("ros2 run mlx90614_reader mlx90614_reader", "温度传感器")
    time.sleep(1)

    # 9. 温度标记节点
    run_command("ros2 run yahboomcar_mapping temperature_marker", "温度标记")
    time.sleep(1)

    # 10. 火源定位节点
    run_command("ros2 run yahboomcar_mapping fire_source_localization", "火源定位")
    time.sleep(1)

    # 11. 路径记录节点
    run_command("ros2 run yahboomcar_mapping path_recorder", "路径记录")
    time.sleep(1)

    # 12. 路径优化节点
    run_command("ros2 run yahboomcar_mapping path_optimizer", "路径优化")
    time.sleep(1)

    # 13. 路径跟踪节点
    run_command("ros2 run yahboomcar_mapping path_tracker", "路径跟踪")
    time.sleep(1)

    # 14. 探索结束触发节点
    run_command("ros2 run yahboomcar_mapping exploration_ender", "探索结束触发")
    time.sleep(1)

    # 15. RViz 可视化
    run_command("ros2 run rviz2 rviz2 -d " + os.path.abspath("src/slam_gmapping/rviz/map_view.rviz"), "RViz", use_setsid=False)

    print("\n[OK] 所有节点已启动")
    print("[Info] 使用手持建图方法（RF2O激光里程计）")
    print("[Info] 启动时间已优化，节点启动间隔已缩短")
    print("[Tips] 按 i 键让小车前进，j/l 转向，k 停止")
    print("[Tips] 建图完成后保存：")
    print("   ros2 run nav2_map_server map_saver_cli -f ~/yahboomcar_ws/maps/rf2o_gmapping_map")

    # 优化等待和异常处理
    try:
        import threading

        def process_monitor():
            """监控进程状态的后台线程"""
            while True:
                time.sleep(0.5)
                all_finished = True
                for p in processes:
                    if p and p.poll() is None:
                        all_finished = False
                        break
                if all_finished:
                    print("所有进程已结束")
                    os._exit(0)

        monitor_thread = threading.Thread(target=process_monitor, daemon=True)
        monitor_thread.start()

        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("收到键盘中断")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        kill_processes()

if __name__ == "__main__":
    main()
