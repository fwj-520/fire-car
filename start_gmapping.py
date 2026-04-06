#!/usr/bin/env python3
import subprocess
import time
import signal
import sys
import os

# 进程列表
processes = []
process_groups = []  # 存储进程组ID，用于杀死整个进程树

def send_stop_command():
    """发送停止命令给小车 - 先向控制器发送停止指令，然后强制停止电机驱动"""
    print("[Car] 正在发送停止命令...")

    # 方法1：向控制器发送停止指令
    try:
        stop_cmd = "ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --no-daemon 2>/dev/null"
        subprocess.run(stop_cmd, shell=True, timeout=0.3)
        print("[OK] 停止指令已发送到控制器")
    except Exception as e:
        print(f"[Warn] 发送停止命令失败: {e}")

    # 方法2：等待控制器处理停止指令
    try:
        print("[Car] 等待控制器处理停止指令...")
        time.sleep(1.0)
    except Exception as e:
        print(f"[Warn] 等待停止命令处理失败: {e}")

    # 方法3：强制停止电机驱动节点
    try:
        print("[Car] 强制停止电机驱动节点...")
        subprocess.run("pkill -f 'hiwonder_motor_driver' 2>/dev/null", shell=True)
        time.sleep(0.5)
        print("[OK] 电机驱动节点已停止")
    except Exception as e:
        print(f"[Warn] 停止电机节点失败: {e}")

    print("[OK] 小车已停止")

def kill_processes():
    """杀死所有子进程及它们的子进程 - 确保彻底清理所有进程"""
    # 先强制停止所有 ROS2 相关进程（使用系统命令）
    try:
        targets = ['ydlidar_ros2_driver_node', 'slam_gmapping', 'simple_odom', 'simple_avoidance', 'rviz2', 'rf2o_laser_odometry', 'hiwonder_motor_driver', 'mlx90614_reader', 'temperature_marker']
        for target in targets:
            for i in range(3):  # 增加尝试次数
                subprocess.run(f"pkill -f '{target}' 2>/dev/null", shell=True)
                subprocess.run(f"pkill -9 -f '{target}' 2>/dev/null", shell=True)  # 强制杀死
                time.sleep(0.2)  # 稍长的等待时间
    except Exception as e:
        print(f"[Warn] 清理系统进程失败: {e}")

    # 然后杀死所有子进程
    for i, p in enumerate(processes):
        if p and p.poll() is None:
            try:
                # 对于使用 setsid 的进程，发送信号到进程组
                if i < len(process_groups) and process_groups[i] != 0:
                    try:
                        os.killpg(process_groups[i], signal.SIGKILL)
                        try:
                            p.wait(timeout=0.3)
                        except:
                            pass
                    except:
                        pass
                else:
                    try:
                        p.kill()
                        try:
                            p.wait(timeout=0.3)
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
        time.sleep(0.5)
        print("[OK] 所有进程已彻底清理")
    except Exception as e:
        print(f"[Warn] 最终清理残留进程失败: {e}")

def signal_handler(sig, frame):
    print("\n[Stop] 正在停止小车...")
    # 立即停止所有进程，不等待停止指令处理
    print("[Stop] 立即停止所有节点...")
    kill_processes()

    print("[Stop] 所有节点已关闭")
    # 强制退出
    os._exit(0)

def run_command(cmd, name, use_setsid=True):  # 在 Linux 上支持 setsid，用于更好的进程管理
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
    print("[Car] 启动手持建图系统（rf2o激光里程计）")
    print("=" * 50)

    # 清理残留节点
    print("[Tools] 清理残留节点...")
    subprocess.run("pkill -f 'ydlidar_ros2_driver_node' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'slam_gmapping' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'hiwonder_motor_driver' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'simple_odom' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'simple_avoidance' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'rviz2' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'rf2o_laser_odometry' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'mlx90614_reader' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'temperature_marker' 2>/dev/null", shell=True)
    time.sleep(1)

    # 1. 激光雷达（使用手持建图方法）
    run_command("ros2 launch ydlidar_ros2_driver ydlidar_launch.py", "激光雷达")
    time.sleep(3)

    # 2. 静态TF：base_footprint -> base_link
    run_command("ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link", "base_footprint -> base_link TF")
    time.sleep(1)

    # 3. 静态TF：base_link -> laser_frame（正常安装）
    run_command("ros2 run tf2_ros static_transform_publisher 0.07 0.0075 0.024 0 0 0 base_link laser_frame", "雷达TF")
    time.sleep(1)

    # 3. RF2O 激光里程计（替代 simple_odom）
    run_command("ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py", "RF2O 激光里程计")
    time.sleep(3)

    # 4. 电机驱动（使用默认参数）
    run_command("ros2 run stm32_driver hiwonder_motor_driver", "电机驱动")
    time.sleep(2)

    # 5. 避障节点
    run_command("ros2 run yahboomcar_avoidance simple_avoidance", "避障节点")
    time.sleep(2)

    # 6. GMapping 建图节点（使用官方launch文件）
    run_command("ros2 launch slam_gmapping slam_gmapping_launch.py", "GMapping 建图")
    time.sleep(3)

    # 7. 温度传感器节点
    run_command("ros2 run mlx90614_reader mlx90614_reader", "温度传感器")
    time.sleep(2)

    # 8. 温度可视化标记节点
    run_command("ros2 run yahboomcar_mapping temperature_marker", "温度标记")
    time.sleep(2)

    # 11. RViz 可视化（使用 slam_gmapping 包中的配置）
    run_command("ros2 run rviz2 rviz2 -d " + os.path.abspath("src/slam_gmapping/rviz/map_view.rviz"), "RViz", use_setsid=False)

    print("\n[OK] 所有节点已启动")
    print("[Info] 使用手持建图方法（RF2O激光里程计）")
    print("[Info] RF2O优势：直接从激光雷达提取里程计，更准确")
    print("[Info] 在 RViz 中已加载 slam_gmapping 提供的配置")
    print("[Tips] 使用正常的gmapping建图，质量更好")
    print("[Tips] 按 i 键让小车前进，j/l 转向，k 停止")
    print("[Tips] 查看TF树：ros2 run tf2_tools view_frames")
    print("[Tips] 建图完成后保存：")
    print("   ros2 run nav2_map_server map_saver_cli -f ~/yahboomcar_ws/maps/rf2o_gmapping_map")

    # 优化等待和异常处理
    try:
        import threading

        def process_monitor():
            """监控进程状态的后台线程"""
            while True:
                time.sleep(0.5)
                # 检查所有进程是否都已结束
                all_finished = True
                for p in processes:
                    if p and p.poll() is None:
                        all_finished = False
                        break
                if all_finished:
                    print("所有进程已结束")
                    os._exit(0)

        # 启动监控线程
        monitor_thread = threading.Thread(target=process_monitor, daemon=True)
        monitor_thread.start()

        # 等待键盘中断
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
