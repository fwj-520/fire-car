#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
import numpy as np
from collections import deque
import tf2_ros
from geometry_msgs.msg import TransformStamped

class AvoidanceMapping(Node):
    def __init__(self):
        super().__init__('avoidance_mapping')
        
        # QoS设置
        laser_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # 订阅/发布
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, laser_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # ========== 建图参数 ==========
        self.map_resolution = 0.05
        self.map_width = 800
        self.map_height = 800
        self.map_origin_x = -20.0
        self.map_origin_y = -20.0
        
        self.map = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        self.hits = np.zeros((self.map_height, self.map_width), dtype=np.int32)
        self.misses = np.zeros((self.map_height, self.map_width), dtype=np.int32)
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # ========== 避障参数（快速响应）==========
        self.safe_distance = 0.6
        self.danger_distance = 0.4
        
        # 速度设置：确保电机能转动
        self.min_speed = 0.5          # 电机最低启动速度（关键！）
        self.max_speed = 0.7          # 最大前进速度
        self.turn_speed = 0.8         # 转向速度
        self.back_speed = -0.5        # 后退速度
        
        # 平滑参数（极快响应）
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.accel = 3.0              # 加速度（快速响应）
        self.decel = 5.0              # 减速度
        
        # 状态
        self.scan_received = False
        self.scan_history = deque(maxlen=3)
        
        # 雷达安装补偿（左偏90度）
        self.laser_offset_rad = math.radians(90)
        
        # TF广播
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 计数器
        self.frame_count = 0
        
        # 定时器：控制频率50Hz（更平滑），建图2Hz（减轻负担）
        self.create_timer(0.02, self.control_timer_callback)   # 50Hz
        self.create_timer(0.5, self.mapping_timer_callback)    # 2Hz
        
        self.get_logger().info('='*60)
        self.get_logger().info('🚗 避障+建图融合节点 (快速响应版)')
        self.get_logger().info(f'   控制频率: 50Hz')
        self.get_logger().info(f'   最小速度: {self.min_speed}m/s')
        self.get_logger().info(f'   最大速度: {self.max_speed}m/s')
        self.get_logger().info('='*60)
        
    def scan_callback(self, scan):
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info('✅ 成功接收激光数据')
        
        self.scan_history.append(scan)
        self.calculate_target_speed(scan)
    
    def calculate_target_speed(self, scan):
        n = len(scan.ranges)
        
        def angle_to_index(angle_deg):
            compensated_angle = angle_deg - 90
            angle_rad = math.radians(compensated_angle)
            idx = int((angle_rad - scan.angle_min) / scan.angle_increment)
            return max(0, min(idx, n-1))
        
        def get_smooth_distance(center_idx, window=15):
            start = max(0, center_idx - window)
            end = min(n, center_idx + window)
            distances = []
            for i in range(start, end):
                d = scan.ranges[i]
                if not math.isinf(d) and not math.isnan(d) and d > 0.1:
                    distances.append(d)
            if distances:
                distances.sort()
                return distances[len(distances)//2]
            return float('inf')
        
        front_idx = angle_to_index(0)
        left_idx = angle_to_index(-90)
        right_idx = angle_to_index(90)
        
        front_dist = get_smooth_distance(front_idx)
        left_dist = get_smooth_distance(left_idx)
        right_dist = get_smooth_distance(right_idx)
        
        # 避障逻辑（立即生效）
        if front_dist < self.danger_distance:
            self.target_linear = self.back_speed
            self.target_angular = self.turn_speed if left_dist > right_dist else -self.turn_speed
            self.get_logger().info(f'🔥 后退 (前:{front_dist:.2f}m)')
        elif front_dist < self.safe_distance:
            self.target_linear = 0.0
            self.target_angular = self.turn_speed if left_dist > right_dist else -self.turn_speed
            self.get_logger().info(f'⚠️ 转向 (前:{front_dist:.2f}m)')
        else:
            # 安全区域：速度随距离增加，不低于最小速度
            speed_factor = min(1.0, (front_dist - self.safe_distance) / 1.5)
            self.target_linear = self.min_speed + (self.max_speed - self.min_speed) * speed_factor
            self.target_angular = 0.0
            self.get_logger().info(f'✅ 直行 {self.target_linear:.2f}m/s')
        
        # 限制范围
        self.target_linear = max(self.back_speed, min(self.max_speed, self.target_linear))
    
    def control_timer_callback(self):
        """50Hz 高速控制"""
        dt = 0.02
        
        # 极快速度响应
        if self.target_linear > self.current_linear:
            self.current_linear = min(self.target_linear, self.current_linear + self.accel * dt)
        else:
            self.current_linear = max(self.target_linear, self.current_linear - self.decel * dt)
        
        if self.target_angular > self.current_angular:
            self.current_angular = min(self.target_angular, self.current_angular + self.accel * dt)
        else:
            self.current_angular = max(self.target_angular, self.current_angular - self.decel * dt)
        
        # 发布指令
        cmd = Twist()
        cmd.linear.x = self.current_linear
        cmd.angular.z = self.current_angular
        self.cmd_pub.publish(cmd)
        
        # 更新机器人位置（用于建图）
        self.robot_x += self.current_linear * dt * math.cos(self.robot_theta)
        self.robot_y += self.current_linear * dt * math.sin(self.robot_theta)
        self.robot_theta += self.current_angular * dt
    
    def mapping_timer_callback(self):
        """2Hz 建图（降低频率，不影响避障）"""
        if not self.scan_history:
            return
        
        latest_scan = self.scan_history[-1]
        self.update_map(latest_scan)
        self.publish_map()
        self.publish_tf()
    
    def update_map(self, scan):
        n = len(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        robot_map_x = int((self.robot_x - self.map_origin_x) / self.map_resolution)
        robot_map_y = int((self.robot_y - self.map_origin_y) / self.map_resolution)
        
        if not (0 <= robot_map_x < self.map_width and 0 <= robot_map_y < self.map_height):
            return
        
        points = []
        for i in range(0, n, 4):  # 隔点采样
            d = scan.ranges[i]
            if math.isinf(d) or math.isnan(d) or d < 0.1 or d > 8.0:
                continue
            
            angle = angle_min + i * angle_increment + self.laser_offset_rad
            point_x = self.robot_x + d * math.cos(angle + self.robot_theta)
            point_y = self.robot_y + d * math.sin(angle + self.robot_theta)
            
            map_x = int((point_x - self.map_origin_x) / self.map_resolution)
            map_y = int((point_y - self.map_origin_y) / self.map_resolution)
            
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                points.append((map_x, map_y))
        
        # 标记占用
        for (map_x, map_y) in points:
            self.hits[map_y, map_x] += 1
            total = self.hits[map_y, map_x] + self.misses[map_y, map_x]
            prob = min(100, max(0, int(50 + 40 * (self.hits[map_y, map_x] - self.misses[map_y, map_x]) / (total + 1))))
            self.map[map_y, map_x] = prob
        
        # 标记空闲（简化版，减轻计算）
        step = 10
        for (map_x, map_y) in points[::20]:
            x, y = robot_map_x, robot_map_y
            dist = 0
            while (abs(x - map_x) > 1 or abs(y - map_y) > 1) and dist < 50:
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    if self.map[y, x] != 100:
                        self.misses[y, x] += 1
                        total = self.hits[y, x] + self.misses[y, x]
                        prob = min(100, max(0, int(50 - 40 * (self.misses[y, x] - self.hits[y, x]) / (total + 1))))
                        self.map[y, x] = prob
                if x < map_x:
                    x += step
                elif x > map_x:
                    x -= step
                if y < map_y:
                    y += step
                elif y > map_y:
                    y -= step
                dist += 1
    
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.orientation.w = 1.0
        
        # 噪声过滤
        filtered = self.map.copy()
        for y in range(1, self.map_height-1):
            for x in range(1, self.map_width-1):
                if self.map[y, x] == 100:
                    neighbors = 0
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            if 0 <= y+dy < self.map_height and 0 <= x+dx < self.map_width:
                                if self.map[y+dy, x+dx] == 100:
                                    neighbors += 1
                    if neighbors < 2:
                        filtered[y, x] = -1
        
        msg.data = filtered.flatten().tolist()
        self.map_pub.publish(msg)
    
    def publish_tf(self):
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = self.get_clock().now().to_msg()
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_link'
        t_odom_base.transform.translation.x = self.robot_x
        t_odom_base.transform.translation.y = self.robot_y
        t_odom_base.transform.rotation.z = math.sin(self.robot_theta / 2.0)
        t_odom_base.transform.rotation.w = math.cos(self.robot_theta / 2.0)
        
        t_base_laser = TransformStamped()
        t_base_laser.header.stamp = self.get_clock().now().to_msg()
        t_base_laser.header.frame_id = 'base_link'
        t_base_laser.child_frame_id = 'laser_frame'
        t_base_laser.transform.rotation.z = math.sin(self.laser_offset_rad / 2.0)
        t_base_laser.transform.rotation.w = math.cos(self.laser_offset_rad / 2.0)
        
        self.tf_broadcaster.sendTransform([t_odom_base, t_base_laser])

def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceMapping()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 正在停止...')
        stop_cmd = Twist()
        for i in range(5):
            node.cmd_pub.publish(stop_cmd)
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
