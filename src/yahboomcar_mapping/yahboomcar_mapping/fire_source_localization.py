#!/usr/bin/env python3
"""
火源定位节点 - 使用热源梯度算法实现火源位置计算
功能：
1. 订阅温度传感器数据
2. 订阅小车坐标（通过TF2获取map坐标系下的位置）
3. 缓存最近几十组温度+坐标数据
4. 用热源梯度法计算温度变化方向，推导出火源位置
5. 发布火源坐标话题
6. 发布火源RViz标记（黄色大球）
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math
from collections import deque
import numpy as np

# 配置参数
TEMP_BUFFER_SIZE = 50  # 温度数据缓存大小
TEMP_FIRE_THRESHOLD = 28.0  # 着火点温度阈值（降低以便更容易测试）
GRADIENT_THRESHOLD = 0.5  # 梯度阈值，用于判断温度变化方向
MARKER_SIZE = 0.8  # 火源标记大小（米）
MARKER_COLOR_R = 1.0  # 红色
MARKER_COLOR_G = 1.0  # 绿色（黄色 = 红色 + 绿色）
MARKER_COLOR_B = 0.0  # 蓝色
MARKER_ALPHA = 0.8  # 透明度

class FireSourceLocalizationNode(Node):
    def __init__(self):
        super().__init__('fire_source_localization')

        self.get_logger().info('🔥 火源定位节点已启动')
        self.get_logger().info(f'温度阈值: {TEMP_FIRE_THRESHOLD}°C')
        self.get_logger().info(f'数据缓存大小: {TEMP_BUFFER_SIZE}')

        # 创建订阅者
        self.temp_sub = self.create_subscription(
            Float32MultiArray, '/mlx90614/temperature_array',
            self.temperature_callback, 10)

        # 创建发布者
        self.fire_pose_pub = self.create_publisher(
            PoseStamped, '/fire_source_pose', 10)

        self.fire_marker_pub = self.create_publisher(
            Marker, '/fire_source_marker', 10)

        # TF2 监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 存储温度和坐标数据的缓存 (timestamp, x, y, temperature)
        self.temp_data_buffer = deque(maxlen=TEMP_BUFFER_SIZE)

        # 定时器用于计算火源位置（提高频率到10Hz）
        self.timer = self.create_timer(0.1, self.calculate_fire_position)

    def get_robot_position(self):
        """通过 TF2 获取机器人在 map 坐标系的位置"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # 计算朝向（yaw角）
            orientation = transform.transform.rotation
            siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            return x, y, theta
        except Exception as e:
            try:
                self.get_logger().debug(f"无法获取TF变换: {e}")
            except:
                pass
            return None, None, None

    def temperature_callback(self, msg):
        """温度数据回调函数"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        if len(msg.data) >= 4:
            left_ambient = msg.data[0]
            left_object = msg.data[1]
            right_ambient = msg.data[2]
            right_object = msg.data[3]

            # 获取当前机器人位置
            robot_x, robot_y, robot_theta = self.get_robot_position()

            if robot_x is not None and robot_y is not None:
                # 根据传感器是否探测到异常数据设置权重
                if left_object > TEMP_FIRE_THRESHOLD and right_object <= TEMP_FIRE_THRESHOLD:
                    # 左传感器探测到异常数据，右传感器未探测到
                    current_temp = left_object * 0.8 + right_object * 0.2
                elif right_object > TEMP_FIRE_THRESHOLD and left_object <= TEMP_FIRE_THRESHOLD:
                    # 右传感器探测到异常数据，左传感器未探测到
                    current_temp = right_object * 0.8 + left_object * 0.2
                else:
                    # 两个传感器都探测到异常数据或都未探测到
                    current_temp = (left_object + right_object) / 2

                self.temp_data_buffer.append({
                    'timestamp': current_time,
                    'x': robot_x,
                    'y': robot_y,
                    'temperature': current_temp,
                    'left_temp': left_object,
                    'right_temp': right_object
                })
                try:
                    self.get_logger().debug(
                        f'添加温度数据: ({robot_x:.2f}, {robot_y:.2f}) - {current_temp:.2f}°C')
                except:
                    pass

    def calculate_fire_position(self):
        """计算火源位置的主函数 - 优化稳定性"""
        try:
            # 检查缓冲区大小
            if len(self.temp_data_buffer) < 3:
                self.get_logger().debug('数据点不足，无法计算火源位置')
                self.publish_no_fire_marker()
                return

            # 提取数据
            xs = np.array([d['x'] for d in self.temp_data_buffer])
            ys = np.array([d['y'] for d in self.temp_data_buffer])
            temps = np.array([d['temperature'] for d in self.temp_data_buffer])

            # 检查是否有温度超过阈值的数据点
            if np.max(temps) < TEMP_FIRE_THRESHOLD:
                self.get_logger().debug('未检测到超过温度阈值的区域')
                self.publish_no_fire_marker()
                return

            # 找到所有超过温度阈值的点
            fire_indices = np.where(temps > TEMP_FIRE_THRESHOLD)[0]
            fire_xs = xs[fire_indices]
            fire_ys = ys[fire_indices]
            fire_temps = temps[fire_indices]

            # 找到独立的高温区域
            clusters = self.find_heat_clusters(fire_xs, fire_ys, fire_temps)

            if len(clusters) == 0:
                self.publish_no_fire_marker()
                return

            # 限制聚类数量，避免资源耗尽
            max_clusters = 5
            if len(clusters) > max_clusters:
                # 只保留温度最高的几个聚类
                clusters.sort(key=lambda c: np.max(c['temps']), reverse=True)
                clusters = clusters[:max_clusters]

            # 发布每个聚类的火源标记
            for i, cluster in enumerate(clusters):
                # 计算聚类的中心位置
                total_weight = np.sum(cluster['temps'])
                if total_weight > 0:
                    cluster_x = np.sum(cluster['xs'] * cluster['temps']) / total_weight
                    cluster_y = np.sum(cluster['ys'] * cluster['temps']) / total_weight
                else:
                    cluster_x = np.mean(cluster['xs'])
                    cluster_y = np.mean(cluster['ys'])

                try:
                    self.get_logger().info(
                        f'🔥 检测到火源 {i+1}: ({cluster_x:.2f}, {cluster_y:.2f}) - 最高温度: {np.max(cluster["temps"]):.1f}°C')
                except:
                    pass

                # 发布火源标记（为每个标记使用不同的ID）
                self.publish_fire_marker(cluster_x, cluster_y, i)

                # 如果是第一个聚类，也发布到/fire_source_pose话题
                if i == 0:
                    self.publish_fire_position(cluster_x, cluster_y)

        except Exception as e:
            self.get_logger().error(f'计算火源位置时出错: {e}')
            # 清除所有标记，防止错误累积
            self.publish_no_fire_marker()

    def calculate_heat_gradient(self, xs, ys, temps):
        """使用热源聚类算法计算火源位置"""
        # 找到所有超过温度阈值的点
        fire_indices = np.where(temps > TEMP_FIRE_THRESHOLD)[0]

        if len(fire_indices) == 0:
            return np.mean(xs), np.mean(ys)

        # 只使用超过温度阈值的点
        fire_xs = xs[fire_indices]
        fire_ys = ys[fire_indices]
        fire_temps = temps[fire_indices]

        # 检查是否有多个独立的高温区域（简单的距离阈值方法）
        clusters = self.find_heat_clusters(fire_xs, fire_ys, fire_temps)

        # 如果有多个聚类，只返回温度最高的聚类的位置
        if len(clusters) > 1:
            # 找到温度最高的聚类
            max_temp_cluster = max(clusters, key=lambda c: np.max(c['temps']))
            cluster_xs = max_temp_cluster['xs']
            cluster_ys = max_temp_cluster['ys']
            cluster_temps = max_temp_cluster['temps']
            self.get_logger().info(f'🔥 检测到 {len(clusters)} 个独立热源，选择温度最高的聚类')
        else:
            cluster_xs = fire_xs
            cluster_ys = fire_ys
            cluster_temps = fire_temps

        # 使用温度作为权重，计算加权平均位置
        total_weight = np.sum(cluster_temps)
        if total_weight > 0:
            weighted_x = np.sum(cluster_xs * cluster_temps) / total_weight
            weighted_y = np.sum(cluster_ys * cluster_temps) / total_weight
        else:
            weighted_x = np.mean(cluster_xs)
            weighted_y = np.mean(cluster_ys)

        # 获取温度最高点，用于调整位置
        max_temp_idx = np.argmax(cluster_temps)
        max_x = cluster_xs[max_temp_idx]
        max_y = cluster_ys[max_temp_idx]
        max_temp = cluster_temps[max_temp_idx]

        # 计算温度最高点与加权平均位置的距离
        distance = math.sqrt((max_x - weighted_x)**2 + (max_y - weighted_y)**2)

        # 如果距离较大，说明温度分布比较分散，应该更偏向温度最高点
        if distance > 0.5:
            # 根据温度高低调整权重
            temp_ratio = max_temp / np.mean(cluster_temps)
            alpha = min(temp_ratio / 2, 0.8)  # 权重介于0.5-0.8之间
            final_x = alpha * max_x + (1 - alpha) * weighted_x
            final_y = alpha * max_y + (1 - alpha) * weighted_y
        else:
            final_x = weighted_x
            final_y = weighted_y

        return final_x, final_y

    def find_heat_clusters(self, xs, ys, temps, distance_threshold=1.0):
        """简单的聚类算法，根据距离找到独立的高温区域"""
        clusters = []

        for i in range(len(xs)):
            x = xs[i]
            y = ys[i]
            temp = temps[i]

            # 检查是否属于已有的聚类
            found_cluster = False
            for cluster in clusters:
                cluster_center_x = np.mean(cluster['xs'])
                cluster_center_y = np.mean(cluster['ys'])
                distance = math.sqrt((x - cluster_center_x)**2 + (y - cluster_center_y)**2)

                if distance < distance_threshold:
                    cluster['xs'] = np.append(cluster['xs'], x)
                    cluster['ys'] = np.append(cluster['ys'], y)
                    cluster['temps'] = np.append(cluster['temps'], temp)
                    found_cluster = True
                    break

            if not found_cluster:
                # 创建新的聚类
                new_cluster = {
                    'xs': np.array([x]),
                    'ys': np.array([y]),
                    'temps': np.array([temp])
                }
                clusters.append(new_cluster)

        return clusters

    def publish_fire_position(self, x, y):
        """发布火源坐标"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.fire_pose_pub.publish(pose_msg)

    def publish_fire_marker(self, x, y, marker_id=0):
        """发布火源RViz标记（黄色大球）"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fire_source'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # 根据标记ID调整大小（可选，让不同的标记有不同的大小）
        scale = MARKER_SIZE * (1 + marker_id * 0.2)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = MARKER_COLOR_R
        marker.color.g = MARKER_COLOR_G
        marker.color.b = MARKER_COLOR_B
        marker.color.a = MARKER_ALPHA

        # 设置标记持续时间（0表示永久，直到被删除）
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        self.fire_marker_pub.publish(marker)

    def publish_no_fire_marker(self):
        """发布无火源的标记（删除之前的标记）"""
        # 发布一个DELETEALL动作来删除所有标记
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fire_source'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.DELETEALL

        self.fire_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = FireSourceLocalizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            try:
                node.get_logger().info('🛑 节点已停止')
            except:
                pass
    except Exception as e:
        if node:
            try:
                node.get_logger().error(f'节点异常: {e}')
            except:
                print(f'节点异常: {e}')
    finally:
        # 确保先销毁节点
        if node:
            try:
                node.destroy_node()
            except:
                pass
        # 检查 rclpy 是否还在运行，避免双重 shutdown 错误
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
