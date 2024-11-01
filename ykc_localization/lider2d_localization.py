import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Joy
import math
import numpy as np

class SelfLocalizationNode(Node):
    def __init__(self):
        super().__init__('lider2d_localization')
        self.pose = Pose2D()
        self.startpose = Pose2D()
        self.startpose.x = 0.35
        self.startpose.y = 0.35
        self.startpose.theta = 0.0
        self.mesure_time = 0.1
        
        # ポール位置と半径
        self.pole_positions = [
            {"x": 1.3, "y": 3.3, "radius": 0.23},
            {"x": 2.3, "y": 3.3, "radius": 0.23}
        ]
        
        # パラメータ設定
        self.grid_radius = 0.5
        self.grid_spacing = 0.01
        self.angle_range = 5.0
        self.angle_step = 0.5
        self.particle_count = 100
        
        # センサーデータの制限
        self.scan_range = (-70, 70)  # 使用範囲を±70°に設定
        self.scan_angle_resolution = 0.25  # 角度分解能
        self.eff_thre = 0.5
        
        # サブスクライバーとパブリッシャー
        self.sensor_sub = self.create_subscription(LaserScan, '/scan', self.sensor_callback, 10)
        self.pose_publisher = self.create_publisher(Pose2D, '/predicted_pose', 10)
        self.robot_vel_sub = self.create_subscription(Pose2D, '/robot_vel', self.robot_vel_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    mesure_time = 0.1

    def robot_vel_callback(self, msg):
        # ロボットの速度による予測更新
        self.pose.x += (msg.x * np.sin(self.pose.theta) + msg.y * np.cos(self.pose.theta)) * self.mesure_time
        self.pose.y += (msg.y * np.sin(self.pose.theta) - msg.x * np.cos(self.pose.theta)) * self.mesure_time
        self.pose.theta += msg.theta * self.mesure_time

    def joy_callback(self, msg):
        if msg.buttons[3] == 1:
            self.pose.x = self.startpose.x
            self.pose.y = self.startpose.y
            self.pose.theta = self.startpose.theta
            self.pose_publisher.publish(self.pose)

    def sensor_callback(self, msg):
        # LIDARデータを使用範囲にクリップ
        clipped_ranges = msg.ranges[int((135 + self.scan_range[0]) / self.scan_angle_resolution):
                                    int((135 + self.scan_range[1]) / self.scan_angle_resolution)]
        
        # パーティクル生成
        particles, weights = self.generate_particles(), np.ones(self.particle_count) / self.particle_count

        for i, particle in enumerate(particles):
            weights[i] = self.calculate_weight(particle, clipped_ranges)

        # 正規化と重み付き平均で自己位置推定
        weights /= np.sum(weights)
        estimated_x = np.sum([particle[0] * weight for particle, weight in zip(particles, weights)])
        estimated_y = np.sum([particle[1] * weight for particle, weight in zip(particles, weights)])
        
        self.pose.x, self.pose.y = estimated_x, estimated_y
        self.pose_publisher.publish(self.pose)

    def generate_particles(self):
        """指定範囲内にパーティクルを生成"""
        particles = []
        for _ in range(self.particle_count):
            x = self.pose.x + np.random.uniform(-self.grid_radius, self.grid_radius)
            y = self.pose.y + np.random.uniform(-self.grid_radius, self.grid_radius)
            theta = self.pose.theta + np.random.uniform(-math.radians(5), math.radians(5))
            particles.append((x, y, theta))
        return particles

    def calculate_weight(self, particle, clipped_ranges):
        """パーティクルごとに予測距離と実測値の誤差から重みを計算"""
        error_sum = 0.0
        for pole in self.pole_positions:
            predicted_distance = self.predict_distance_to_pole(particle, pole)
            actual_distance = self.get_lidar_distance(clipped_ranges, particle, pole)
            error_sum += (predicted_distance - actual_distance) ** 2

        return math.exp(-error_sum)

    def predict_distance_to_pole(self, particle, pole):
        """ポールの円形の外周に対する最小距離を計算"""
        min_distance = float('inf')
        num_points = 36  # 円周上の分割数
        for i in range(num_points):
            angle = i * (2 * math.pi / num_points)
            pole_edge_x = pole["x"] + pole["radius"] * math.cos(angle)
            pole_edge_y = pole["y"] + pole["radius"] * math.sin(angle)
            distance = math.sqrt((pole_edge_x - particle[0]) ** 2 + (pole_edge_y - particle[1]) ** 2)
            min_distance = min(min_distance, distance)
        return min_distance

    def get_lidar_distance(self, clipped_ranges, particle, pole):
        """実際のLIDAR測定値の距離を取得"""
        angle_to_pole = math.degrees(math.atan2(pole["y"] - particle[1], pole["x"] - particle[0])) - math.degrees(particle[2])
        angle_index = int((angle_to_pole + 135) / self.scan_angle_resolution)
        
        if 0 <= angle_index < len(clipped_ranges):
            return clipped_ranges[angle_index]
        return float('inf')

def main(args=None):
    rclpy.init(args=args)
    node = SelfLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
