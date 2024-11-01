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
        self.pose = Pose2D()  #自己位置
        self.startpose = Pose2D(0.35,0.35,0)  #初期位置
        
        # 2つのポールの位置を設定
        self.pole_positions = [
            {"x": 1.3, "y": 3.3, "radius": 0.25},  # ポール1
            {"x": 2.3, "y": 3.3, "radius": 0.25}  # ポール2
        ]
        
        # 設定可能なパラメータ
        self.grid_radius = 0.5    # グリッド半径（メートル）
        self.grid_spacing = 0.01  # グリッド間隔（メートル）
        self.angle_range = 5.0    # 角度範囲（度）
        self.angle_step = 0.5     # 角度ステップ（度）
        
        # サブスクライバーの設定
        self.sensor_sub = self.create_subscription(LaserScan, '/scan', self.sensor_callback, 10)
        self.pose_publisher = self.create_publisher(Pose2D, '/predicted_pose', 10)
        self.robot_vel_sub = self.create_subscription(Pose2D, '/robot_vel', self.robot_vel_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
    mesure_time = 0.1
        
    def robot_vel_callback(self, msg):
        # ロボットの速度を使って次の自己位置を予測
        self.pose.x += (msg.x*np.sin(self.pose.theta)+msg.y*np.cos(self.pose.theta))*self.mesure_time
        self.pose.y += (msg.y*np.sin(self.pose.theta)-msg.x*np.cos(self.pose.theta))*self.mesure_time
        self.pose.theta += msg.theta*self.mesure_time
        
    def joy_callback(self, msg):
        if msg.buttons[3] == 1:
            self.pose = self.startpose
            self.pose_publisher.publish(self.pose)

    def sensor_callback(self, msg):
        # ±100°範囲のクリップされたLaserScanデータ
        clipped_ranges = msg.ranges[int(135 - 100) * 4:int(135 + 100) * 4]
        
        # グリッド点を生成
        candidate_points = self.generate_grid_points()
        best_point, min_error = None, float('inf')

        # 各候補点について誤差計算
        for point in candidate_points:
            error_sum = 0.0
            for pole in self.pole_positions:
                predicted_distances = self.calculate_distances_to_pole(point, pole)
                actual_distance = self.get_lidar_distance_to_pole(clipped_ranges, point, pole)

                # 距離の誤差を累積
                error_sum += abs(predicted_distances - actual_distance)
            
            # 最小誤差を持つ点を選択
            if error_sum < min_error:
                min_error = error_sum
                best_point = point

        # 推定位置を更新
        if best_point:
            self.pose.x, self.pose.y = best_point
            self.pose_publisher.publish(self.pose)

    def generate_grid_points(self):
        """指定された半径と間隔でグリッド点を生成"""
        points = []
        for x in np.arange(-self.grid_radius, self.grid_radius, self.grid_spacing):
            for y in np.arange(-self.grid_radius, self.grid_radius, self.grid_spacing):
                if math.sqrt(x ** 2 + y ** 2) <= self.grid_radius:
                    points.append((self.pose.x + x, self.pose.y + y))
        return points

    def calculate_distances_to_pole(self, point, pole):
        """各候補点からポールまでの予測距離を、角度範囲に基づいて計算"""
        distances = []
        for angle_offset in np.arange(-self.angle_range, self.angle_range, self.angle_step):
            angle = math.atan2(pole["y"] - point[1], pole["x"] - point[0]) + math.radians(angle_offset)
            distances.append(math.sqrt((pole["x"] - point[0]) ** 2 + (pole["y"] - point[1]) ** 2))
        return min(distances)  # 最小値を採用

    def get_lidar_distance_to_pole(self, clipped_ranges, point, pole):
        """ポール方向に最も近いセンサーの距離を取得"""
        angle_to_pole = math.degrees(math.atan2(pole["y"] - point[1], pole["x"] - point[0]))
        angle_index = int((angle_to_pole + 135) / 0.25)
        return clipped_ranges[angle_index] if 0 <= angle_index < len(clipped_ranges) else float('inf')

def main(args=None):
    print('Hi from ykc_localization.')
    rclpy.init(args=args)
    node = SelfLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
