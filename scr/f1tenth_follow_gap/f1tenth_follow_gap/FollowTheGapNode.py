#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import numpy as np
import math
from scipy.ndimage import gaussian_filter1d

class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')

        # Parámetros
        self.declare_parameter('max_steer_deg', 33.0)
        self.declare_parameter('bubble_radius', 110) #110
        self.declare_parameter('safety_margin_front', 1.2)
        self.declare_parameter('safety_margin_side', 1.4)
        self.declare_parameter('curve_slowdown_factor', 0.35)
        
        self.declare_parameter('max_straight_speed', 10.0)
        self.declare_parameter('min_curve_speed', 2.0)
        
        self.declare_parameter('enable_disparity_blur', True)
        self.declare_parameter('disparity_blur_kernel', 9.0)
        self.declare_parameter('disparity_blur_sigma', 2.0)

        self.max_steer = math.radians(self.get_parameter('max_steer_deg').value)
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.safety_front = self.get_parameter('safety_margin_front').value
        self.safety_side = self.get_parameter('safety_margin_side').value
        self.curve_factor_param = self.get_parameter('curve_slowdown_factor').value
        self.max_straight_speed = self.get_parameter('max_straight_speed').value
        self.min_curve_speed = self.get_parameter('min_curve_speed').value
        self.enable_blur = self.get_parameter('enable_disparity_blur').value
        self.blur_kernel = self.get_parameter('disparity_blur_kernel').value
        self.blur_sigma = self.get_parameter('disparity_blur_sigma').value

        # Suscripciones y publicadores
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/stop_flag', self.stop_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.current_velocity = 0.0
        self.angles = None
        self.stop_vehicle = False

    def stop_callback(self, msg):
        self.stop_vehicle = msg.data
        if self.stop_vehicle:
            self.get_logger().info("STOP_FLAG recibido. Vehículo detenido.")

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x

    # =============================
    # Métodos de procesamiento LIDAR
    # =============================
    def preprocess_lidar(self, ranges):
        proc = np.array(ranges)
        proc[np.isinf(proc)] = 30.0
        return np.clip(proc, 0.01, 30.0)

    def apply_disparity_blur(self, ranges):
        if not self.enable_blur or self.blur_kernel <= 1:
            return ranges
        disparity = np.copy(ranges)
        disparity[disparity < 0.3] = 0.0
        blurred = gaussian_filter1d(disparity, sigma=self.blur_sigma, mode='wrap')
        blurred[disparity < 0.3] = 0.0
        blurred = np.minimum(blurred, disparity)
        return blurred

    def find_max_gap(self, free_space):
        masked = np.ma.masked_where(free_space == 0, free_space)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = 0
        largest = None
        for s in slices:
            size = s.stop - s.start
            if size > max_len:
                max_len = size
                largest = s
        return largest.start, largest.stop if largest else (0, len(free_space))

    def find_best_point(self, start_i, end_i, ranges):
        if self.angles is None:
            return len(ranges) // 2
        d = np.array(ranges[start_i:end_i])
        a = self.angles[start_i:end_i]
        valid = d > 0.5
        if not np.any(valid):
            return len(ranges) // 2
        d = d[valid]
        a = a[valid]
        farthest_i = np.argmax(d)
        best_angle = a[farthest_i]
        weights = d ** 2
        avg_angle = np.average(a, weights=weights)
        final_angle = 0.7 * best_angle + 0.3 * avg_angle
        best_idx = np.argmin(np.abs(self.angles - final_angle))
        return best_idx

    def get_curvature_speed_factor(self, target_angle, ranges):
        n = len(ranges)
        idx_45L = int(n * (135 + 45) / 270)
        idx_90L = int(n * (135 + 90) / 270)
        idx_45R = int(n * (135 - 45) / 270)
        idx_90R = int(n * (135 - 90) / 270)
        d45L = ranges[idx_45L] if 0 <= idx_45L < n else 10.0
        d90L = ranges[idx_90L] if 0 <= idx_90L < n else 10.0
        d45R = ranges[idx_45R] if 0 <= idx_45R < n else 10.0
        d90R = ranges[idx_90R] if 0 <= idx_90R < n else 10.0

        if abs(target_angle) > math.radians(18):
            if target_angle > 0:
                lat = min(d45L, d90L)
            else:
                lat = min(d45R, d90R)
            if lat < self.safety_side:
                return max(0.2, lat / self.safety_side)
            elif lat < self.safety_side * 1.8:
                return 0.55
        return 1.0

    # =============================
    # Callback principal de LIDAR
    # =============================
    def scan_callback(self, scan_msg):
        if self.stop_vehicle:
            self.publish_drive(0.0, 0.0)
            return

        ranges = self.preprocess_lidar(scan_msg.ranges)
        ranges = self.apply_disparity_blur(ranges)

        if self.angles is None:
            n = len(ranges)
            self.angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, n)

        closest_idx = np.argmin(ranges)
        L = max(0, closest_idx - self.bubble_radius)
        R = min(len(ranges)-1, closest_idx + self.bubble_radius)
        ranges[L:R+1] = 0

        front = ranges[len(ranges)//2 - 30 : len(ranges)//2 + 30]
        if len(front) > 0 and np.min(front) < self.safety_front:
            self.publish_drive(0.0, 0.0)
            return

        start, end = self.find_max_gap(ranges)
        best_idx = self.find_best_point(start, end, ranges)
        target_angle = self.angles[best_idx]

        max_gap = np.max(ranges[start:end]) if end > start else 0

        if abs(target_angle) < math.radians(10) and max_gap > 8.0:
            base_speed = self.max_straight_speed
        elif abs(target_angle) < math.radians(20):
            base_speed = 0.4* (self.max_straight_speed + self.min_curve_speed)
        else:
            base_speed = self.min_curve_speed

        factor = self.get_curvature_speed_factor(target_angle, ranges)
        target_speed = base_speed * factor
        target_speed = max(self.min_curve_speed, target_speed)
        steer = np.clip(target_angle, -self.max_steer, self.max_steer)
        self.publish_drive(target_speed, steer)

    # =============================
    # Publicar velocidad y dirección
    # =============================
    def publish_drive(self, speed, steer):
        if self.stop_vehicle:
            speed = 0.0
            steer = 0.0
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)
        self.get_logger().info(
            f"Speed: {speed:5.2f} m/s | Steer: {math.degrees(steer):+6.1f}°",
            throttle_duration_sec=0.2
        )

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_drive(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

