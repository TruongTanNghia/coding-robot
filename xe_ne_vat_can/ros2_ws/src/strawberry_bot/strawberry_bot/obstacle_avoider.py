#!/usr/bin/env python3
"""
obstacle_avoider — né vật cản phản xạ (reactive) bằng LiDAR C1.

Logic 3 vùng:
  - Phía trước THÔNG THOÁNG (> slow_dist)      -> chạy thẳng tốc độ cruise
  - Có vật trong vùng CHẬM (stop_dist..slow_dist) -> giảm tốc + lái dần về phía thoáng hơn
  - Vật quá GẦN (< stop_dist)                   -> dừng tiến, xoay tại chỗ về phía thoáng
    (xích quay tại chỗ được nên tận dụng luôn)

Subscribe:  /scan  (sensor_msgs/LaserScan)
Publish  :  /cmd_vel (geometry_msgs/Twist)

Chạy: ros2 run strawberry_bot obstacle_avoider
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # ---- Tham số chỉnh được lúc chạy ----
        self.declare_parameter('cruise_speed', 0.15)   # m/s khi thoáng
        self.declare_parameter('slow_speed', 0.07)     # m/s khi có vật gần
        self.declare_parameter('turn_speed', 0.6)      # rad/s khi xoay né
        self.declare_parameter('stop_dist', 0.35)      # m — gần hơn mức này thì dừng & xoay
        self.declare_parameter('slow_dist', 0.80)      # m — gần hơn mức này thì giảm tốc
        self.declare_parameter('front_angle_deg', 60.0)  # góc quạt phía trước (±30°)
        self.declare_parameter('side_angle_deg', 60.0)   # bề rộng quạt trái/phải
        self.declare_parameter('lidar_yaw_offset_deg', 0.0)  # nếu lidar gắn lệch hướng đầu xe

        self.timer = self.create_timer(0.5, self._watchdog)
        self.last_scan_time = None

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.get_logger().info('Obstacle avoider sẵn sàng — chờ /scan...')

    # Nếu mất /scan quá 1s thì phát cmd_vel = 0 (Arduino cũng có watchdog riêng)
    def _watchdog(self):
        if self.last_scan_time is None:
            return
        dt = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
        if dt > 1.0:
            self.pub.publish(Twist())
            self.get_logger().warn('Mất tín hiệu LiDAR > 1s, dừng xe!')

    # ---------- Tiện ích ----------
    def sector_min(self, scan: LaserScan, ang_from, ang_to):
        """Khoảng cách nhỏ nhất trong quạt [ang_from, ang_to] (radian, 0 = trước mặt)."""
        offset = math.radians(self.get_parameter('lidar_yaw_offset_deg').value)
        n = len(scan.ranges)
        best = float('inf')
        for i in range(n):
            a = scan.angle_min + i * scan.angle_increment + offset
            # chuẩn hóa về (-pi, pi]
            a = math.atan2(math.sin(a), math.cos(a))
            if ang_from <= a <= ang_to:
                r = scan.ranges[i]
                if scan.range_min < r < scan.range_max and not math.isinf(r) and not math.isnan(r):
                    best = min(best, r)
        return best

    # ---------- Vòng điều khiển chính ----------
    def on_scan(self, scan: LaserScan):
        self.last_scan_time = self.get_clock().now()

        half_front = math.radians(self.get_parameter('front_angle_deg').value) / 2.0
        side_w = math.radians(self.get_parameter('side_angle_deg').value)

        d_front = self.sector_min(scan, -half_front, half_front)
        d_left = self.sector_min(scan, half_front, half_front + side_w)
        d_right = self.sector_min(scan, -half_front - side_w, -half_front)

        stop_d = self.get_parameter('stop_dist').value
        slow_d = self.get_parameter('slow_dist').value
        cruise = self.get_parameter('cruise_speed').value
        slow = self.get_parameter('slow_speed').value
        turn = self.get_parameter('turn_speed').value

        cmd = Twist()

        if d_front < stop_d:
            # Quá gần: dừng tiến, xoay tại chỗ về phía thoáng hơn
            cmd.linear.x = 0.0
            cmd.angular.z = turn if d_left > d_right else -turn
            state = 'XOAY-NÉ'
        elif d_front < slow_d:
            # Vùng chậm: vừa đi vừa lái lệch về phía thoáng
            # Hệ số 0..1: càng gần vật càng lái mạnh
            k = (slow_d - d_front) / (slow_d - stop_d)
            cmd.linear.x = slow + (cruise - slow) * (1.0 - k)
            steer = turn * 0.7 * k
            cmd.angular.z = steer if d_left > d_right else -steer
            state = 'GIẢM TỐC + LÁI'
        else:
            cmd.linear.x = cruise
            cmd.angular.z = 0.0
            state = 'THẲNG'

        self.pub.publish(cmd)
        self.get_logger().debug(
            f'{state} | trước={d_front:.2f} trái={d_left:.2f} phải={d_right:.2f} '
            f'-> v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}'
        )


def main():
    rclpy.init()
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Dừng xe khi tắt node
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
