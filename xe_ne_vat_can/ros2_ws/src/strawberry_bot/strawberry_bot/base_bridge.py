#!/usr/bin/env python3
"""
base_bridge — cầu nối giữa ROS2 và Arduino Mega.

  /cmd_vel (Twist)  ->  "V pwmL pwmR\n"  qua USB serial
  "E tickL tickR"   ->  /odom (Odometry) + TF odom->base_link

Chạy: ros2 run strawberry_bot base_bridge
"""
import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import serial


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class BaseBridge(Node):
    def __init__(self):
        super().__init__('base_bridge')

        # ---- Tham số: ĐO LẠI TRÊN XE THẬT rồi chỉnh trong launch file ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.0325)      # m — bán kính bánh xích
        self.declare_parameter('wheel_separation', 0.18)    # m — khoảng cách 2 dải xích
        self.declare_parameter('ticks_per_rev', 1320.0)     # xung encoder / 1 vòng bánh
        self.declare_parameter('max_wheel_speed', 0.25)     # m/s tại PWM=255 (đo thực tế)
        self.declare_parameter('min_pwm', 60)               # PWM nhỏ hơn mức này motor không quay

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.tpr = self.get_parameter('ticks_per_rev').value
        self.vmax = self.get_parameter('max_wheel_speed').value
        self.min_pwm = self.get_parameter('min_pwm').value

        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self.get_logger().info(f'Đã mở serial {self.port} @ {self.baud}')

        # ---- ROS I/O ----
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_bc = TransformBroadcaster(self)

        # ---- Trạng thái odom ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_ticks = None
        self.prev_time = self.get_clock().now()

        # Thread đọc serial riêng để không nghẽn executor
        self._stop = False
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    # ---------- cmd_vel -> PWM ----------
    def on_cmd_vel(self, msg: Twist):
        v = msg.linear.x          # m/s
        w = msg.angular.z         # rad/s
        # Động học vi sai: tốc độ dài của mỗi dải xích
        v_l = v - w * self.L / 2.0
        v_r = v + w * self.L / 2.0

        pwm_l = self.speed_to_pwm(v_l)
        pwm_r = self.speed_to_pwm(v_r)
        try:
            self.ser.write(f'V {pwm_l} {pwm_r}\n'.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi ghi serial: {e}')

    def speed_to_pwm(self, v):
        """Ánh xạ tuyến tính m/s -> PWM, có bù vùng chết (deadband)."""
        if abs(v) < 1e-4:
            return 0
        pwm = int(round(abs(v) / self.vmax * 255.0))
        pwm = max(self.min_pwm, min(255, pwm))
        return pwm if v > 0 else -pwm

    # ---------- Encoder -> Odom ----------
    def read_loop(self):
        while not self._stop and rclpy.ok():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
            except (serial.SerialException, OSError, TypeError):
                # cổng serial bị đóng khi node tắt -> thoát êm
                if self._stop:
                    break
                continue
            if not line.startswith('E'):
                continue
            parts = line.split()
            if len(parts) != 3:
                continue
            try:
                tl, tr = int(parts[1]), int(parts[2])
            except ValueError:
                continue
            self.update_odom(tl, tr)

    def update_odom(self, tl, tr):
        now = self.get_clock().now()
        if self.prev_ticks is None:
            self.prev_ticks = (tl, tr)
            self.prev_time = now
            return

        dtl = tl - self.prev_ticks[0]
        dtr = tr - self.prev_ticks[1]
        self.prev_ticks = (tl, tr)

        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0.0:
            return

        # Quãng đường mỗi bánh
        dist_per_tick = 2.0 * math.pi * self.R / self.tpr
        d_l = dtl * dist_per_tick
        d_r = dtr * dist_per_tick

        d = (d_l + d_r) / 2.0
        dyaw = (d_r - d_l) / self.L

        self.x += d * math.cos(self.yaw + dyaw / 2.0)
        self.y += d * math.sin(self.yaw + dyaw / 2.0)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw),
                              math.cos(self.yaw + dyaw))

        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = d / dt
        odom.twist.twist.angular.z = dyaw / dt
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_bc.sendTransform(t)

    def destroy_node(self):
        self._stop = True
        self.reader.join(timeout=0.5)   # chờ thread đọc thoát rồi mới đóng cổng
        try:
            self.ser.write(b'S\n')
            self.ser.flush()
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = BaseBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
