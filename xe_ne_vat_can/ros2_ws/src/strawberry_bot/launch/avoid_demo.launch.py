"""
Launch demo "xe chạy + né vật cản":
  1. Driver LiDAR C1 (sllidar_ros2)
  2. base_bridge  — cmd_vel -> Arduino, encoder -> odom
  3. obstacle_avoider — scan -> cmd_vel

Chạy: ros2 launch strawberry_bot avoid_demo.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Driver LiDAR SLAMTEC C1 — cần cài gói sllidar_ros2 trước
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',   # đổi nếu C1 nhận cổng khác
                'serial_baudrate': 460800,       # C1 dùng 460800
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen',
        ),

        Node(
            package='strawberry_bot',
            executable='base_bridge',
            name='base_bridge',
            parameters=[{
                'port': '/dev/ttyACM0',          # cổng Arduino Mega
                'baud': 115200,
                # === ĐO LẠI 4 SỐ NÀY TRÊN XE THẬT ===
                'wheel_radius': 0.0325,
                'wheel_separation': 0.18,
                'ticks_per_rev': 1320.0,
                'max_wheel_speed': 0.25,
                'min_pwm': 60,
            }],
            output='screen',
        ),

        Node(
            package='strawberry_bot',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            parameters=[{
                'cruise_speed': 0.15,
                'slow_speed': 0.07,
                'turn_speed': 0.6,
                'stop_dist': 0.35,
                'slow_dist': 0.80,
                'front_angle_deg': 60.0,
                'side_angle_deg': 60.0,
                'lidar_yaw_offset_deg': 0.0,
            }],
            output='screen',
        ),
    ])
