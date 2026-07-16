"""
Launch demo "xe chạy + né vật cản":
  1. Driver LiDAR C1 (sllidar_ros2)
  2. base_bridge  — cmd_vel -> Arduino, encoder -> odom
  3. obstacle_avoider — scan -> cmd_vel

Chạy: ros2 launch strawberry_bot avoid_demo.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Cổng LiDAR: mặc định trỏ theo serial CP2102N của C1 (đường dẫn by-id
        # cố định, không bị nhảy ttyUSB0/1 khi cắm lại). Đổi lúc chạy nếu cần:
        #   ros2 launch strawberry_bot avoid_demo.launch.py lidar_port:=/dev/ttyUSB1
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/serial/by-id/'
                          'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                          '96929910127fef118cef221cedd322a4-if00-port0'),

        # Driver LiDAR SLAMTEC C1 — cần cài gói sllidar_ros2 trước
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 460800,       # C1 dùng 460800
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen',
        ),

        # TF tĩnh base_link -> laser: demo né chưa cần, nhưng SLAM/RViz2 bắt buộc.
        # Đo lại x (m, LiDAR lệch trước/sau tâm xe) và z (độ cao LiDAR) trên xe thật.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.10',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
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
