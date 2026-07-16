# Demo: Xe chạy + né vật cản (LiDAR C1, ROS2 Jazzy)

## Kiến trúc

```
LiDAR C1 --/scan--> [obstacle_avoider] --/cmd_vel--> [base_bridge] --USB serial--> Arduino Mega --> L298N --> 2 motor
                                                          ^                             |
                                                          +---- "E tickL tickR" --------+
                                                          (base_bridge tính /odom + TF)
```

Đây là né vật cản **phản xạ** (reactive), không cần bản đồ — đúng cho milestone
"xe chạy được, thấy vật cản thì né". Sau này ghép Nav2 vào thì node
`obstacle_avoider` tắt đi, Nav2 sẽ publish `/cmd_vel` thay nó, còn
`base_bridge` giữ nguyên (tái sử dụng 100%).

## 1. Nạp code Arduino

- Mở `arduino/motor_bridge/motor_bridge.ino` bằng Arduino IDE, chọn board
  **Mega 2560**, nạp bình thường.
- Chân cắm khớp đúng bảng wire-by-wire trong Final1.docx. Nhớ **tháo jumper
  ENA/ENB** trên board L298N và **nối chung GND** Mega ↔ L298N.
- Test nhanh không cần ROS: mở Serial Monitor (115200, newline), gõ
  `V 120 120` → 2 bánh quay tới; gõ `S` → dừng. Đồng thời sẽ thấy dòng
  `E <tickL> <tickR>` nhảy số khi quay bánh bằng tay.
- Bánh nào quay ngược thì đảo 2 dây OUT của kênh đó (hoặc đổi `DIR_LEFT`/
  `DIR_RIGHT` thành `-1` trong code).

## 2. Cài trên Pi5X

```bash
sudo apt install ros-jazzy-sllidar-ros2 python3-serial
# nếu apt không có sllidar: clone https://github.com/Slamtec/sllidar_ros2 vào src rồi build chung

mkdir -p ~/ros2_ws/src
cp -r ros2_ws/src/strawberry_bot ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select strawberry_bot
source install/setup.bash
```

Cấp quyền cổng serial (làm 1 lần rồi logout/login):
```bash
sudo usermod -aG dialout $USER
```

## 3. Chạy demo

```bash
ros2 launch strawberry_bot avoid_demo.launch.py
```

Kiểm tra nhanh từng tầng nếu có trục trặc:
```bash
ros2 topic hz /scan        # LiDAR có dữ liệu chưa (~10Hz)
ros2 topic echo /cmd_vel   # avoider có ra lệnh không
ros2 topic echo /odom      # encoder về tới ROS chưa
# test motor không cần avoider:
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

## 4. BỐN SỐ PHẢI ĐO LẠI trước khi tin odom

Trong `avoid_demo.launch.py`, phần `base_bridge`:

| Tham số | Cách đo |
|---|---|
| `wheel_radius` | Đo đường kính bánh xích / 2 (m) |
| `wheel_separation` | Khoảng cách tâm 2 dải xích (m). Xe xích trượt khi xoay nên sau này chỉnh số này lớn hơn thực tế một chút cho khớp góc quay |
| `ticks_per_rev` | Quay bánh đúng 1 vòng bằng tay, xem tick nhảy bao nhiêu trên Serial Monitor. (Encoder 12V-35rpm loại phổ biến: 11 PPR × tỉ số hộp giảm tốc, ISR đếm cạnh lên kênh A nên x1) |
| `max_wheel_speed` | Cho `V 255 255`, đo xe đi được bao nhiêu mét trong 5 giây → chia 5 |

Riêng demo né vật cản thì odom sai chút cũng không sao (avoider chỉ dùng
/scan), nhưng đo sớm để sau này SLAM/Nav2 dùng luôn.

## 5. Chỉnh hành vi né

Chỉnh trực tiếp lúc đang chạy, không cần build lại:
```bash
ros2 param set /obstacle_avoider stop_dist 0.4
ros2 param set /obstacle_avoider cruise_speed 0.12
```

- Xe né quá muộn → tăng `slow_dist`, `stop_dist`
- Xe rung lắc, đổi hướng liên tục → giảm `turn_speed`, tăng `front_angle_deg`
- LiDAR gắn quay lưng về đầu xe → `lidar_yaw_offset_deg: 180.0`

## 6. An toàn (đã cài sẵn trong code)

1. **Arduino watchdog**: mất lệnh `V` quá 500ms → motor tự dừng (Pi treo,
   rớt USB đều an toàn).
2. **Avoider watchdog**: mất `/scan` quá 1s → publish cmd_vel = 0.
3. Tắt node (Ctrl+C) → tự gửi lệnh dừng trước khi thoát.
