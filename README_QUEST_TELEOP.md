# Quest UDP Teleoperation

Mục tiêu của phần này:

```text
Quest controller pose -> UDP -> Python trên Windows -> OpenArm MuJoCo
```

Phần này chưa train AI, chưa làm VLA. Đây là cầu nối teleoperation nhẹ, phù hợp máy Windows
không có quyền admin.

## Script Đã Tạo

```text
scripts/40_receive_quest_udp.py
scripts/41_send_fake_quest_udp.py
scripts/42_view_quest_udp_teleop_left.py
scripts/stage5_quest_common.py
```

## Bước 1: Test Nhận UDP

Mở terminal 1:

```powershell
.\.venv\Scripts\python.exe .\scripts\40_receive_quest_udp.py
```

Mở terminal 2 để giả lập Quest:

```powershell
.\.venv\Scripts\python.exe .\scripts\41_send_fake_quest_udp.py
```

Nếu đúng, terminal 1 sẽ in packet dạng:

```text
QuestControllerPacket(x=..., y=..., z=..., grip=...)
```

## Bước 2: Điều Khiển OpenArm Bằng UDP

Chạy viewer teleop:

```powershell
.\.venv\Scripts\python.exe .\scripts\42_view_quest_udp_teleop_left.py
```

Nếu chưa có Quest app, mở terminal khác và chạy fake sender:

```powershell
.\.venv\Scripts\python.exe .\scripts\41_send_fake_quest_udp.py
```

Tay trái robot sẽ đi theo packet giả lập.

## Packet JSON Mong Đợi

Quest app nên gửi UDP JSON:

```json
{"x": 0.0, "y": 0.0, "z": 0.0, "grip": 0.0}
```

Hoặc dạng nested:

```json
{"left": {"x": 0.0, "y": 0.0, "z": 0.0, "grip": 0.0}}
```

## Mapping Hiện Tại

```text
robot_x = origin_x + quest_x * scale
robot_y = origin_y + quest_z * scale
robot_z = origin_z + quest_y * scale
```

Default:

```text
origin = [0.36, 0.15, 1.12]
scale  = 0.6
```

## Gripper

Đã đo lại trong MuJoCo:

```text
left open  = 0.7854
left close = 0.0
```

Mapping:

```text
grip <= 0.5 -> open
grip >  0.5 -> close
```

## Khi Dùng Quest Thật

Nên dùng hotspot điện thoại hoặc Wi-Fi nhà:

```text
Quest và laptop phải cùng mạng
tránh Wi-Fi công ty nếu bị chặn thiết bị nói chuyện với nhau
```

Lấy IP laptop:

```powershell
ipconfig
```

Quest app gửi UDP tới:

```text
<IPv4 laptop>:5005
```

## Trạng Thái

Đã hoàn thành PC-side bridge:

```text
UDP receive: pass
fake UDP sender: pass
MuJoCo teleop viewer: ready
```

Bước tiếp theo là tạo app Quest gửi pose thật qua UDP.
