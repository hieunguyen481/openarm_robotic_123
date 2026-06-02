# Hướng Dẫn Chạy Trên Meta Quest

Mục tiêu của phần này là dùng tay cầm Meta Quest để điều khiển tay trái OpenArm trong MuJoCo theo thời gian thực.

Luồng dữ liệu hiện tại:

```text
Meta Quest Browser
-> webxr_quest/index.html
-> WebSocket ws://<IP laptop>:8765
-> scripts/50_ws_to_udp_bridge.py
-> UDP 127.0.0.1:5005
-> scripts/42_view_quest_udp_teleop_left.py
-> OpenArm MuJoCo viewer
```

## 1. Chuẩn Bị

Máy laptop cần có:

```text
Python >= 3.10
Git
Meta Quest và laptop cùng một mạng Wi-Fi/hotspot
```

Cài thư viện:

```powershell
cd C:\VScode\openarm_mujoco-master
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Nếu `websockets` bị chặn khi cài qua mạng công ty, hãy đổi sang Wi-Fi/hotspot khác.

## 2. Lấy IP Laptop

Chạy:

```powershell
ipconfig
```

Tìm dòng:

```text
IPv4 Address
```

Ví dụ:

```text
192.168.43.185
```

Trong hướng dẫn dưới đây, thay IP này bằng IP thật của laptop bạn.

## 3. Mở Web Server Cho Quest

Mở terminal 1:

```powershell
cd C:\VScode\openarm_mujoco-master\webxr_quest
C:\VScode\openarm_mujoco-master\.venv\Scripts\python.exe -m http.server 8000 --bind 0.0.0.0
```

Trên Meta Quest Browser, mở:

```text
http://192.168.43.185:8000/
```

Nếu trang hiện ra `Quest WebXR -> OpenArm MuJoCo` là đúng.

## 4. Chạy WebSocket Bridge

Mở terminal 2:

```powershell
cd C:\VScode\openarm_mujoco-master
.\.venv\Scripts\python.exe .\scripts\50_ws_to_udp_bridge.py
```

Bridge này nhận dữ liệu từ Quest qua WebSocket rồi chuyển sang UDP cho MuJoCo.

Mặc định:

```text
WebSocket: 0.0.0.0:8765
UDP:       127.0.0.1:5005
```

## 5. Chạy MuJoCo Viewer

Mở terminal 3:

```powershell
cd C:\VScode\openarm_mujoco-master
.\.venv\Scripts\python.exe .\scripts\42_view_quest_udp_teleop_left.py
```

Lúc này cửa sổ MuJoCo sẽ mở ra.

## 6. Kết Nối Từ Quest

Trên trang web trong Quest:

1. Kiểm tra ô WebSocket URL:

```text
ws://192.168.43.185:8765
```

2. Bấm:

```text
1. Connect WebSocket
```

Nếu đúng, trang sẽ báo:

```text
WebSocket connected
```

3. Bấm:

```text
2. Send Test Packet
```

Nếu đúng, terminal chạy bridge sẽ in ra gói tin, và terminal MuJoCo sẽ thấy target thay đổi.

4. Bấm:

```text
3. Enter VR / Start Controller Tracking
```

Sau đó di chuyển tay cầm trái. Tay trái robot trong MuJoCo sẽ di chuyển theo.

## 7. Điều Khiển

Mapping hiện tại:

```text
Quest trái/phải  -> robot x
Quest lên/xuống  -> robot z
Quest trước/sau  -> robot y
Grip tay trái    -> mở/đóng gripper
```

Gripper:

```text
grip <= 0.5 -> mở
grip >  0.5 -> đóng
```

Nếu robot đi quá xa hoặc quá nhạy, giảm scale:

```powershell
.\.venv\Scripts\python.exe .\scripts\42_view_quest_udp_teleop_left.py --scale 0.3
```

## 8. Nếu Quest Không Mở Được Trang

Kiểm tra:

```text
Quest và laptop có cùng Wi-Fi/hotspot không?
IP laptop có đúng không?
Terminal web server cổng 8000 còn chạy không?
Windows Firewall có chặn python.exe không?
```

Cho phép qua firewall:

1. Mở Start Menu.
2. Tìm `Allow an app through Windows Firewall`.
3. Bấm `Change settings`.
4. Bấm `Allow another app...`.
5. Chọn:

```text
C:\VScode\openarm_mujoco-master\.venv\Scripts\python.exe
```

6. Tích cả `Private` và `Public`.
7. Bấm `OK`.

Nếu vẫn lỗi, mở PowerShell bằng quyền Admin và chạy:

```powershell
netsh advfirewall firewall add rule name="OpenArm WebXR HTTP 8000" dir=in action=allow protocol=TCP localport=8000
netsh advfirewall firewall add rule name="OpenArm WebXR WS 8765" dir=in action=allow protocol=TCP localport=8765
```

## 9. Nếu WebXR Không Chạy

Nếu trang mở được nhưng bấm `Enter VR` báo:

```text
WebXR not available
```

thì nguyên nhân thường là trình duyệt không cho WebXR chạy qua HTTP thường.

Khi đó vẫn có thể test kết nối bằng nút:

```text
2. Send Test Packet
```

Nếu muốn tracking tay cầm thật ổn định hơn, hướng tốt hơn là làm app Quest bằng Unity/Meta XR SDK để gửi UDP trực tiếp về laptop.

## 10. Test Không Cần Quest

Nếu muốn kiểm tra MuJoCo trước:

Terminal 1:

```powershell
.\.venv\Scripts\python.exe .\scripts\42_view_quest_udp_teleop_left.py
```

Terminal 2:

```powershell
.\.venv\Scripts\python.exe .\scripts\41_send_fake_quest_udp.py
```

Nếu tay robot di chuyển trong viewer, phần nhận UDP và điều khiển MuJoCo đã hoạt động.
