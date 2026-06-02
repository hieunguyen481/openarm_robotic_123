# OpenArm MuJoCo - Giai đoạn 4

Báo cáo ngắn dễ đọc: `REPORT_STAGE4.md`

## Mục tiêu

Giai đoạn 4 bắt đầu chuyển từ push sang pick/grasp:

```text
mở gripper -> align tới cube -> đóng gripper -> thử lift cube -> record pick episode
```

Pick khó hơn push rất nhiều. Vì vậy giai đoạn này ưu tiên kiểm tra từng phần nhỏ, không giả
định rằng cứ đóng gripper là sẽ nhấc được object.

## Thứ tự chạy

Test gripper open/close trong viewer:

```powershell
.\.venv\Scripts\python.exe .\scripts\30_test_gripper_open_close.py
```

Test headless:

```powershell
.\.venv\Scripts\python.exe .\scripts\30_test_gripper_open_close.py --headless-steps 300
```

Align gripper tới cube:

```powershell
.\.venv\Scripts\python.exe .\scripts\31_align_gripper_to_cube_left.py
```

Đóng gripper và kiểm tra contact:

```powershell
.\.venv\Scripts\python.exe .\scripts\32_grasp_cube_left.py
```

Thử lift cube:

```powershell
.\.venv\Scripts\python.exe .\scripts\33_pick_lift_cube_left.py
```

Record pick episode:

```powershell
.\.venv\Scripts\python.exe .\scripts\34_record_pick_episode.py
```

Inspect dataset:

```powershell
.\.venv\Scripts\python.exe .\scripts\35_inspect_pick_episode.py
```

Random pick test:

```powershell
.\.venv\Scripts\python.exe .\scripts\36_pick_random_objects_left.py --num-trials 5
```

Xem random pick nhiều vị trí trong viewer:

```powershell
.\.venv\Scripts\python.exe .\scripts\38_view_pick_random_left.py --num-trials 5
```

Chạy chậm hơn để quan sát kỹ:

```powershell
.\.venv\Scripts\python.exe .\scripts\38_view_pick_random_left.py --num-trials 5 --slow 2
```

Nếu muốn cube đổi vị trí rõ hơn:

```powershell
.\.venv\Scripts\python.exe .\scripts\38_view_pick_random_left.py --num-trials 5 --x-low 0.38 --x-high 0.48 --y-low 0.13 --y-high 0.25
```

Xem và tune pick trong viewer:

```powershell
.\.venv\Scripts\python.exe .\scripts\37_view_pick_tune_left.py
```

Nếu lần nhấc đầu bị hụt, script sẽ tự thử lại. Có thể tăng số lần thử:

```powershell
.\.venv\Scripts\python.exe .\scripts\37_view_pick_tune_left.py --retries 3
```

Thử chỉnh vị trí kẹp:

```powershell
.\.venv\Scripts\python.exe .\scripts\37_view_pick_tune_left.py --grasp-x 0.04 --grasp-y 0.0 --grasp-z 0.015
```

Gợi ý chỉnh:

- Nếu cổ tay/palm còn chạm cube: tăng độ cao đường vào bằng `--hover-z` hoặc `--pre-dz`.
- Nếu ngón tay chưa ôm vào cube: thử `--grasp-x 0.02` hoặc `--grasp-x 0.06`.
- Nếu ngón tay cao quá: giảm `--grasp-z`, ví dụ `0.005`.
- Nếu ngón tay thấp quá hoặc đẩy cube: tăng `--grasp-z`, ví dụ `0.025`.

## Kết quả hiện tại

Gripper range:

```text
left ctrl[8] open/close  = 0.7854 / 0.0
right ctrl[16] open/close = -0.7854 / 0.0
```

Align:

```text
pregrasp_success = True
grasp_success = True
```

Grasp riêng:

```text
cube_contact_with_left_finger = True
```

Lift:

```text
object_lift_height = 0.000000
success = False
```

Record pick episode:

```text
success = False sau khi chuyển sang clean-grasp mode
```

Random pick sau khi tune:

```text
success_rate = 0.400
```

## Dataset

Đã ghi được:

```text
outputs/pick_episode_000.npz
```

Shape chính:

```text
qpos          (945, 26)
qvel          (945, 25)
ctrl          (945, 17)
left_ee_pos   (945, 3)
object_pos    (945, 3)
object_quat   (945, 4)
rgb_images    (945, 240, 320, 3)
depth_images  (945, 240, 320)
phase         (945,)
```

## Trạng thái

Stage 4 hiện đã đạt phần kiểm tra gripper, align, grasp contact, lift và record dataset thành
công. Sau khi sửa đúng chiều open/close của gripper và tune `grasp_pos`, cube đã được nhấc
lên vượt ngưỡng 4 cm.

```text
grasp_pos
closing speed
finger friction
cube mass
gripper alignment
```

Đã xác nhận bản pick cũ nhấc được cube nhưng có palm/ee_base contact. Hiện đã chuyển sang
clean-grasp mode, ưu tiên không để cổ tay/palm chạm cube. Vì vậy lift tạm thời chưa đạt lại.
Bước tiếp theo là tune grasp sạch bằng fingertip midpoint trước khi sang Behavior Cloning/VLA.
