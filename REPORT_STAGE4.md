# Báo cáo Stage 4 - Pick / Grasp

## Mục đích chính

Stage 4 dùng để kiểm tra:

```text
Robot có thể dùng gripper để kẹp cube và nhấc cube khỏi bàn không?
```

Đây là bước khó hơn push. Push chỉ cần tạo lực ngang lên object, còn pick cần object nằm đúng
giữa hai ngón, có contact ổn định, đủ friction và không bị trượt khi lift.

## Đã làm được gì nổi bật?

Đã tạo chuỗi script từ số `30` theo đúng yêu cầu:

```text
30_test_gripper_open_close.py
31_align_gripper_to_cube_left.py
32_grasp_cube_left.py
33_pick_lift_cube_left.py
34_record_pick_episode.py
35_inspect_pick_episode.py
36_pick_random_objects_left.py
37_view_pick_tune_left.py
38_view_pick_random_left.py
```

Gripper đã điều khiển được:

```text
left ctrl[8]    open=0.7854, close=0.0
right ctrl[16]  open=-0.7854, close=0.0
```

Align tới cube đã chạy được:

```text
pregrasp_success = True
grasp_success = True
```

Grasp riêng đã có contact với finger:

```text
cube_contact_with_left_finger = True
```

Điều này chứng minh robot đã đưa gripper tới cube và tạo tiếp xúc vật lý với cube.

## Lift

Sau khi sửa đúng chiều open/close của gripper, bản pick cũ đã lift được cube nhưng có dấu hiệu
dựa vào palm/ee_base contact. Hiện đã chuyển sang clean-grasp mode, ưu tiên không để cổ tay
hoặc palm chạm cube:

```text
object_lift_height = 0.000000
success = False
```

Kết luận kỹ thuật: cần tune tiếp fingertip midpoint để nhấc cube bằng finger contact sạch,
không dựa vào palm contact.

## Dữ liệu tạo được

Đã record được:

```text
outputs/pick_episode_000.npz
```

Dataset có:

```text
qpos
qvel
ctrl
left_ee_pos
right_ee_pos
object_pos
object_quat
rgb_images
depth_images
phase
gripper_ctrl
object_height
ee_object_dist
object_lift_height
success
```

Episode hiện tại là episode debug, chưa phải episode pick thành công:

```text
success = False
```

## Có đạt yêu cầu Stage 4 chưa?

Chưa đạt bản sạch.

Đã đạt:

- Biết gripper open/close value.
- Điều khiển gripper được.
- Align gripper tới cube được.
- Đóng gripper có thể tạo contact với cube.
- Record và inspect được pick dataset.

Chưa đạt robust:

- Clean grasp chưa lift được cube.
- Cần tránh palm/ee_base contact khi gắp.
- Random pick chưa nên đánh giá cho đến khi clean grasp cố định đạt.

## Nên làm gì tiếp theo?

Tiếp theo nên test robustness:

```text
1. Chạy lại random pick nhiều vị trí.
2. Đo success_rate.
3. Nếu success_rate thấp, tune tiếp grasp_pos theo từng vùng cube.
4. Khi random pick ổn hơn, mới sang dataset collection / behavior cloning.
```

Stage 4 hiện đã có pick thành công cơ bản, nhưng chưa phải pick robust.

Script nên dùng để tune bằng mắt:

```powershell
.\.venv\Scripts\python.exe .\scripts\37_view_pick_tune_left.py
```

Script nên dùng để xem nhiều vị trí random:

```powershell
.\.venv\Scripts\python.exe .\scripts\38_view_pick_random_left.py --num-trials 5
```
