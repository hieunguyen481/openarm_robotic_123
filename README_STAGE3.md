# OpenArm MuJoCo - Giai đoạn 3

Báo cáo ngắn dễ đọc: `REPORT_STAGE3.md`

## Mục tiêu

Giai đoạn 3 chuyển từ reach task sang object interaction:

```text
reach -> reach-to-object -> touch -> push -> record push episode
```

Mục tiêu hiện tại chưa phải pick/grasp. Mục tiêu là thêm object, đọc object state,
chạm object, đẩy object và ghi dataset có robot + object + camera + action.

## Scene Mới

Không sửa trực tiếp `v2/cell.xml`. Script tạo scene mới:

```text
v2/cell_object.xml
```

Object được thêm:

```text
target_cube
target_cube_freejoint
target_cube_geom
```

Sau khi thêm cube:

```text
nq = 26
nv = 25
nu = 17
```

Lý do:

- Cube có `freejoint`
- `qpos` tăng thêm 7: position xyz + quaternion
- `qvel` tăng thêm 6: linear velocity + angular velocity
- `ctrl` vẫn là 17 vì action của robot không đổi

## Thứ Tự Chạy

```powershell
.\.venv\Scripts\python.exe .\scripts\18_create_object_scene.py
.\.venv\Scripts\python.exe .\scripts\19_inspect_object_scene.py
.\.venv\Scripts\python.exe .\scripts\20_log_object_position.py
.\.venv\Scripts\python.exe .\scripts\21_reach_to_object_left.py
.\.venv\Scripts\python.exe .\scripts\22_touch_object_left.py
.\.venv\Scripts\python.exe .\scripts\23_push_object_left.py
.\.venv\Scripts\python.exe .\scripts\24_record_push_episode.py
.\.venv\Scripts\python.exe .\scripts\25_inspect_push_episode.py
```

## Kết Quả Chính

### Object scene

Cube load thành công:

```text
target_cube id != -1
target_cube_freejoint id != -1
target_cube_geom id != -1
```

### Object position

Sau settle:

```text
object_pos ~= [0.42, 0.19, 1.0299]
```

### Reach-to-object

```text
final_distance = 0.029658
success = True
```

### Touch object

Touch có contact thật với cube:

```text
cube_contact = True
ncon = 9
```

Object cũng dịch chuyển nhẹ:

```text
object_displacement ~= [0.013, 0.003, 0.001]
```

### Push object

Cube được đẩy theo trục +x:

```text
push_distance_x = 0.060575
success = True
```

### Record push episode

File dataset:

```text
outputs/push_episode_000.npz
```

Shape:

```text
qpos               (1109, 26)
qvel               (1109, 25)
ctrl               (1109, 17)
left_ee_pos        (1109, 3)
right_ee_pos       (1109, 3)
object_pos         (1109, 3)
object_quat        (1109, 4)
rgb_images         (1109, 240, 320, 3)
depth_images       (1109, 240, 320)
distance_to_object (1109,)
success            True
```

Push distance trong recorded episode:

```text
push_distance_x = 0.060943
success = True
```

## Output Quan Trọng

```text
outputs/object_position.txt
outputs/reach_to_object_left.txt
outputs/touch_object_left.txt
outputs/push_object_left.txt
outputs/push_episode_000.txt
outputs/push_episode_000.npz
```

## Checklist

- [x] Tạo được `v2/cell_object.xml`
- [x] Load được scene có `target_cube`
- [x] Log được `object_pos`
- [x] Reach-to-object thành công
- [x] Touch object có contact với cube
- [x] Push object làm cube dịch chuyển
- [x] Record được `push_episode_000.npz`
- [x] Inspect được qpos/qvel/ctrl/rgb/depth/object_pos

## Ghi chú

- Giai đoạn này vẫn chưa làm pick/grasp.
- Push dễ hơn pick vì chỉ cần tiếp xúc và tạo lực ngang.
- Pick cần thêm gripper open/close, alignment, friction và lift success check.
## Giai Đoạn 3.5 - Push Robustness

Sau khi push một cube cố định đã chạy được, bước 3.5 dùng để kiểm tra robot có đẩy được
nhiều vị trí cube khác nhau hay không.

Chạy test 20 vị trí cube cho tay trái:

```powershell
.\.venv\Scripts\python.exe .\scripts\26_push_random_objects_left.py
```

Chạy test 20 vị trí cube cho tay phải:

```powershell
.\.venv\Scripts\python.exe .\scripts\27_push_random_objects_right.py
```

Tiêu chí success:

```text
push_distance_x > 0.03 m
```

Kết quả hiện tại:

```text
left_success_rate  = 0.900
right_success_rate = 0.950
```

Ý nghĩa: sau khi chỉnh waypoint để nhìn sát object hơn, tay trái đạt 18/20 case và tay phải
đạt 19/20 case, vẫn đạt ngưỡng robustness cơ bản.

## Ghi Dataset Push Nhiều Vị Trí

Script này ghi dataset nhiều episode nhưng chỉ lưu ảnh theo `image_stride` để tránh file quá
lớn:

```powershell
.\.venv\Scripts\python.exe .\scripts\28_record_push_dataset.py --segment left --num-episodes 5 --image-stride 10
.\.venv\Scripts\python.exe .\scripts\28_record_push_dataset.py --segment right --num-episodes 5 --image-stride 10
```

Output:

```text
outputs/push_dataset_left.npz
outputs/push_dataset_right.npz
```

Dataset có:

```text
episodes
rgb_images
depth_images
image_episode_ids
image_frame_ids
summary_rows
```

## Xem Trực Quan Trong Simulator

Muốn nhìn robot đẩy cube trực tiếp trong MuJoCo viewer:

```powershell
.\.venv\Scripts\python.exe .\scripts\29_view_push_task.py --segment left
```

Xem tay phải:

```powershell
.\.venv\Scripts\python.exe .\scripts\29_view_push_task.py --segment right
```

Xem một vị trí cube random:

```powershell
.\.venv\Scripts\python.exe .\scripts\29_view_push_task.py --segment left --random
```
