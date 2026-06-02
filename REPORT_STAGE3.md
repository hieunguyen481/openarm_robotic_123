# Báo cáo Stage 3 - Tương tác object và push

## Mục đích chính

Stage 3 dùng để chứng minh:

```text
Robot có thể tương tác vật lý với object trong MuJoCo.
```

Stage 2 chỉ đưa tay tới target trong không khí. Stage 3 thêm cube thật vào scene để kiểm tra:

```text
đưa tay tới object -> chạm object -> tạo contact -> đẩy object di chuyển
```

Đây là bước chuyển từ reach sang manipulation.

## Đã làm được gì nổi bật?

Đã tạo scene mới:

```text
v2/cell_object.xml
```

Cube có `freejoint`, nên object có state riêng:

```text
nq: 19 -> 26
nv: 19 -> 25
ctrl: vẫn là 17
```

Ý nghĩa:

- Robot vẫn dùng action `ctrl` 17 chiều.
- Cube có vị trí và quaternion riêng trong state.
- Có thể log cả robot state và object state.

Robot đã chạm cube thật, có contact giữa cube và collision geometry của tay:

```text
target_cube_geom với ee/finger/link collision
```

Push đơn hiện tại:

```text
push_distance_x = 0.058425 m
success = True
```

Ý nghĩa: cube bị đẩy đi khoảng 5.8 cm theo trục `+x`.

## Stage 3.5: kiểm tra nhiều vị trí cube

Để không chỉ test một vị trí cố định, đã random nhiều vị trí cube quanh vùng thao tác.

Tiêu chí success:

```text
push_distance_x > 0.03 m
```

Kết quả:

```text
left_success_rate  = 0.900
right_success_rate = 0.950
```

Ý nghĩa:

- Tay trái push thành công 18/20 vị trí.
- Tay phải push thành công 19/20 vị trí.
- Cả hai vượt tiêu chí cơ bản `success_rate >= 0.7`.

## Dữ liệu tạo được

Push episode đơn:

```text
outputs/push_episode_000.npz
```

Dataset push nhiều vị trí:

```text
outputs/push_dataset_left.npz
outputs/push_dataset_right.npz
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
success
```

## Xem trực quan

Chạy:

```powershell
.\.venv\Scripts\python.exe .\scripts\29_view_push_task.py --segment left
```

Test object xa hơn:

```powershell
.\.venv\Scripts\python.exe .\scripts\29_view_push_task.py --segment left --far
```

Nếu thấy tay robot chạm cube và cube dịch chuyển, nghĩa là Stage 3 thành công.

## Kết luận

Stage 3 đạt yêu cầu.

Robot đã đi được từ reach sang manipulation cơ bản: có object thật, có contact thật, đẩy
object di chuyển được, và ghi được dataset có robot state + object state + camera + action.
