# Báo cáo Stage 2 - Điều khiển đầu tay tới target 3D

## Mục đích chính

Stage 2 dùng để chứng minh:

```text
Robot có thể đưa đầu tay tới một điểm 3D mong muốn.
```

Stage 1 chỉ mới chứng minh điều khiển được từng joint. Stage 2 nâng lên mức cao hơn:

```text
target_xyz -> IK -> joint target -> ctrl -> đầu tay di chuyển
```

Đây là bước cần có trước khi robot chạm hoặc đẩy object.

## Đã làm được gì nổi bật?

Đã lấy được vị trí đầu tay trái và phải:

```text
left_ee_control_point  ~= [0.366,  0.1535, 1.125]
right_ee_control_point ~= [0.366, -0.1535, 1.125]
```

Đã viết và chạy Jacobian IK để đưa đầu tay tới target 3D.

Kết quả reach cơ bản:

```text
left reach  = success
right reach = success
```

Kết quả test nhiều target random:

```text
left_success_rate  = 1.000
right_success_rate = 1.000
threshold = 0.02 m
```

Ý nghĩa: cả hai tay đều reach tốt với các target gần quanh home pose.

## Dữ liệu tạo được

Đã ghi được reach episode:

```text
outputs/reach_episode_000.npz
```

Dataset có:

```text
qpos
qvel
ctrl
left_ee_pos
right_ee_pos
distance
rgb_images
depth_images
success
```

Điểm quan trọng là có `distance`, nên xem được quá trình đầu tay tiến gần target theo thời
gian.

## Xem trực quan

Chạy:

```powershell
.\.venv\Scripts\python.exe .\scripts\16_view_reach_task.py --segment left
```

hoặc:

```powershell
.\.venv\Scripts\python.exe .\scripts\16_view_reach_task.py --segment right
```

Nếu thấy đầu tay tự chạy tới một điểm gần đó, nghĩa là IK/reach hoạt động.

## Kết luận

Stage 2 đạt yêu cầu.

Robot không chỉ điều khiển từng joint nữa, mà đã có thể điều khiển đầu tay tới target 3D.
Đây là nền tảng trực tiếp cho object manipulation ở Stage 3.
