# Báo cáo Stage 1 - Điều khiển robot cơ bản

## Mục đích chính

Stage 1 dùng để chứng minh:

```text
Robot OpenArm load được trong MuJoCo và có thể điều khiển chuyển động bằng ctrl.
```

Đây là nền tảng đầu tiên. Nếu chưa điều khiển được joint thì chưa thể làm reach, push hay
manipulation.

## Đã làm được gì nổi bật?

Đã xác định được kích thước state/action:

```text
qpos = 19
qvel = 19
ctrl = 17
camera = 5
```

Ý nghĩa:

- `qpos`: vị trí các joint trong simulator.
- `qvel`: vận tốc các joint.
- `ctrl`: lệnh điều khiển robot.
- Action đúng là `ctrl` 17 chiều.

Đã map được `ctrl[i]` điều khiển joint nào. Ví dụ:

```text
ctrl[1]  -> left_joint1
ctrl[4]  -> left_joint4
ctrl[9]  -> right_joint1
ctrl[12] -> right_joint4
```

Điểm quan trọng nhất: khi chạy test joint, robot thật sự di chuyển trong viewer.

```powershell
.\.venv\Scripts\python.exe .\scripts\04_test_joint_control.py --actuator 4 --amplitude 0.4 --freq 0.3
```

Lệnh này làm tay trái dao động. Đây là bằng chứng trực quan rằng:

```text
ctrl -> actuator -> joint -> robot chuyển động
```

## Dữ liệu tạo được

Đã ghi được observation dataset:

```text
outputs/episode_000.npz
```

Shape chính:

```text
qpos         (120, 19)
qvel         (120, 19)
ctrl         (120, 17)
rgb_images   (120, 240, 320, 3)
depth_images (120, 240, 320)
```

## Kết luận

Stage 1 đạt yêu cầu.

Robot load được, physics ổn, joint điều khiển được, camera render được, và dataset cơ bản
ghi được. Nói ngắn gọn: robot đã sẵn sàng để chuyển sang điều khiển theo target 3D.
