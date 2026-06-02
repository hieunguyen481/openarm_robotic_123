# OpenArm MuJoCo - Giai đoạn 2

Báo cáo ngắn dễ đọc: `REPORT_STAGE2.md`

Mục tiêu giai đoạn này:

- Lấy được vị trí end-effector trái/phải trong không gian 3D
- Giữ robot ở home pose ổn định
- Điều khiển tay bằng target joint pose
- Dùng Jacobian IK để đưa `left_ee_control_point` tới gần target 3D
- Chạy và log một reach task đơn giản
- Kiểm tra thêm độ bền bằng nhiều target gần nhau

## Kết Luận Nhanh

Với scene hiện tại:

```text
left_ee_control_point  ~= [0.366,  0.1535, 1.125]
right_ee_control_point ~= [0.366, -0.1535, 1.125]
```

Action vẫn là:

```text
ctrl in R^17
```

Reach target mặc định trong script là target gần vị trí ban đầu của tay trái:

```text
target_pos = left_ee_start + [0.04, 0.04, 0.03]
```

Kết quả reach cơ bản:

```text
start_distance = 0.064031 m
final_distance = 0.029881 m
threshold = 0.030000 m
success = True
```

Kết quả Stage 2.5 robustness test với threshold chặt hơn `0.02 m` và bỏ qua
target quá gần ban đầu (`start_distance < 0.04 m`):

```text
Left arm, 20 random targets:
success_rate = 1.000
mean_final_distance = 0.019839 m
max_final_distance = 0.019966 m
mean_steps_to_success = 204.300

Right arm, 20 random targets:
success_rate = 1.000
mean_final_distance = 0.019802 m
max_final_distance = 0.019999 m
mean_steps_to_success = 203.650
```

## Các File Script

```text
scripts/08_log_ee_position.py
scripts/09_home_pose_control.py
scripts/10_joint_pose_controller.py
scripts/11_left_arm_jacobian_ik.py
scripts/12_reach_task_left.py
scripts/13_record_reach_episode.py
scripts/14_reach_random_targets_left.py
scripts/15_reach_task_right.py
scripts/16_view_reach_task.py
scripts/17_reach_random_targets_right.py
```

## Thứ Tự Chạy

```powershell
.\.venv\Scripts\python.exe .\scripts\08_log_ee_position.py
.\.venv\Scripts\python.exe .\scripts\09_home_pose_control.py
.\.venv\Scripts\python.exe .\scripts\10_joint_pose_controller.py
.\.venv\Scripts\python.exe .\scripts\11_left_arm_jacobian_ik.py
.\.venv\Scripts\python.exe .\scripts\12_reach_task_left.py
.\.venv\Scripts\python.exe .\scripts\13_record_reach_episode.py
.\.venv\Scripts\python.exe .\scripts\14_reach_random_targets_left.py
.\.venv\Scripts\python.exe .\scripts\15_reach_task_right.py
.\.venv\Scripts\python.exe .\scripts\17_reach_random_targets_right.py
```

Ghi episode bằng camera khác:

```powershell
.\.venv\Scripts\python.exe .\scripts\13_record_reach_episode.py --camera camera_head_left --output reach_episode_head_left.npz
.\.venv\Scripts\python.exe .\scripts\13_record_reach_episode.py --camera camera_wrist_left --output reach_episode_wrist_left.npz
```

Xem reach task trực tiếp trong MuJoCo viewer:

```powershell
.\.venv\Scripts\python.exe .\scripts\16_view_reach_task.py --segment left
.\.venv\Scripts\python.exe .\scripts\16_view_reach_task.py --segment right
```

## Output Quan Trọng

```text
outputs/ee_position.txt
outputs/home_pose_check.txt
outputs/joint_pose_control.txt
outputs/left_ik_trace.txt
outputs/left_ik_trace.npz
outputs/reach_task_left.txt
outputs/reach_task_left.npz
outputs/reach_episode_000.txt
outputs/reach_episode_000.npz
outputs/reach_random_left_summary.txt
outputs/reach_random_left_summary.npz
outputs/reach_task_right.txt
outputs/reach_task_right.npz
outputs/reach_random_right_summary.txt
outputs/reach_random_right_summary.npz
outputs/reach_episode_head_left.npz
outputs/reach_episode_wrist_left.npz
```

## Ý Tưởng IK

Robot nhận control theo joint:

```text
data.ctrl[left_joint_ctrl_ids] = target_joint_positions
```

Nhưng task muốn điều khiển theo tọa độ:

```text
target_pos = [x, y, z]
```

Vì vậy script dùng Jacobian:

```text
target_xyz -> Jacobian IK -> target joint positions -> data.ctrl
```

Mỗi bước:

1. Đọc vị trí hiện tại của `left_ee_control_point`
2. Tính `error = target_pos - current_pos`
3. Tính Jacobian vị trí bằng `mujoco.mj_jacSite`
4. Tính `dq` bằng damped least squares
5. Cập nhật target joint và clip theo joint/ctrl range
6. Gọi `mujoco.mj_step`

## Checklist

- [x] Lấy được `left_ee_pos` / `right_ee_pos`
- [x] Giữ được home pose bằng `home_ctrl`
- [x] Điều khiển được tay trái bằng target joint pose
- [x] Điều khiển được tay phải bằng target joint pose
- [x] Viết được Jacobian IK cho left arm
- [x] `left_ee` đi gần target 3D
- [x] Có distance giảm dần
- [x] Có success khi distance < threshold
- [x] Log được `reach_episode_000.npz`
- [x] Log đủ `distance_history`, `ee_pos_history`, `ctrl_history`, `qpos_history`, `qvel_history`
- [x] Test 20 target random gần tay trái
- [x] Đạt success rate > 70% với target gần ở threshold `0.02 m`
- [x] Test reach task tay phải
- [x] Test 20 target random gần tay phải
- [x] Đạt right-arm success rate >= 0.8 ở threshold `0.02 m`
- [x] Record episode bằng `camera_head_left`
- [x] Record episode bằng `camera_wrist_left`
- [x] Có script xem reach trực tiếp trong viewer

## Ghi Chú

- Đây là IK vị trí 3D cơ bản, chưa điều khiển orientation của end-effector.
- Target đang được chọn gần home pose để kiểm tra pipeline trước.
- Giai đoạn sau có thể mở rộng sang target ngẫu nhiên, right arm IK, bimanual task,
  hoặc task gắp vật.
- Random reach hiện đạt tốt với target gần. Với task khó hơn, vẫn nên cải thiện
  controller bằng target filtering, null-space posture control, orientation IK
  hoặc planner.
