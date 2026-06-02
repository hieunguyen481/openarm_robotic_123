# OpenArm MuJoCo - Giai đoạn 1

Báo cáo ngắn dễ đọc: `REPORT_STAGE1.md`

Mục tiêu của giai đoạn này là hiểu model OpenArm trong MuJoCo:

- Robot đang load từ file XML nào
- Robot có bao nhiêu trạng thái `qpos`, `qvel`
- Robot có bao nhiêu lệnh điều khiển `ctrl`
- `ctrl[i]` điều khiển joint nào
- Camera nào render được RGB/depth
- Ghi được dữ liệu mô phỏng ra file `.npz`

## XML Đang Dùng

File scene chính:

```text
C:\VScode\openarm_mujoco-master\v2\cell.xml
```

Đây là scene mặc định trả về bởi:

```python
openarm_mujoco.v2.openarm_cell_xml()
```

Scene này gồm OpenArm bimanual trong OpenArm Cell, có tay trái, tay phải,
lifter, bàn/cell và camera.

## Thông Tin Model

Kết quả chính:

- `nq = 19`
- `nv = 19`
- `nu = 17`
- `nbody = 22`
- `njnt = 19`
- `nsite = 2`
- `ncam = 5`

Ý nghĩa:

- `qpos`: vị trí/trạng thái joint, có 19 chiều
- `qvel`: vận tốc joint, có 19 chiều
- `ctrl`: lệnh điều khiển actuator, có 17 chiều

Với controller hoặc policy sau này:

```text
state position: qpos in R^19
state velocity: qvel in R^19
action/control: ctrl in R^17
```

Tại sao `qpos` có 19 nhưng `ctrl` chỉ có 17?

Vì model có 19 giá trị vị trí joint, nhưng chỉ có 17 actuator điều khiển trực tiếp.
Hai joint dư ra là finger joint thứ hai của mỗi gripper. Chúng được coupled/mimic
theo finger joint chính, nên không có control riêng.

File thông tin đầy đủ:

```text
outputs/model_info.txt
```

## Bảng Actuator Mapping

Bảng này cho biết `ctrl[i]` điều khiển joint nào.

| ctrl index | actuator name | joint name | ctrl range | ghi chú |
|---|---|---|---|---|
| 0 | lifter_ctrl | openarm_lifter_joint | [0.0, 0.3] | nâng/hạ lifter |
| 1 | left_joint1_ctrl | openarm_left_joint1 | [-3.49066, 1.39626] | tay trái joint 1 |
| 2 | left_joint2_ctrl | openarm_left_joint2 | [-3.31613, 0.174533] | tay trái joint 2 |
| 3 | left_joint3_ctrl | openarm_left_joint3 | [-1.5708, 1.5708] | tay trái joint 3 |
| 4 | left_joint4_ctrl | openarm_left_joint4 | [0.0, 2.44346] | tay trái joint 4 |
| 5 | left_joint5_ctrl | openarm_left_joint5 | [-1.5708, 1.5708] | tay trái joint 5 |
| 6 | left_joint6_ctrl | openarm_left_joint6 | [-0.785398, 0.785398] | tay trái joint 6 |
| 7 | left_joint7_ctrl | openarm_left_joint7 | [-1.5708, 1.5708] | tay trái joint 7 |
| 8 | left_finger1_ctrl | openarm_left_finger_joint1 | [0.0, 0.7854] | gripper trái |
| 9 | right_joint1_ctrl | openarm_right_joint1 | [-1.39626, 3.49066] | tay phải joint 1 |
| 10 | right_joint2_ctrl | openarm_right_joint2 | [-0.174533, 3.31613] | tay phải joint 2 |
| 11 | right_joint3_ctrl | openarm_right_joint3 | [-1.5708, 1.5708] | tay phải joint 3 |
| 12 | right_joint4_ctrl | openarm_right_joint4 | [0.0, 2.44346] | tay phải joint 4 |
| 13 | right_joint5_ctrl | openarm_right_joint5 | [-1.5708, 1.5708] | tay phải joint 5 |
| 14 | right_joint6_ctrl | openarm_right_joint6 | [-0.785398, 0.785398] | tay phải joint 6 |
| 15 | right_joint7_ctrl | openarm_right_joint7 | [-1.5708, 1.5708] | tay phải joint 7 |
| 16 | right_finger1_ctrl | openarm_right_finger_joint1 | [-0.7854, 0.0] | gripper phải |

File mapping đầy đủ:

```text
outputs/actuator_map.txt
```

Kết quả test từng actuator:

```text
outputs/joint_control_summary.txt
```

## Cách Test Joint Control

Script test joint dùng `base_ctrl` lấy từ `qpos` hiện tại, không ép toàn bộ control
về zero. Như vậy robot dao động quanh tư thế home/hiện tại:

```python
data.ctrl[:] = base_ctrl
data.ctrl[actuator_id] = base_ctrl[actuator_id] + amplitude * sin(...)
```

Giá trị control cũng được clip theo `ctrlrange`, để không gửi lệnh vượt giới hạn
của actuator.

Các actuator dễ nhìn khi test bằng viewer:

- `ctrl[1]`: vai/tay trái joint 1
- `ctrl[4]`: tay trái joint 4, dễ thấy chuyển động
- `ctrl[7]`: cổ tay trái joint 7
- `ctrl[9]`: vai/tay phải joint 1
- `ctrl[12]`: tay phải joint 4, dễ thấy chuyển động
- `ctrl[15]`: cổ tay phải joint 7

Ví dụ chạy:

```powershell
.\.venv\Scripts\python.exe .\scripts\04_test_joint_control.py --actuator 4 --amplitude 0.4 --freq 0.3
```

## Body, Site, Camera

Site end-effector:

- `left_ee_control_point`
- `right_ee_control_point`

Camera trong model:

- `camera_ceiling`
- `camera_head_left`
- `camera_head_right`
- `camera_wrist_left`
- `camera_wrist_right`

Script ghi dataset mặc định dùng `camera_ceiling`, nhưng có thể chọn camera khác:

```powershell
.\.venv\Scripts\python.exe .\scripts\06_record_observation.py --camera camera_wrist_left --output episode_wrist_left.npz
```

Đã test cả 5 camera render RGB/depth thành công.

Kết quả:

```text
outputs/camera_test.txt
outputs/camera_test.npz
outputs/camera_tests/
```

## Dataset Output

File dataset đã tạo:

```text
outputs/episode_000.npz
```

Shape dữ liệu:

- `qpos`: `(120, 19)`, `float64`
- `qvel`: `(120, 19)`, `float64`
- `ctrl`: `(120, 17)`, `float64`
- `rgb_images`: `(120, 240, 320, 3)`, `uint8`
- `depth_images`: `(120, 240, 320)`, `float32`

Ý nghĩa:

- `120`: số frame/timestep đã ghi
- `19`: số chiều `qpos` hoặc `qvel`
- `17`: số chiều action/control
- `240 x 320`: kích thước ảnh
- `3`: ba kênh màu RGB

Mỗi timestep có thể hiểu là:

```text
observation_t = qpos, qvel, rgb image, depth image
action_t = ctrl
```

Đây mới là dataset kiểm tra pipeline. Nó chưa phải dataset hành vi tốt để train
robot làm task.

## Thứ Tự Chạy Chuẩn

Chạy các bước tự động:

```powershell
.\.venv\Scripts\python.exe .\scripts\00_check_install.py
.\.venv\Scripts\python.exe .\scripts\01_find_xml.py
.\.venv\Scripts\python.exe .\scripts\02_inspect_model.py
.\.venv\Scripts\python.exe .\scripts\03_passive_sim.py --headless-steps 1000
.\.venv\Scripts\python.exe .\scripts\05_map_actuators.py
.\.venv\Scripts\python.exe .\scripts\04_test_joint_control.py --headless-all
.\.venv\Scripts\python.exe .\scripts\06_record_observation.py
.\.venv\Scripts\python.exe .\scripts\07_inspect_npz.py
.\.venv\Scripts\python.exe .\scripts\08_test_cameras.py
```

Chạy viewer để tự quan sát:

```powershell
.\.venv\Scripts\python.exe .\scripts\03_passive_sim.py
.\.venv\Scripts\python.exe .\scripts\04_test_joint_control.py --actuator 4
.\.venv\Scripts\python.exe .\scripts\04_test_joint_control.py --actuator 1 --amplitude 0.4 --freq 0.3
.\.venv\Scripts\python.exe .\scripts\04_test_joint_control.py --actuator 12 --amplitude 0.4 --freq 0.3
```

## Checklist

- [x] Chạy được `openarm-mujoco-launch`
- [x] Tìm được XML chính
- [x] Load XML bằng `mujoco.MjModel.from_xml_path`
- [x] In được `nq`, `nv`, `nu`
- [x] In được joint list
- [x] In được actuator list
- [x] In được body/site/camera list
- [x] Passive simulation headless không sinh NaN/nổ physics
- [x] Test được từng `ctrl[i]` bằng headless sweep
- [x] Tạo được actuator-to-joint map
- [x] Render được RGB image
- [x] Render được depth image
- [x] Test được toàn bộ camera có tên
- [x] Log được `episode_000.npz`
- [x] Inspect được shape của `qpos`, `qvel`, `ctrl`, `rgb`, `depth`
- [ ] Tự quan sát passive simulation trong viewer
- [ ] Tự quan sát từng actuator chuyển động trong viewer

## Ghi Chú

- `openarm_mujoco` là namespace package, nên `openarm_mujoco.__file__` có thể là
  `None`. Module v2 thật nằm ở `src\openarm_mujoco\v2\__init__.py`.
- Headless sweep xác nhận tất cả actuator đều làm state thay đổi. Nhưng mô tả
  bằng mắt như “vai xoay”, “khuỷu gập”, “cổ tay quay” vẫn nên quan sát trong viewer.
- Action đúng hiện tại là `ctrl` 17 chiều, không phải 19 chiều.
