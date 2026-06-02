"""Hold the robot at home pose and report drift."""

import numpy as np

from stage2_common import OUTPUT_DIR, hold_ctrl, make_ready_model, site_position

model, data, home_ctrl = make_ready_model()
start_qpos = data.qpos.copy()
start_left = site_position(model, data, "left_ee_control_point")
start_right = site_position(model, data, "right_ee_control_point")

hold_ctrl(model, data, home_ctrl, steps=1000)

end_left = site_position(model, data, "left_ee_control_point")
end_right = site_position(model, data, "right_ee_control_point")

lines = [
    "===== HOME POSE CONTROL CHECK =====",
    f"home_ctrl_shape = {home_ctrl.shape}",
    f"home_ctrl = {home_ctrl}",
    f"qpos_max_abs_drift = {np.max(np.abs(data.qpos - start_qpos)):.8f}",
    f"left_ee_start = {start_left}",
    f"left_ee_end = {end_left}",
    f"left_ee_drift = {np.linalg.norm(end_left - start_left):.8f}",
    f"right_ee_start = {start_right}",
    f"right_ee_end = {end_right}",
    f"right_ee_drift = {np.linalg.norm(end_right - start_right):.8f}",
]

print("\n".join(lines))
(OUTPUT_DIR / "home_pose_check.txt").write_text("\n".join(lines), encoding="utf-8")
print("\nSaved to outputs/home_pose_check.txt")
