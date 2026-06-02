"""Move both arms to simple target joint poses with position actuators."""

import numpy as np

import mujoco

from stage2_common import (
    OUTPUT_DIR,
    actuator_summary,
    clip_ctrl,
    make_ready_model,
    resolve_arm,
    set_arm_ctrl,
)

model, data, home_ctrl = make_ready_model()
left = resolve_arm(model, "left")
right = resolve_arm(model, "right")

target_ctrl = home_ctrl.copy()
left_target = data.qpos[left.qpos_ids].copy()
right_target = data.qpos[right.qpos_ids].copy()

left_target[0] += 0.20
left_target[3] += 0.25
left_target[6] += 0.15
right_target[0] -= 0.20
right_target[3] += 0.25
right_target[6] -= 0.15

for alpha in np.linspace(0.0, 1.0, 500):
    data.ctrl[:] = home_ctrl
    set_arm_ctrl(
        model,
        data,
        left,
        (1.0 - alpha) * data.qpos[left.qpos_ids] + alpha * left_target,
    )
    set_arm_ctrl(
        model,
        data,
        right,
        (1.0 - alpha) * data.qpos[right.qpos_ids] + alpha * right_target,
    )
    data.ctrl[:] = clip_ctrl(model, data.ctrl)
    mujoco.mj_step(model, data)

left_error = np.linalg.norm(data.qpos[left.qpos_ids] - left_target)
right_error = np.linalg.norm(data.qpos[right.qpos_ids] - right_target)

lines = [
    "===== JOINT POSE CONTROLLER =====",
    "Left arm actuators:",
    *actuator_summary(model, left.ctrl_ids),
    "Right arm actuators:",
    *actuator_summary(model, right.ctrl_ids),
    f"left_target = {left_target}",
    f"right_target = {right_target}",
    f"left_final = {data.qpos[left.qpos_ids]}",
    f"right_final = {data.qpos[right.qpos_ids]}",
    f"left_joint_error_norm = {left_error:.6f}",
    f"right_joint_error_norm = {right_error:.6f}",
]

print("\n".join(lines))
(OUTPUT_DIR / "joint_pose_control.txt").write_text("\n".join(lines), encoding="utf-8")
print("\nSaved to outputs/joint_pose_control.txt")
