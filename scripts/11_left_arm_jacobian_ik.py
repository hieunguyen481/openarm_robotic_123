"""Move left end-effector toward a target with Jacobian IK."""

import numpy as np

import mujoco

from stage2_common import OUTPUT_DIR, jacobian_ik_step, make_ready_model, resolve_arm

model, data, home_ctrl = make_ready_model()
left = resolve_arm(model, "left")

start_pos = data.site_xpos[left.site_id].copy()
target_pos = start_pos + np.array([0.04, 0.04, 0.03])
distances = []

for _ in range(500):
    data.ctrl[:] = home_ctrl
    distance = jacobian_ik_step(model, data, left, target_pos)
    distances.append(distance)
    mujoco.mj_step(model, data)

final_pos = data.site_xpos[left.site_id].copy()
final_distance = float(np.linalg.norm(target_pos - final_pos))

np.savez_compressed(
    OUTPUT_DIR / "left_ik_trace.npz",
    target_pos=target_pos,
    start_pos=start_pos,
    final_pos=final_pos,
    distances=np.array(distances),
)

lines = [
    "===== LEFT ARM JACOBIAN IK =====",
    f"start_pos = {start_pos}",
    f"target_pos = {target_pos}",
    f"final_pos = {final_pos}",
    f"start_distance = {distances[0]:.6f}",
    f"final_distance = {final_distance:.6f}",
    f"improvement = {distances[0] - final_distance:.6f}",
]

print("\n".join(lines))
(OUTPUT_DIR / "left_ik_trace.txt").write_text("\n".join(lines), encoding="utf-8")
print("\nSaved to outputs/left_ik_trace.txt")
print("Saved to outputs/left_ik_trace.npz")
