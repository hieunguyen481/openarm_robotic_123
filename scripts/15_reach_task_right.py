"""Run a complete right-arm reach task episode."""

import numpy as np

from stage2_common import OUTPUT_DIR, make_ready_model, resolve_arm, run_reach_episode

model, data, home_ctrl = make_ready_model()
right = resolve_arm(model, "right")

start_pos = data.site_xpos[right.site_id].copy()
target_pos = start_pos + np.array([0.04, -0.04, 0.03])
threshold = 0.03
result = run_reach_episode(
    model,
    data,
    home_ctrl,
    right,
    target_pos,
    threshold=threshold,
    max_steps=700,
)

final_pos = result.ee_pos_history[-1]
final_distance = float(result.distance_history[-1])

lines = [
    "===== RIGHT REACH TASK =====",
    f"target_pos = {target_pos}",
    f"start_pos = {start_pos}",
    f"final_pos = {final_pos}",
    f"start_distance = {result.distance_history[0]:.6f}",
    f"final_distance = {final_distance:.6f}",
    f"threshold = {threshold:.6f}",
    f"success = {result.success}",
    f"success_step = {result.success_step}",
    f"steps_run = {len(result.distance_history)}",
]

print("\n".join(lines))
(OUTPUT_DIR / "reach_task_right.txt").write_text("\n".join(lines), encoding="utf-8")
np.savez_compressed(
    OUTPUT_DIR / "reach_task_right.npz",
    target_pos=target_pos,
    start_pos=start_pos,
    final_pos=final_pos,
    distance_history=result.distance_history,
    right_ee_pos_history=result.ee_pos_history,
    ctrl_history=result.ctrl_history,
    qpos_history=result.qpos_history,
    qvel_history=result.qvel_history,
    success=np.array(result.success),
    success_step=np.array(result.success_step),
)
print("\nSaved to outputs/reach_task_right.txt")
print("Saved to outputs/reach_task_right.npz")
