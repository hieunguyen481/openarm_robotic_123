"""Run random nearby right-arm reach targets and summarize robustness."""

import numpy as np

from stage2_common import OUTPUT_DIR, run_random_reach_targets

segment = "right"
num_targets = 20
threshold = 0.02
min_start_distance = 0.04
max_steps = 1000
rows, summary = run_random_reach_targets(
    segment=segment,
    num_targets=num_targets,
    threshold=threshold,
    min_start_distance=min_start_distance,
    seed=11,
    max_steps=max_steps,
)

lines = [
    "===== RIGHT REACH RANDOM TARGETS =====",
    f"num_targets = {int(summary['num_targets'])}",
    f"threshold = {summary['threshold']:.6f}",
    f"min_start_distance = {summary['min_start_distance']:.6f}",
    f"success_rate = {summary['success_rate']:.3f}",
    f"mean_final_distance = {summary['mean_final_distance']:.6f}",
    f"min_final_distance = {summary['min_final_distance']:.6f}",
    f"max_final_distance = {summary['max_final_distance']:.6f}",
    f"mean_steps_to_success = {summary['mean_steps_to_success']:.3f}",
    f"skipped_close_targets = {int(summary['skipped_close_targets'])}",
    f"max_steps = {int(summary['max_steps'])}",
    f"gain = {summary['gain']:.3f}",
    f"damping = {summary['damping']:.3f}",
    f"max_step = {summary['max_step']:.3f}",
    "",
    "id,target_x,target_y,target_z,start_distance,final_distance,success,success_step,steps_run,attempts",
]
for row in rows:
    lines.append(",".join(str(x) for x in row))

print("\n".join(lines))
(OUTPUT_DIR / "reach_random_right_summary.txt").write_text(
    "\n".join(lines), encoding="utf-8"
)
np.savez_compressed(
    OUTPUT_DIR / "reach_random_right_summary.npz",
    rows=np.array(rows, dtype=object),
    summary=np.array(summary, dtype=object),
    success_rate=np.array(summary["success_rate"]),
)
print("\nSaved to outputs/reach_random_right_summary.txt")
print("Saved to outputs/reach_random_right_summary.npz")
