"""Run push robustness trials for random left-side cube positions."""

import numpy as np

from stage1_common import OUTPUT_DIR
from stage3_common import run_random_push_objects, write_lines

SEGMENT = "left"
NUM_POSITIONS = 20
SEED = 31
SUCCESS_THRESHOLD = 0.03

rows, summary = run_random_push_objects(
    segment=SEGMENT,
    num_positions=NUM_POSITIONS,
    seed=SEED,
    success_threshold=SUCCESS_THRESHOLD,
)

header = [
    "trial_id",
    "requested_x",
    "requested_y",
    "requested_z",
    "object_start_x",
    "object_start_y",
    "object_start_z",
    "object_end_x",
    "object_end_y",
    "object_end_z",
    "push_distance_x",
    "success",
    "push_ik_success",
    "push_final_distance",
]

lines = [
    "===== PUSH RANDOM OBJECTS LEFT =====",
    f"num_positions = {int(summary['num_positions'])}",
    f"success_threshold = {summary['success_threshold']:.3f}",
    f"success_rate = {summary['success_rate']:.3f}",
    f"mean_push_distance_x = {summary['mean_push_distance_x']:.6f}",
    f"min_push_distance_x = {summary['min_push_distance_x']:.6f}",
    f"max_push_distance_x = {summary['max_push_distance_x']:.6f}",
    f"failed_cases = {int(summary['failed_cases'])}",
    "",
    ",".join(header),
]
lines.extend(",".join(str(value) for value in row) for row in rows)

np.savez_compressed(
    OUTPUT_DIR / "push_random_left_summary.npz",
    rows=np.array(rows, dtype=object),
    header=np.array(header),
    summary_keys=np.array(list(summary.keys())),
    summary_values=np.array(list(summary.values()), dtype=float),
)

print("\n".join(lines))
write_lines(OUTPUT_DIR / "push_random_left_summary.txt", lines)
print("\nSaved to outputs/push_random_left_summary.txt")
print("Saved to outputs/push_random_left_summary.npz")
