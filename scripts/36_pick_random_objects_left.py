"""Run early random cube-position pick trials with the left arm."""

import argparse

import numpy as np

from stage1_common import OUTPUT_DIR
from stage3_common import write_lines
from stage4_common import LIFT_SUCCESS_HEIGHT, run_pick_left, sample_pick_positions

parser = argparse.ArgumentParser()
parser.add_argument("--num-trials", type=int, default=5)
parser.add_argument("--seed", type=int, default=61)
args = parser.parse_args()

positions = sample_pick_positions(num_positions=args.num_trials, seed=args.seed)
rows = []

for trial_id, position in enumerate(positions):
    result = run_pick_left(cube_position=position, record=False)
    rows.append(
        [
            trial_id,
            *position.tolist(),
            *result.object_start.tolist(),
            *result.object_end.tolist(),
            result.object_lift_height,
            result.finger_contact,
            result.success,
        ]
    )

successes = [row[-1] for row in rows]
lift_heights = [row[-3] for row in rows]
success_rate = float(np.mean(successes))

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
    "object_lift_height",
    "finger_contact",
    "success",
]

lines = [
    "===== PICK RANDOM OBJECTS LEFT =====",
    f"num_trials = {args.num_trials}",
    f"success_threshold = {LIFT_SUCCESS_HEIGHT:.6f}",
    f"success_rate = {success_rate:.3f}",
    f"mean_lift_height = {float(np.mean(lift_heights)):.6f}",
    f"max_lift_height = {float(np.max(lift_heights)):.6f}",
    "",
    ",".join(header),
]
lines.extend(",".join(str(value) for value in row) for row in rows)

np.savez_compressed(
    OUTPUT_DIR / "pick_random_left_summary.npz",
    rows=np.array(rows, dtype=object),
    header=np.array(header),
    success_rate=np.array(success_rate),
)

print("\n".join(lines))
write_lines(OUTPUT_DIR / "pick_random_left_summary.txt", lines)
