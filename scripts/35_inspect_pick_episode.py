"""Inspect the recorded pick episode npz file."""

import argparse

import numpy as np

from stage1_common import OUTPUT_DIR
from stage3_common import write_lines

parser = argparse.ArgumentParser()
parser.add_argument("--path", default=str(OUTPUT_DIR / "pick_episode_000.npz"))
args = parser.parse_args()

data = np.load(args.path, allow_pickle=True)
lines = ["===== INSPECT PICK EPISODE =====", f"path = {args.path}"]

for key in data.files:
    value = data[key]
    lines.append(f"{key}: shape={value.shape}, dtype={value.dtype}")

lines.extend(
    [
        f"object_lift_height = {float(data['object_lift_height']):.6f}",
        f"finger_contact = {bool(data['finger_contact'])}",
        f"success = {bool(data['success'])}",
    ]
)

print("\n".join(lines))
write_lines(OUTPUT_DIR / "pick_episode_000_inspect.txt", lines)
