"""Reach to a safe approach point near the target cube."""

import numpy as np

from stage1_common import OUTPUT_DIR
from stage2_common import resolve_arm
from stage3_common import (
    cube_pos,
    left_ee_pos,
    load_object_scene,
    run_ik_to_pos,
    settle,
    write_lines,
)

scene = load_object_scene()
settle(scene)
left = resolve_arm(scene.model, "left")

object_pos = cube_pos(scene)
start_pos = left_ee_pos(scene)
target_pos = object_pos + np.array([-0.05, 0.0, 0.08])
success, distances = run_ik_to_pos(scene, left, target_pos, threshold=0.03)
final_pos = left_ee_pos(scene)

lines = [
    "===== REACH TO OBJECT LEFT =====",
    f"object_pos = {object_pos}",
    f"target_pos = {target_pos}",
    f"start_ee_pos = {start_pos}",
    f"final_ee_pos = {final_pos}",
    f"start_distance = {distances[0]:.6f}",
    f"final_distance = {distances[-1]:.6f}",
    f"success = {success}",
    f"steps_run = {len(distances)}",
]

print("\n".join(lines))
write_lines(OUTPUT_DIR / "reach_to_object_left.txt", lines)
print("\nSaved to outputs/reach_to_object_left.txt")
