"""Push the target cube along +x with the left arm."""

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

object_start = cube_pos(scene)
approach_pos = object_start + np.array([-0.14, -0.02, 0.08])
touch_pos = object_start + np.array([-0.10, -0.02, 0.05])
push_pos = object_start + np.array([0.02, -0.02, 0.05])

run_ik_to_pos(scene, left, approach_pos, threshold=0.035)
run_ik_to_pos(scene, left, touch_pos, threshold=0.03)
success_push_ik, push_distances = run_ik_to_pos(
    scene,
    left,
    push_pos,
    threshold=0.035,
    max_steps=1000,
)

object_end = cube_pos(scene)
object_displacement = object_end - object_start
push_distance_x = float(object_displacement[0])
success = push_distance_x > 0.02

lines = [
    "===== PUSH OBJECT LEFT =====",
    f"object_start_pos = {object_start}",
    f"object_end_pos = {object_end}",
    f"object_displacement = {object_displacement}",
    f"push_distance_x = {push_distance_x:.6f}",
    f"final_ee_pos = {left_ee_pos(scene)}",
    f"push_ik_success = {success_push_ik}",
    f"push_final_distance = {push_distances[-1]:.6f}",
    f"success = {success}",
]

print("\n".join(lines))
write_lines(OUTPUT_DIR / "push_object_left.txt", lines)
print("\nSaved to outputs/push_object_left.txt")
