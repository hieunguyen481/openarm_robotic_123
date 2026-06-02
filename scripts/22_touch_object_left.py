"""Move the left end-effector to touch the target cube."""

import numpy as np

import mujoco

from stage1_common import OUTPUT_DIR
from stage2_common import resolve_arm
from stage3_common import (
    contact_names,
    cube_pos,
    has_cube_contact,
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
approach_pos = object_start + np.array([-0.05, 0.0, 0.08])
touch_pos = object_start + np.array([-0.025, 0.0, 0.04])

run_ik_to_pos(scene, left, approach_pos, threshold=0.03)
success, distances = run_ik_to_pos(scene, left, touch_pos, threshold=0.025)

for _ in range(100):
    mujoco.mj_step(scene.model, scene.data)

object_end = cube_pos(scene)
displacement = object_end - object_start
contacts = contact_names(scene)
cube_contact = has_cube_contact(scene)

lines = [
    "===== TOUCH OBJECT LEFT =====",
    f"object_start = {object_start}",
    f"object_end = {object_end}",
    f"object_displacement = {displacement}",
    f"touch_pos = {touch_pos}",
    f"final_ee_pos = {left_ee_pos(scene)}",
    f"final_distance = {distances[-1]:.6f}",
    f"ik_success = {success}",
    f"ncon = {scene.data.ncon}",
    f"cube_contact = {cube_contact}",
]
for i, (geom1, geom2, dist) in enumerate(contacts[:20]):
    lines.append(f"contact[{i}] = {geom1}, {geom2}, dist={dist:.6f}")

print("\n".join(lines))
write_lines(OUTPUT_DIR / "touch_object_left.txt", lines)
print("\nSaved to outputs/touch_object_left.txt")
