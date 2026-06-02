"""Align the open left gripper near the cube."""

import numpy as np

from stage1_common import OUTPUT_DIR
from stage3_common import cube_pos, left_ee_pos, write_lines
from stage4_common import (
    LEFT_OPEN,
    left_pick_waypoints,
    run_ik_with_gripper,
    setup_pick_scene,
)

scene, left = setup_pick_scene()
object_start = cube_pos(scene)
waypoints = left_pick_waypoints(object_start)

pregrasp_success, pregrasp_distance = run_ik_with_gripper(
    scene,
    left,
    waypoints["pregrasp"],
    gripper_value=LEFT_OPEN,
    threshold=0.035,
    max_steps=700,
)
grasp_success, grasp_distance = run_ik_with_gripper(
    scene,
    left,
    waypoints["grasp"],
    gripper_value=LEFT_OPEN,
    threshold=0.030,
    max_steps=700,
)

object_now = cube_pos(scene)
ee_now = left_ee_pos(scene)
lines = [
    "===== ALIGN GRIPPER TO CUBE LEFT =====",
    f"object_start_pos = {object_start}",
    f"pregrasp_pos = {waypoints['pregrasp']}",
    f"grasp_pos = {waypoints['grasp']}",
    f"pregrasp_success = {pregrasp_success}",
    f"pregrasp_final_distance = {pregrasp_distance:.6f}",
    f"grasp_success = {grasp_success}",
    f"grasp_final_distance = {grasp_distance:.6f}",
    f"final_left_ee_pos = {ee_now}",
    f"object_pos = {object_now}",
    f"ee_object_distance = {np.linalg.norm(ee_now - object_now):.6f}",
]

print("\n".join(lines))
write_lines(OUTPUT_DIR / "align_gripper_to_cube_left.txt", lines)
