"""Close the left gripper around the cube and check finger contact."""

from stage1_common import OUTPUT_DIR
from stage3_common import cube_pos, write_lines
from stage4_common import (
    LEFT_OPEN,
    close_gripper,
    cube_left_finger_contact,
    left_pick_waypoints,
    run_ik_with_gripper,
    setup_pick_scene,
)

scene, left = setup_pick_scene()
object_start = cube_pos(scene)
waypoints = left_pick_waypoints(object_start)

run_ik_with_gripper(scene, left, waypoints["pregrasp"], gripper_value=LEFT_OPEN)
run_ik_with_gripper(
    scene,
    left,
    waypoints["grasp"],
    gripper_value=LEFT_OPEN,
    threshold=0.035,
)
object_before_close = cube_pos(scene)
close_gripper(scene, left)
object_after_close = cube_pos(scene)
finger_contact = cube_left_finger_contact(scene)

lines = [
    "===== GRASP CUBE LEFT =====",
    f"object_start_pos = {object_start}",
    f"object_pos_before_close = {object_before_close}",
    f"object_pos_after_close = {object_after_close}",
    f"object_displacement_after_close = {object_after_close - object_before_close}",
    f"cube_contact_with_left_finger = {finger_contact}",
]

print("\n".join(lines))
write_lines(OUTPUT_DIR / "grasp_cube_left.txt", lines)
