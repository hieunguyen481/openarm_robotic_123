"""Run a full scripted left pick attempt and report lift success."""

from stage1_common import OUTPUT_DIR
from stage3_common import write_lines
from stage4_common import LIFT_SUCCESS_HEIGHT, run_pick_left

result = run_pick_left(record=False)

lines = [
    "===== PICK LIFT CUBE LEFT =====",
    f"object_start_pos = {result.object_start}",
    f"object_after_close_pos = {result.object_after_close}",
    f"object_end_pos = {result.object_end}",
    f"object_lift_height = {result.object_lift_height:.6f}",
    f"lift_success_threshold = {LIFT_SUCCESS_HEIGHT:.6f}",
    f"finger_contact = {result.finger_contact}",
    f"final_ee_object_dist = {result.final_ee_object_dist:.6f}",
    f"success = {result.success}",
]

print("\n".join(lines))
write_lines(OUTPUT_DIR / "pick_lift_cube_left.txt", lines)
