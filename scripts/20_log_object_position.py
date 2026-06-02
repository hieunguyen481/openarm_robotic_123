"""Log target cube and end-effector positions."""

import numpy as np

from stage1_common import OUTPUT_DIR
from stage3_common import cube_pos, left_ee_pos, load_object_scene, settle, write_lines

scene = load_object_scene()
settle(scene)

object_pos = cube_pos(scene)
left_pos = left_ee_pos(scene)
distance = np.linalg.norm(object_pos - left_pos)

lines = [
    "===== OBJECT POSITION =====",
    f"object_pos = {object_pos}",
    f"left_ee_pos = {left_pos}",
    f"distance_left_ee_to_object = {distance:.6f}",
]

print("\n".join(lines))
write_lines(OUTPUT_DIR / "object_position.txt", lines)
print("\nSaved to outputs/object_position.txt")
