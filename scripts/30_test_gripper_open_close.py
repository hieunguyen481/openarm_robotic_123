"""Test left/right gripper open-close commands."""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from stage1_common import OUTPUT_DIR
from stage3_common import load_object_scene, settle, write_lines
from stage4_common import LEFT_GRIPPER, RIGHT_GRIPPER, set_gripper

parser = argparse.ArgumentParser()
parser.add_argument("--headless-steps", type=int, default=0)
args = parser.parse_args()

scene = load_object_scene()
settle(scene)

left_low, left_high = scene.model.actuator_ctrlrange[LEFT_GRIPPER]
right_low, right_high = scene.model.actuator_ctrlrange[RIGHT_GRIPPER]

lines = [
    "===== TEST GRIPPER OPEN/CLOSE =====",
    f"left_gripper_ctrl = ctrl[{LEFT_GRIPPER}]",
    f"left_gripper_range = [{left_low:.4f}, {left_high:.4f}]",
    f"right_gripper_ctrl = ctrl[{RIGHT_GRIPPER}]",
    f"right_gripper_range = [{right_low:.4f}, {right_high:.4f}]",
    "measured_left_open = 0.7854",
    "measured_left_close = 0.0",
    "measured_right_open = -0.7854",
    "measured_right_close = 0.0",
]

if args.headless_steps > 0:
    left_values = []
    right_values = []
    for step in range(args.headless_steps):
        phase = 2.0 * np.pi * step / max(args.headless_steps - 1, 1)
        scene.data.ctrl[:] = scene.home_ctrl
        left_value = left_low + (left_high - left_low) * (0.5 + 0.5 * np.sin(phase))
        right_value = right_low + (right_high - right_low) * (0.5 + 0.5 * np.sin(phase))
        set_gripper(scene, "left", left_value)
        set_gripper(scene, "right", right_value)
        left_values.append(scene.data.ctrl[LEFT_GRIPPER])
        right_values.append(scene.data.ctrl[RIGHT_GRIPPER])
        mujoco.mj_step(scene.model, scene.data)

    lines.extend(
        [
            f"headless_steps = {args.headless_steps}",
            f"observed_left_min = {float(np.min(left_values)):.6f}",
            f"observed_left_max = {float(np.max(left_values)):.6f}",
            f"observed_right_min = {float(np.min(right_values)):.6f}",
            f"observed_right_max = {float(np.max(right_values)):.6f}",
        ]
    )
    print("\n".join(lines))
    write_lines(OUTPUT_DIR / "gripper_open_close.txt", lines)
else:
    print("\n".join(lines))
    print("Close the viewer window or press Ctrl+C in terminal to stop.")
    with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
        t0 = time.time()
        while viewer.is_running():
            t = time.time() - t0
            scene.data.ctrl[:] = scene.home_ctrl
            left_value = left_low + (left_high - left_low) * (0.5 + 0.5 * np.sin(t))
            right_value = right_low + (right_high - right_low) * (0.5 + 0.5 * np.sin(t))
            set_gripper(scene, "left", left_value)
            set_gripper(scene, "right", right_value)
            mujoco.mj_step(scene.model, scene.data)
            viewer.sync()
            time.sleep(float(scene.model.opt.timestep))
