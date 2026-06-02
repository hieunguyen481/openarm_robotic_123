"""View and tune the left pick/grasp trajectory in MuJoCo."""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from stage3_common import cube_pos, left_ee_pos
from stage4_common import (
    LEFT_CLOSE,
    LEFT_OPEN,
    cube_left_finger_contact,
    cube_left_nonfinger_contact,
    cube_left_nonfinger_contact_names,
    jacobian_grasp_midpoint_step,
    left_grasp_midpoint_pos,
    set_gripper,
    setup_pick_scene,
)

parser = argparse.ArgumentParser()
parser.add_argument("--grasp-x", type=float, default=0.04)
parser.add_argument("--grasp-y", type=float, default=0.0)
parser.add_argument("--grasp-z", type=float, default=0.015)
parser.add_argument("--pre-dx", type=float, default=0.02)
parser.add_argument("--pre-dz", type=float, default=0.125)
parser.add_argument("--lift-dz", type=float, default=0.13)
parser.add_argument("--retry-lift-dz", type=float, default=0.16)
parser.add_argument("--close-steps", type=int, default=320)
parser.add_argument("--regrip-steps", type=int, default=180)
parser.add_argument("--retries", type=int, default=2)
parser.add_argument("--success-height", type=float, default=0.04)
parser.add_argument("--slow", type=float, default=1.0)
args = parser.parse_args()


def viewer_sleep(scene) -> None:
    time.sleep(float(scene.model.opt.timestep) * args.slow)


def move_to_target(scene, arm, viewer, target_pos, phase_name, gripper_value):
    """Move to a target while keeping the viewer updated."""
    last_distance = float("nan")
    for _ in range(800):
        if not viewer.is_running():
            break
        scene.data.ctrl[:] = scene.home_ctrl
        set_gripper(scene, "left", gripper_value)
        last_distance = jacobian_grasp_midpoint_step(
            scene,
            arm,
            target_pos,
            gain=0.7,
            damping=0.04,
            max_step=0.04,
        )
        set_gripper(scene, "left", gripper_value)
        mujoco.mj_step(scene.model, scene.data)
        viewer.sync()
        viewer_sleep(scene)
        if cube_left_nonfinger_contact(scene):
            print(f"{phase_name}_bad_contact = True")
            print(
                f"{phase_name}_bad_contacts = "
                f"{cube_left_nonfinger_contact_names(scene)}"
            )
            break
        if last_distance < 0.03:
            break
    print(f"{phase_name}_final_distance = {last_distance:.6f}")


def hold_closed(scene, viewer, steps, phase_name) -> None:
    """Hold the current arm target with the left gripper closed."""
    print(f"phase = {phase_name}")
    current_ctrl = scene.data.ctrl.copy()
    for _ in range(steps):
        if not viewer.is_running():
            break
        scene.data.ctrl[:] = current_ctrl
        set_gripper(scene, "left", LEFT_CLOSE)
        mujoco.mj_step(scene.model, scene.data)
        viewer.sync()
        viewer_sleep(scene)


def open_gripper(scene, viewer, steps, phase_name) -> None:
    """Open the left gripper visibly over several viewer frames."""
    print(f"phase = {phase_name}")
    current_ctrl = scene.data.ctrl.copy()
    start_value = float(scene.data.ctrl[8])
    for step in range(steps):
        if not viewer.is_running():
            break
        alpha = step / max(steps - 1, 1)
        gripper_value = (1.0 - alpha) * start_value + alpha * LEFT_OPEN
        scene.data.ctrl[:] = current_ctrl
        set_gripper(scene, "left", gripper_value)
        mujoco.mj_step(scene.model, scene.data)
        viewer.sync()
        viewer_sleep(scene)


def close_gripper(scene, viewer, steps, phase_name) -> None:
    """Close the left gripper visibly over several viewer frames."""
    print(f"phase = {phase_name}")
    current_ctrl = scene.data.ctrl.copy()
    start_value = float(scene.data.ctrl[8])
    for step in range(steps):
        if not viewer.is_running():
            break
        alpha = step / max(steps - 1, 1)
        gripper_value = (1.0 - alpha) * start_value + alpha * LEFT_CLOSE
        scene.data.ctrl[:] = current_ctrl
        set_gripper(scene, "left", gripper_value)
        mujoco.mj_step(scene.model, scene.data)
        viewer.sync()
        viewer_sleep(scene)


def make_targets(object_pos):
    """Build pregrasp, grasp, and lift targets from the current cube position."""
    grasp_offset = np.array([args.grasp_x, args.grasp_y, args.grasp_z])
    grasp = object_pos + grasp_offset
    pregrasp = grasp + np.array([args.pre_dx, 0.0, args.pre_dz])
    lift = grasp + np.array([0.0, 0.0, args.lift_dz])
    return pregrasp, grasp, lift


scene, left = setup_pick_scene()
object_start = cube_pos(scene)
pregrasp_pos, grasp_pos, lift_pos = make_targets(object_start)

print("===== VIEW PICK TUNE LEFT =====")
print(f"object_start_pos = {object_start}")
print(f"pregrasp_pos = {pregrasp_pos}")
print(f"grasp_pos = {grasp_pos}")
print(f"lift_pos = {lift_pos}")
print("Try changing --grasp-x --grasp-y --grasp-z if the cube is not centered.")
print("Close the viewer window or press Ctrl+C in terminal to stop.")

with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
    for attempt in range(args.retries + 1):
        object_now = cube_pos(scene)
        pregrasp_pos, grasp_pos, lift_pos = make_targets(object_now)
        attempt_lift_pos = lift_pos + np.array([0.0, 0.0, attempt * args.retry_lift_dz])

        print(f"===== attempt {attempt + 1} =====")
        print(f"object_pos_attempt_start = {object_now}")
        print(f"pregrasp_pos = {pregrasp_pos}")
        print(f"grasp_pos = {grasp_pos}")
        print(f"lift_pos = {attempt_lift_pos}")

        open_gripper(scene, viewer, 120, f"open_attempt_{attempt + 1}")
        move_to_target(
            scene,
            left,
            viewer,
            pregrasp_pos,
            f"pregrasp_attempt_{attempt + 1}",
            LEFT_OPEN,
        )
        move_to_target(
            scene,
            left,
            viewer,
            grasp_pos,
            f"grasp_attempt_{attempt + 1}",
            LEFT_OPEN,
        )
        print(f"left_ee_pos_at_grasp = {left_ee_pos(scene)}")
        print(f"grasp_midpoint_at_grasp = {left_grasp_midpoint_pos(scene)}")
        print(f"object_pos_at_grasp = {cube_pos(scene)}")
        print(f"bad_contact_at_grasp = {cube_left_nonfinger_contact(scene)}")

        close_gripper(scene, viewer, args.close_steps, f"close_attempt_{attempt + 1}")
        hold_closed(scene, viewer, args.regrip_steps, f"hold_closed_{attempt + 1}")
        print(f"finger_contact_after_close = {cube_left_finger_contact(scene)}")
        print(f"bad_contact_after_close = {cube_left_nonfinger_contact(scene)}")
        print(f"object_pos_after_close = {cube_pos(scene)}")

        move_to_target(
            scene,
            left,
            viewer,
            attempt_lift_pos,
            f"lift_attempt_{attempt + 1}",
            LEFT_CLOSE,
        )

        object_now = cube_pos(scene)
        lift_height = object_now[2] - object_start[2]
        print(f"object_pos_after_lift_attempt_{attempt + 1} = {object_now}")
        print(f"object_lift_height_attempt_{attempt + 1} = {lift_height:.6f}")
        print(
            f"finger_contact_attempt_{attempt + 1} = {cube_left_finger_contact(scene)}"
        )

        if lift_height > args.success_height:
            print("lift_success = True")
            break

        if attempt < args.retries and viewer.is_running():
            print("lift_success = False, opening gripper and retrying")

    object_end = cube_pos(scene)
    print(f"object_end_pos = {object_end}")
    print(f"object_lift_height = {object_end[2] - object_start[2]:.6f}")
    print(f"finger_contact_end = {cube_left_finger_contact(scene)}")

    while viewer.is_running():
        viewer.sync()
        viewer_sleep(scene)
