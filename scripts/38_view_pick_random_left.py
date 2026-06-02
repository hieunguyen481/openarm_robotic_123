"""View scripted left pick attempts for random cube positions."""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from stage2_common import resolve_arm
from stage1_common import reset_to_keyframe
from stage3_common import (
    CUBE_INITIAL_QUAT,
    cube_pos,
    load_object_scene,
    set_cube_pose,
    settle,
)
from stage4_common import (
    LEFT_CLOSE,
    LEFT_OPEN,
    cube_left_finger_contact,
    cube_left_nonfinger_contact,
    cube_left_nonfinger_contact_names,
    jacobian_grasp_midpoint_step,
    left_grasp_midpoint_pos,
    set_gripper,
)

parser = argparse.ArgumentParser()
parser.add_argument("--num-trials", type=int, default=5)
parser.add_argument("--seed", type=int, default=71)
parser.add_argument("--x-low", type=float, default=0.38)
parser.add_argument("--x-high", type=float, default=0.48)
parser.add_argument("--y-low", type=float, default=0.13)
parser.add_argument("--y-high", type=float, default=0.25)
parser.add_argument("--grasp-x", type=float, default=0.04)
parser.add_argument("--grasp-y", type=float, default=0.0)
parser.add_argument("--grasp-z", type=float, default=0.015)
parser.add_argument("--hover-x", type=float, default=-0.24)
parser.add_argument("--hover-z", type=float, default=0.24)
parser.add_argument("--pre-dx", type=float, default=0.02)
parser.add_argument("--pre-dz", type=float, default=0.125)
parser.add_argument("--lift-dz", type=float, default=0.130)
parser.add_argument("--close-steps", type=int, default=320)
parser.add_argument("--hold-steps", type=int, default=180)
parser.add_argument("--pause-steps", type=int, default=180)
parser.add_argument("--success-height", type=float, default=0.04)
parser.add_argument("--slow", type=float, default=1.0)
args = parser.parse_args()


def viewer_sleep(scene) -> None:
    time.sleep(float(scene.model.opt.timestep) * args.slow)


def reset_trial(scene, position):
    """Reset robot/cube for one visual trial."""
    reset_to_keyframe(scene.model, scene.data)
    set_cube_pose(scene, position, CUBE_INITIAL_QUAT)
    scene.data.ctrl[:] = scene.home_ctrl
    settle(scene)


def make_targets(object_pos):
    """Build high approach, pregrasp, grasp, and lift targets."""
    grasp_offset = np.array([args.grasp_x, args.grasp_y, args.grasp_z])
    grasp = object_pos + grasp_offset
    hover = object_pos + np.array([args.hover_x, args.grasp_y, args.hover_z])
    pregrasp = grasp + np.array([args.pre_dx, 0.0, args.pre_dz])
    lift = grasp + np.array([0.0, 0.0, args.lift_dz])
    return hover, pregrasp, grasp, lift


def set_left_gripper_smooth(scene, viewer, start_value, end_value, steps, phase_name):
    """Move left gripper command smoothly while rendering."""
    print(f"phase = {phase_name}")
    current_ctrl = scene.data.ctrl.copy()
    for step in range(steps):
        if not viewer.is_running():
            break
        alpha = step / max(steps - 1, 1)
        gripper_value = (1.0 - alpha) * start_value + alpha * end_value
        scene.data.ctrl[:] = current_ctrl
        set_gripper(scene, "left", gripper_value)
        mujoco.mj_step(scene.model, scene.data)
        viewer.sync()
        viewer_sleep(scene)


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


def pause(scene, viewer, steps):
    """Keep rendering for a short pause between phases/trials."""
    for _ in range(steps):
        if not viewer.is_running():
            break
        viewer.sync()
        viewer_sleep(scene)


scene = load_object_scene()
left = resolve_arm(scene.model, "left")
rng = np.random.default_rng(args.seed)
positions = np.column_stack(
    [
        rng.uniform(args.x_low, args.x_high, size=args.num_trials),
        rng.uniform(args.y_low, args.y_high, size=args.num_trials),
        np.full(args.num_trials, 1.035),
    ]
)
results = []

print("===== VIEW RANDOM PICK LEFT =====")
print(f"num_trials = {args.num_trials}")
print(f"seed = {args.seed}")
print(f"x_range = [{args.x_low}, {args.x_high}]")
print(f"y_range = [{args.y_low}, {args.y_high}]")
print("Close the viewer window or press Ctrl+C in terminal to stop.")

with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
    for trial_id, requested_pos in enumerate(positions):
        if not viewer.is_running():
            break

        print(f"===== trial {trial_id} =====")
        reset_trial(scene, requested_pos)
        viewer.sync()
        pause(scene, viewer, args.pause_steps)
        object_start = cube_pos(scene)
        hover_pos, pregrasp_pos, grasp_pos, lift_pos = make_targets(object_start)

        print(f"requested_pos = {requested_pos}")
        print(f"object_start_pos = {object_start}")
        print(f"hover_pos = {hover_pos}")
        print(f"pregrasp_pos = {pregrasp_pos}")
        print(f"grasp_pos = {grasp_pos}")
        print(f"lift_pos = {lift_pos}")

        set_left_gripper_smooth(
            scene,
            viewer,
            float(scene.data.ctrl[8]),
            LEFT_OPEN,
            120,
            "open",
        )
        move_to_target(scene, left, viewer, hover_pos, "hover", LEFT_OPEN)
        hover_pos, pregrasp_pos, grasp_pos, lift_pos = make_targets(cube_pos(scene))
        move_to_target(scene, left, viewer, pregrasp_pos, "pregrasp", LEFT_OPEN)
        hover_pos, pregrasp_pos, grasp_pos, lift_pos = make_targets(cube_pos(scene))
        move_to_target(scene, left, viewer, grasp_pos, "grasp", LEFT_OPEN)
        print(f"object_pos_at_grasp = {cube_pos(scene)}")
        print(f"grasp_midpoint_at_grasp = {left_grasp_midpoint_pos(scene)}")
        print(f"bad_contact_at_grasp = {cube_left_nonfinger_contact(scene)}")

        set_left_gripper_smooth(
            scene,
            viewer,
            LEFT_OPEN,
            LEFT_CLOSE,
            args.close_steps,
            "close",
        )
        pause(scene, viewer, args.hold_steps)
        print(f"finger_contact_after_close = {cube_left_finger_contact(scene)}")
        print(f"bad_contact_after_close = {cube_left_nonfinger_contact(scene)}")
        print(f"object_pos_after_close = {cube_pos(scene)}")

        hover_pos, pregrasp_pos, grasp_pos, lift_pos = make_targets(cube_pos(scene))
        move_to_target(scene, left, viewer, lift_pos, "lift", LEFT_CLOSE)
        pause(scene, viewer, args.pause_steps)

        object_end = cube_pos(scene)
        lift_height = float(object_end[2] - object_start[2])
        success = lift_height > args.success_height
        results.append(success)
        print(f"object_end_pos = {object_end}")
        print(f"object_lift_height = {lift_height:.6f}")
        print(f"success = {success}")
        print(f"running_success_rate = {float(np.mean(results)):.3f}")
        pause(scene, viewer, args.pause_steps)

    if results:
        print("===== RANDOM PICK SUMMARY =====")
        print(f"completed_trials = {len(results)}")
        print(f"success_rate = {float(np.mean(results)):.3f}")

    while viewer.is_running():
        viewer.sync()
        viewer_sleep(scene)
