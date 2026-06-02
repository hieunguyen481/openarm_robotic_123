"""View a push task directly in the MuJoCo viewer."""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from stage2_common import jacobian_ik_step, resolve_arm
from stage3_common import (
    CUBE_INITIAL_POS,
    cube_pos,
    load_object_scene,
    push_waypoints,
    sample_cube_positions,
    set_cube_pose,
    settle,
)

parser = argparse.ArgumentParser()
parser.add_argument("--segment", choices=["left", "right"], default="left")
parser.add_argument("--random", action="store_true")
parser.add_argument("--far", action="store_true")
parser.add_argument("--object-x", type=float, default=None)
parser.add_argument("--object-y", type=float, default=None)
parser.add_argument("--object-z", type=float, default=None)
parser.add_argument("--seed", type=int, default=43)
args = parser.parse_args()

scene = load_object_scene()

if args.object_x is not None or args.object_y is not None or args.object_z is not None:
    pos = CUBE_INITIAL_POS.copy()
    if args.object_x is not None:
        pos[0] = args.object_x
    if args.object_y is not None:
        pos[1] = args.object_y
    if args.object_z is not None:
        pos[2] = args.object_z
    set_cube_pose(scene, pos)
elif args.far:
    y = 0.23 if args.segment == "left" else -0.23
    set_cube_pose(scene, np.array([0.50, y, CUBE_INITIAL_POS[2]]))
elif args.random:
    pos = sample_cube_positions(segment=args.segment, num_positions=1, seed=args.seed)[
        0
    ]
    set_cube_pose(scene, pos)
settle(scene)
arm = resolve_arm(scene.model, args.segment)

print(f"segment = {args.segment}")
print(f"object_start_pos = {cube_pos(scene)}")
print("Close the viewer window or press Ctrl+C in terminal to stop.")

with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
    for phase_name, target_pos, threshold, max_steps in push_waypoints(
        cube_pos(scene),
        args.segment,
    ):
        print(f"phase = {phase_name}, target_pos = {target_pos}")
        for _ in range(max_steps):
            if not viewer.is_running():
                break

            scene.data.ctrl[:] = scene.home_ctrl
            distance = jacobian_ik_step(
                scene.model,
                scene.data,
                arm,
                target_pos,
                gain=0.7,
                damping=0.04,
                max_step=0.04,
            )
            mujoco.mj_step(scene.model, scene.data)
            viewer.sync()

            if distance < threshold and phase_name != "push":
                break

            time.sleep(float(scene.model.opt.timestep))

        if not viewer.is_running():
            break

    print(f"object_end_pos = {cube_pos(scene)}")

    while viewer.is_running():
        viewer.sync()
        time.sleep(float(scene.model.opt.timestep))
