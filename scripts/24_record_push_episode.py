"""Record a full push episode with robot, object, camera, and action."""

import mujoco
import numpy as np

from stage1_common import OUTPUT_DIR
from stage2_common import jacobian_ik_step, resolve_arm
from stage3_common import (
    cube_pos,
    cube_quat,
    left_ee_pos,
    load_object_scene,
    right_ee_pos,
    settle,
)

scene = load_object_scene()
settle(scene)
left = resolve_arm(scene.model, "left")
renderer = mujoco.Renderer(scene.model, height=240, width=320)
camera_name = "camera_ceiling"

object_start = cube_pos(scene)
approach_pos = object_start + np.array([-0.14, -0.02, 0.08])
touch_pos = object_start + np.array([-0.10, -0.02, 0.05])
push_goal_pos = object_start + np.array([0.02, -0.02, 0.05])
waypoints = [
    ("approach", approach_pos, 0.035, 500),
    ("touch", touch_pos, 0.030, 500),
    ("push", push_goal_pos, 0.035, 1000),
]

qpos_history = []
qvel_history = []
ctrl_history = []
left_ee_history = []
right_ee_history = []
object_pos_history = []
object_quat_history = []
distance_history = []
rgb_history = []
depth_history = []
phase_history = []

for phase_name, target_pos, threshold, max_steps in waypoints:
    for _ in range(max_steps):
        scene.data.ctrl[:] = scene.home_ctrl
        distance = jacobian_ik_step(
            scene.model,
            scene.data,
            left,
            target_pos,
            gain=0.7,
            damping=0.04,
            max_step=0.04,
        )

        renderer.update_scene(scene.data, camera=camera_name)
        rgb = renderer.render()
        renderer.enable_depth_rendering()
        renderer.update_scene(scene.data, camera=camera_name)
        depth = renderer.render()
        renderer.disable_depth_rendering()

        qpos_history.append(scene.data.qpos.copy())
        qvel_history.append(scene.data.qvel.copy())
        ctrl_history.append(scene.data.ctrl.copy())
        left_ee_history.append(left_ee_pos(scene))
        right_ee_history.append(right_ee_pos(scene))
        object_pos_history.append(cube_pos(scene))
        object_quat_history.append(cube_quat(scene))
        distance_history.append(distance)
        rgb_history.append(rgb.copy())
        depth_history.append(depth.copy())
        phase_history.append(phase_name)

        if distance < threshold and phase_name != "push":
            break
        mujoco.mj_step(scene.model, scene.data)

object_end = cube_pos(scene)
push_distance_x = float(object_end[0] - object_start[0])
success = push_distance_x > 0.02

np.savez_compressed(
    OUTPUT_DIR / "push_episode_000.npz",
    qpos=np.array(qpos_history),
    qvel=np.array(qvel_history),
    ctrl=np.array(ctrl_history),
    left_ee_pos=np.array(left_ee_history),
    right_ee_pos=np.array(right_ee_history),
    object_pos=np.array(object_pos_history),
    object_quat=np.array(object_quat_history),
    target_pos=touch_pos,
    push_goal_pos=push_goal_pos,
    rgb_images=np.array(rgb_history),
    depth_images=np.array(depth_history),
    distance_to_object=np.linalg.norm(
        np.array(object_pos_history) - np.array(left_ee_history), axis=1
    ),
    waypoint_distance=np.array(distance_history),
    phase=np.array(phase_history),
    push_distance_x=np.array(push_distance_x),
    success=np.array(success),
)

lines = [
    "===== RECORDED PUSH EPISODE =====",
    f"frames = {len(qpos_history)}",
    f"object_start_pos = {object_start}",
    f"object_end_pos = {object_end}",
    f"push_distance_x = {push_distance_x:.6f}",
    f"success = {success}",
    "Saved to outputs/push_episode_000.npz",
]

print("\n".join(lines))
(OUTPUT_DIR / "push_episode_000.txt").write_text("\n".join(lines), encoding="utf-8")
