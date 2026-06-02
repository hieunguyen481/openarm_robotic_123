"""Record a scripted left pick episode with robot, object, camera, and action."""

import argparse

import mujoco
import numpy as np

from stage1_common import OUTPUT_DIR
from stage3_common import load_object_scene, write_lines
from stage4_common import run_pick_left

parser = argparse.ArgumentParser()
parser.add_argument("--camera", default="camera_ceiling")
parser.add_argument("--height", type=int, default=240)
parser.add_argument("--width", type=int, default=320)
parser.add_argument("--output", default="pick_episode_000.npz")
args = parser.parse_args()

result = run_pick_left(record=True)

model = load_object_scene().model
renderer = mujoco.Renderer(model, height=args.height, width=args.width)
data = mujoco.MjData(model)
rgb_history = []
depth_history = []

for qpos, qvel, ctrl in zip(
    result.qpos_history,
    result.qvel_history,
    result.ctrl_history,
):
    data.qpos[:] = qpos
    data.qvel[:] = qvel
    data.ctrl[:] = ctrl
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=args.camera)
    rgb_history.append(renderer.render().copy())
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=args.camera)
    depth_history.append(renderer.render().copy())
    renderer.disable_depth_rendering()

output_path = OUTPUT_DIR / args.output
np.savez_compressed(
    output_path,
    qpos=result.qpos_history,
    qvel=result.qvel_history,
    ctrl=result.ctrl_history,
    left_ee_pos=result.left_ee_history,
    right_ee_pos=result.right_ee_history,
    object_pos=result.object_pos_history,
    object_quat=result.object_quat_history,
    rgb_images=np.array(rgb_history),
    depth_images=np.array(depth_history),
    phase=result.phase_history,
    gripper_ctrl=result.gripper_history,
    object_height=result.object_height_history,
    ee_object_dist=result.ee_object_dist_history,
    object_lift_height=np.array(result.object_lift_height),
    finger_contact=np.array(result.finger_contact),
    success=np.array(result.success),
)

lines = [
    "===== RECORDED PICK EPISODE =====",
    f"frames = {len(result.qpos_history)}",
    f"camera = {args.camera}",
    f"object_start_pos = {result.object_start}",
    f"object_end_pos = {result.object_end}",
    f"object_lift_height = {result.object_lift_height:.6f}",
    f"finger_contact = {result.finger_contact}",
    f"success = {result.success}",
    f"saved_npz = {output_path}",
]

print("\n".join(lines))
write_lines(OUTPUT_DIR / args.output.replace(".npz", ".txt"), lines)
