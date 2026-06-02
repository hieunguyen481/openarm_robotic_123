"""Record a compact multi-episode random push dataset."""

import argparse

import mujoco
import numpy as np

from stage1_common import OUTPUT_DIR
from stage2_common import resolve_arm
from stage3_common import (
    cube_pos,
    load_object_scene,
    run_push_episode,
    sample_cube_positions,
    set_cube_pose,
    settle,
    write_lines,
)

parser = argparse.ArgumentParser()
parser.add_argument("--segment", choices=["left", "right"], default="left")
parser.add_argument("--num-episodes", type=int, default=5)
parser.add_argument("--seed", type=int, default=41)
parser.add_argument("--camera", default="camera_ceiling")
parser.add_argument("--image-stride", type=int, default=10)
parser.add_argument("--height", type=int, default=240)
parser.add_argument("--width", type=int, default=320)
args = parser.parse_args()

positions = sample_cube_positions(
    segment=args.segment,
    num_positions=args.num_episodes,
    seed=args.seed,
)

episodes = []
rgb_history = []
depth_history = []
image_episode_ids = []
image_frame_ids = []
summary_rows = []

for episode_id, cube_start_request in enumerate(positions):
    scene = load_object_scene()
    set_cube_pose(scene, cube_start_request)
    settle(scene)
    arm = resolve_arm(scene.model, args.segment)
    renderer = mujoco.Renderer(scene.model, height=args.height, width=args.width)

    result = run_push_episode(scene, arm, success_threshold=0.03, record=True)

    for frame_id in range(0, len(result.qpos_history), args.image_stride):
        scene.data.qpos[:] = result.qpos_history[frame_id]
        scene.data.qvel[:] = result.qvel_history[frame_id]
        scene.data.ctrl[:] = result.ctrl_history[frame_id]
        mujoco.mj_forward(scene.model, scene.data)

        renderer.update_scene(scene.data, camera=args.camera)
        rgb_history.append(renderer.render().copy())
        renderer.enable_depth_rendering()
        renderer.update_scene(scene.data, camera=args.camera)
        depth_history.append(renderer.render().copy())
        renderer.disable_depth_rendering()
        image_episode_ids.append(episode_id)
        image_frame_ids.append(frame_id)

    episodes.append(
        {
            "qpos": result.qpos_history,
            "qvel": result.qvel_history,
            "ctrl": result.ctrl_history,
            "left_ee_pos": result.left_ee_history,
            "right_ee_pos": result.right_ee_history,
            "object_pos": result.object_pos_history,
            "object_quat": result.object_quat_history,
            "waypoint_distance": result.waypoint_distance_history,
            "phase": result.phase_history,
            "object_start": result.object_start,
            "object_end": result.object_end,
            "push_distance_x": result.push_distance_x,
            "success": result.success,
        }
    )
    summary_rows.append(
        [
            episode_id,
            *cube_start_request.tolist(),
            *cube_pos(scene).tolist(),
            result.push_distance_x,
            result.success,
            len(result.qpos_history),
        ]
    )

output_path = OUTPUT_DIR / f"push_dataset_{args.segment}.npz"
np.savez_compressed(
    output_path,
    episodes=np.array(episodes, dtype=object),
    rgb_images=np.array(rgb_history),
    depth_images=np.array(depth_history),
    image_episode_ids=np.array(image_episode_ids),
    image_frame_ids=np.array(image_frame_ids),
    camera=np.array(args.camera),
    image_stride=np.array(args.image_stride),
    summary_rows=np.array(summary_rows, dtype=object),
)

success_rate = float(np.mean([row[-2] for row in summary_rows]))
mean_push_distance_x = float(np.mean([row[-3] for row in summary_rows]))
lines = [
    "===== RECORDED RANDOM PUSH DATASET =====",
    f"segment = {args.segment}",
    f"num_episodes = {args.num_episodes}",
    f"camera = {args.camera}",
    f"image_stride = {args.image_stride}",
    f"success_rate = {success_rate:.3f}",
    f"mean_push_distance_x = {mean_push_distance_x:.6f}",
    f"saved_npz = {output_path}",
]

print("\n".join(lines))
write_lines(OUTPUT_DIR / f"push_dataset_{args.segment}.txt", lines)
