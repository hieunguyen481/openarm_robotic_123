"""Record a reach episode with state, action, EE position, distance, and images."""

import argparse

import mujoco
import numpy as np

from stage2_common import OUTPUT_DIR, jacobian_ik_step, make_ready_model, resolve_arm

parser = argparse.ArgumentParser()
parser.add_argument("--camera", default="camera_ceiling")
parser.add_argument("--output", default="reach_episode_000.npz")
parser.add_argument("--segment", choices=["left", "right"], default="left")
args = parser.parse_args()

model, data, home_ctrl = make_ready_model()
arm = resolve_arm(model, args.segment)

renderer = mujoco.Renderer(model, height=240, width=320)
camera_name = args.camera
camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
if camera_id == -1:
    raise SystemExit(f"Camera not found: {camera_name}")

start_pos = data.site_xpos[arm.site_id].copy()
direction = np.array([0.04, 0.04, 0.03])
if args.segment == "right":
    direction[1] *= -1
target_pos = start_pos + direction
threshold = 0.03
max_steps = 700

qpos_list = []
qvel_list = []
ctrl_list = []
left_ee_list = []
right_ee_list = []
distance_list = []
rgb_list = []
success = False
success_step = -1

right_site_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_SITE, "right_ee_control_point"
)
left_site_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_SITE, "left_ee_control_point"
)

for step in range(max_steps):
    data.ctrl[:] = home_ctrl
    distance = jacobian_ik_step(model, data, arm, target_pos)

    renderer.update_scene(data, camera=camera_name)
    rgb = renderer.render()

    qpos_list.append(data.qpos.copy())
    qvel_list.append(data.qvel.copy())
    ctrl_list.append(data.ctrl.copy())
    left_ee_list.append(data.site_xpos[left_site_id].copy())
    right_ee_list.append(data.site_xpos[right_site_id].copy())
    distance_list.append(distance)
    rgb_list.append(rgb.copy())

    if distance < threshold:
        success = True
        success_step = step
        break

    mujoco.mj_step(model, data)

output_path = OUTPUT_DIR / args.output
np.savez_compressed(
    output_path,
    qpos=np.array(qpos_list),
    qvel=np.array(qvel_list),
    ctrl=np.array(ctrl_list),
    left_ee_pos=np.array(left_ee_list),
    right_ee_pos=np.array(right_ee_list),
    target_pos=target_pos,
    distance_history=np.array(distance_list),
    distance=np.array(distance_list),
    rgb_images=np.array(rgb_list),
    success=np.array(success),
    success_step=np.array(success_step),
    camera_name=np.array(camera_name),
    segment=np.array(args.segment),
)

lines = [
    f"===== RECORDED {args.segment.upper()} REACH EPISODE =====",
    f"camera_name = {camera_name}",
    f"target_pos = {target_pos}",
    f"frames = {len(distance_list)}",
    f"start_distance = {distance_list[0]:.6f}",
    f"final_distance = {distance_list[-1]:.6f}",
    f"threshold = {threshold:.6f}",
    f"success = {success}",
    f"success_step = {success_step}",
    f"Saved to {output_path}",
]

print("\n".join(lines))
(OUTPUT_DIR / f"{output_path.stem}.txt").write_text("\n".join(lines), encoding="utf-8")
