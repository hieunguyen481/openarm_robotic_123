"""Render camera observations and log qpos/qvel/ctrl/rgb/depth to NPZ."""

import argparse

import mujoco
import numpy as np

from stage1_common import OUTPUT_DIR, load_model, name_or_empty

parser = argparse.ArgumentParser()
parser.add_argument(
    "--camera",
    default=None,
    help="Camera name to render. Defaults to the first model camera.",
)
parser.add_argument("--steps", type=int, default=120)
parser.add_argument("--width", type=int, default=320)
parser.add_argument("--height", type=int, default=240)
parser.add_argument("--output", default="episode_000.npz")
args = parser.parse_args()

model, data = load_model()

renderer = mujoco.Renderer(model, height=args.height, width=args.width)

camera_name = args.camera
if camera_name is not None:
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if camera_id == -1:
        raise SystemExit(f"Camera not found: {camera_name}")
elif model.ncam > 0:
    camera_name = name_or_empty(model, mujoco.mjtObj.mjOBJ_CAMERA, 0)
else:
    print("No camera found. Rendering default view.")

print("Using camera:", camera_name or "default")
base_ctrl = data.ctrl.copy()

qpos_list = []
qvel_list = []
ctrl_list = []
rgb_list = []
depth_list = []

for step in range(args.steps):
    data.ctrl[:] = base_ctrl
    mujoco.mj_step(model, data)

    renderer.update_scene(data, camera=camera_name)
    rgb = renderer.render()

    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=camera_name)
    depth = renderer.render()
    renderer.disable_depth_rendering()

    qpos_list.append(data.qpos.copy())
    qvel_list.append(data.qvel.copy())
    ctrl_list.append(data.ctrl.copy())
    rgb_list.append(rgb.copy())
    depth_list.append(depth.copy())

qpos_arr = np.array(qpos_list)
qvel_arr = np.array(qvel_list)
ctrl_arr = np.array(ctrl_list)
rgb_arr = np.array(rgb_list)
depth_arr = np.array(depth_list)

OUTPUT_DIR.mkdir(exist_ok=True)
output_path = OUTPUT_DIR / args.output
np.savez_compressed(
    output_path,
    qpos=qpos_arr,
    qvel=qvel_arr,
    ctrl=ctrl_arr,
    rgb_images=rgb_arr,
    depth_images=depth_arr,
    camera_name=np.array(camera_name or "default"),
)

print(f"Saved {output_path.relative_to(OUTPUT_DIR.parent)}")
print("qpos:", qpos_arr.shape)
print("qvel:", qvel_arr.shape)
print("ctrl:", ctrl_arr.shape)
print("rgb_images:", rgb_arr.shape)
print("depth_images:", depth_arr.shape)
