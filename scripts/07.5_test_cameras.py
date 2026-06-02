"""Render one RGB/depth frame from every named MuJoCo camera."""

import imageio.v3 as iio
import mujoco
import numpy as np

from stage1_common import OUTPUT_DIR, load_model, name_or_empty

model, data = load_model()
renderer = mujoco.Renderer(model, height=240, width=320)

camera_names = [
    name_or_empty(model, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(model.ncam)
]

rgb_frames = []
depth_frames = []
lines = ["===== CAMERA RENDER TEST ====="]

OUTPUT_DIR.mkdir(exist_ok=True)
camera_dir = OUTPUT_DIR / "camera_tests"
camera_dir.mkdir(exist_ok=True)

for camera_name in camera_names:
    renderer.update_scene(data, camera=camera_name)
    rgb = renderer.render()

    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=camera_name)
    depth = renderer.render()
    renderer.disable_depth_rendering()

    rgb_frames.append(rgb.copy())
    depth_frames.append(depth.copy())
    iio.imwrite(camera_dir / f"{camera_name}_rgb.png", rgb)

    finite_depth = bool(np.isfinite(depth).all())
    lines.append(
        f"{camera_name}: rgb={rgb.shape} {rgb.dtype}, "
        f"depth={depth.shape} {depth.dtype}, finite_depth={finite_depth}"
    )

np.savez_compressed(
    OUTPUT_DIR / "camera_test.npz",
    camera_names=np.array(camera_names),
    rgb_images=np.array(rgb_frames),
    depth_images=np.array(depth_frames),
)

(OUTPUT_DIR / "camera_test.txt").write_text("\n".join(lines), encoding="utf-8")
print("\n".join(lines))
print("\nSaved outputs/camera_test.txt")
print("Saved outputs/camera_test.npz")
print("Saved RGB preview PNGs to outputs/camera_tests/")
