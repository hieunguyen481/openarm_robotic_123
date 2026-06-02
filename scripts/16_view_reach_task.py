"""View a reach task live in MuJoCo."""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from stage2_common import jacobian_ik_step, make_ready_model, resolve_arm

parser = argparse.ArgumentParser()
parser.add_argument("--segment", choices=["left", "right"], default="left")
args = parser.parse_args()

model, data, home_ctrl = make_ready_model()
arm = resolve_arm(model, args.segment)

start_pos = data.site_xpos[arm.site_id].copy()
direction = np.array([0.04, 0.04, 0.03])
if args.segment == "right":
    direction[1] *= -1
target_pos = start_pos + direction
threshold = 0.03

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = model.stat.center
    viewer.cam.distance = model.stat.extent
    viewer.cam.azimuth = model.vis.global_.azimuth
    viewer.cam.elevation = model.vis.global_.elevation

    while viewer.is_running():
        data.ctrl[:] = home_ctrl
        distance = jacobian_ik_step(model, data, arm, target_pos)
        mujoco.mj_step(model, data)
        viewer.sync()
        if distance < threshold:
            print(f"success: distance={distance:.6f}")
            time.sleep(1.0)
            break
        time.sleep(model.opt.timestep)
