"""Run passive OpenArm simulation in the MuJoCo viewer."""

import argparse
import time

import mujoco
import mujoco.viewer

from stage1_common import OUTPUT_DIR, load_model

parser = argparse.ArgumentParser()
parser.add_argument(
    "--headless-steps",
    type=int,
    default=0,
    help="Run this many passive simulation steps without opening the viewer.",
)
args = parser.parse_args()

model, data = load_model()

if args.headless_steps > 0:
    lines = [
        "===== PASSIVE SIM HEADLESS CHECK =====",
        f"steps = {args.headless_steps}",
    ]
    for _ in range(args.headless_steps):
        mujoco.mj_step(model, data)

    lines.extend(
        [
            f"time = {data.time}",
            f"qpos_finite = {bool((data.qpos == data.qpos).all())}",
            f"qvel_finite = {bool((data.qvel == data.qvel).all())}",
            f"qpos = {data.qpos}",
            f"qvel = {data.qvel}",
        ]
    )
    OUTPUT_DIR.mkdir(exist_ok=True)
    (OUTPUT_DIR / "passive_check.txt").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    print("\nSaved to outputs/passive_check.txt")
    raise SystemExit(0)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)
