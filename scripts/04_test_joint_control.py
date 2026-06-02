"""Move one actuator with a small sine wave to identify what it controls."""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from stage1_common import load_model, name_or_empty


def get_home_ctrl_from_qpos(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Build position actuator controls from the current qpos."""
    home_ctrl = np.zeros(model.nu)
    for i in range(model.nu):
        joint_id = int(model.actuator_trnid[i, 0])
        qpos_id = int(model.jnt_qposadr[joint_id])
        home_ctrl[i] = data.qpos[qpos_id]
    return home_ctrl


def clipped_ctrl_value(
    model: mujoco.MjModel,
    actuator_id: int,
    value: float,
) -> float:
    """Clip a control value to its actuator range when the actuator is limited."""
    if not model.actuator_ctrllimited[actuator_id]:
        return value
    low, high = model.actuator_ctrlrange[actuator_id]
    return float(np.clip(value, low, high))


parser = argparse.ArgumentParser()
parser.add_argument("--actuator", type=int, default=4)
parser.add_argument("--amplitude", type=float, default=0.1)
parser.add_argument("--freq", type=float, default=0.5)
parser.add_argument(
    "--headless-all",
    action="store_true",
    help="Sweep every actuator without opening the viewer and save a summary.",
)
parser.add_argument("--steps", type=int, default=240)
args = parser.parse_args()

model, data = load_model()
if not args.headless_all and not 0 <= args.actuator < model.nu:
    raise SystemExit(f"--actuator must be between 0 and {model.nu - 1}")

if args.headless_all:
    from stage1_common import OUTPUT_DIR, reset_to_keyframe

    lines = ["===== HEADLESS JOINT CONTROL SWEEP ====="]
    for actuator_id in range(model.nu):
        reset_to_keyframe(model, data)
        base_ctrl = get_home_ctrl_from_qpos(model, data)
        data.ctrl[:] = base_ctrl
        for _ in range(100):
            mujoco.mj_step(model, data)
        base_ctrl = get_home_ctrl_from_qpos(model, data)
        base_qpos = data.qpos.copy()
        actuator_name = name_or_empty(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        joint_name = name_or_empty(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)

        for step in range(args.steps):
            t = step * model.opt.timestep
            data.ctrl[:] = base_ctrl
            value = base_ctrl[actuator_id] + (
                args.amplitude * np.sin(2 * np.pi * args.freq * t)
            )
            data.ctrl[actuator_id] = clipped_ctrl_value(model, actuator_id, value)
            mujoco.mj_step(model, data)

        delta = data.qpos - base_qpos
        max_abs_delta = float(np.max(np.abs(delta)))
        lines.append(
            f"ctrl[{actuator_id}] -> actuator={actuator_name} -> "
            f"joint={joint_name} -> max_abs_qpos_delta={max_abs_delta:.6f}"
        )

    OUTPUT_DIR.mkdir(exist_ok=True)
    (OUTPUT_DIR / "joint_control_summary.txt").write_text(
        "\n".join(lines), encoding="utf-8"
    )
    print("\n".join(lines))
    print("\nSaved to outputs/joint_control_summary.txt")
    raise SystemExit(0)

actuator_name = name_or_empty(model, mujoco.mjtObj.mjOBJ_ACTUATOR, args.actuator)
joint_id = int(model.actuator_trnid[args.actuator, 0])
joint_name = name_or_empty(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
data.ctrl[:] = get_home_ctrl_from_qpos(model, data)
for _ in range(100):
    mujoco.mj_step(model, data)
base_ctrl = get_home_ctrl_from_qpos(model, data)

print("model.nu =", model.nu)
print(f"Testing ctrl[{args.actuator}] -> {actuator_name} -> {joint_name}")
print("Press Ctrl+C in terminal to stop.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    t0 = time.time()
    last_print = 0

    while viewer.is_running():
        t = time.time() - t0
        data.ctrl[:] = base_ctrl
        value = base_ctrl[args.actuator] + (
            args.amplitude * np.sin(2 * np.pi * args.freq * t)
        )
        data.ctrl[args.actuator] = clipped_ctrl_value(model, args.actuator, value)

        mujoco.mj_step(model, data)
        viewer.sync()

        if int(t) != last_print:
            last_print = int(t)
            print("qpos:", data.qpos[:])
            print("qvel:", data.qvel[:])
            print("ctrl:", data.ctrl[:])

        time.sleep(model.opt.timestep)
