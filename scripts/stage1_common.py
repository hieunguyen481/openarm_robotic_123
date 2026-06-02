"""Shared helpers for the OpenArm MuJoCo Stage 1 scripts."""

from __future__ import annotations

from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
XML_PATH = ROOT / "v2" / "cell.xml"
OUTPUT_DIR = ROOT / "outputs"


def load_model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load the Stage 1 OpenArm scene."""
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)
    reset_to_keyframe(model, data)
    return model, data


def reset_to_keyframe(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    keyframe: str = "home",
) -> None:
    """Reset data to a named keyframe and align position actuator controls."""
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
    if key_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, key_id)

    sync_position_controls(model, data)
    mujoco.mj_forward(model, data)


def sync_position_controls(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Set position actuator controls to the current joint positions."""
    for i in range(model.nu):
        joint_id = int(model.actuator_trnid[i, 0])
        if joint_id >= 0:
            data.ctrl[i] = data.qpos[model.jnt_qposadr[joint_id]]


def name_or_empty(model: mujoco.MjModel, obj_type: mujoco.mjtObj, idx: int) -> str:
    """Return a MuJoCo object name, or an empty string for unnamed objects."""
    return mujoco.mj_id2name(model, obj_type, idx) or ""
