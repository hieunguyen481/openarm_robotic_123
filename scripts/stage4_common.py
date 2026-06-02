"""Shared helpers for Stage 4 pick/grasp scripts."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from stage2_common import ArmControlMap, jacobian_ik_step, resolve_arm
from stage3_common import (
    ObjectScene,
    contact_names,
    cube_pos,
    cube_quat,
    left_ee_pos,
    load_object_scene,
    right_ee_pos,
    set_cube_pose,
    settle,
)

LEFT_GRIPPER = 8
RIGHT_GRIPPER = 16
LEFT_OPEN = 0.7854
LEFT_CLOSE = 0.0
RIGHT_OPEN = -0.7854
RIGHT_CLOSE = 0.0
LIFT_SUCCESS_HEIGHT = 0.04
LEFT_GRASP_GEOMS = (
    "finger_inner_left_collision_03",
    "finger_outer_left_collision_03",
)
LEFT_NONFINGER_PICK_COLLISION_GEOMS = (
    "ee_base_link_left_collision_00",
    "link6_left_collision_00",
)


@dataclass
class PickResult:
    """Summary and optional trajectory from one scripted pick attempt."""

    object_start: np.ndarray
    object_after_close: np.ndarray
    object_end: np.ndarray
    object_lift_height: float
    success: bool
    finger_contact: bool
    final_ee_object_dist: float
    qpos_history: np.ndarray
    qvel_history: np.ndarray
    ctrl_history: np.ndarray
    left_ee_history: np.ndarray
    right_ee_history: np.ndarray
    object_pos_history: np.ndarray
    object_quat_history: np.ndarray
    gripper_history: np.ndarray
    object_height_history: np.ndarray
    ee_object_dist_history: np.ndarray
    phase_history: np.ndarray


def setup_pick_scene(
    cube_position: np.ndarray | None = None,
    disable_nonfinger_collision: bool = True,
) -> tuple[
    ObjectScene,
    ArmControlMap,
]:
    """Load the object scene, optionally place the cube, and resolve the left arm."""
    scene = load_object_scene()
    if disable_nonfinger_collision:
        disable_left_nonfinger_pick_collisions(scene)
    if cube_position is not None:
        set_cube_pose(scene, cube_position)
    settle(scene)
    return scene, resolve_arm(scene.model, "left")


def disable_left_nonfinger_pick_collisions(scene: ObjectScene) -> None:
    """Disable palm/wrist collision geoms for Stage 4 pick debugging."""
    for geom_name in LEFT_NONFINGER_PICK_COLLISION_GEOMS:
        geom_id = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        if geom_id != -1:
            scene.model.geom_contype[geom_id] = 0
            scene.model.geom_conaffinity[geom_id] = 0


def gripper_ctrl_value(segment: str, closed: bool) -> float:
    """Return the configured open/close value for one gripper."""
    if segment == "left":
        return LEFT_CLOSE if closed else LEFT_OPEN
    if segment == "right":
        return RIGHT_CLOSE if closed else RIGHT_OPEN
    raise ValueError(f"Invalid segment: {segment}")


def set_gripper(scene: ObjectScene, segment: str, value: float) -> None:
    """Write one gripper command into ctrl."""
    ctrl_id = LEFT_GRIPPER if segment == "left" else RIGHT_GRIPPER
    low, high = scene.model.actuator_ctrlrange[ctrl_id]
    scene.data.ctrl[ctrl_id] = np.clip(value, low, high)


def left_pick_waypoints(object_pos: np.ndarray) -> dict[str, np.ndarray]:
    """Return target positions for the midpoint between the left fingertips."""
    return {
        "pregrasp": object_pos + np.array([0.06, 0.0, 0.140]),
        "grasp": object_pos + np.array([0.04, 0.0, 0.015]),
        "lift": object_pos + np.array([0.04, 0.0, 0.145]),
    }


def left_grasp_geom_ids(model: mujoco.MjModel) -> tuple[int, int]:
    """Resolve the two visual fingertip geoms used as the grasp midpoint."""
    geom_ids = []
    for geom_name in LEFT_GRASP_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        if geom_id == -1:
            raise ValueError(f"Geom not found: {geom_name}")
        geom_ids.append(geom_id)
    return int(geom_ids[0]), int(geom_ids[1])


def left_grasp_midpoint_pos(scene: ObjectScene) -> np.ndarray:
    """Return the midpoint between the two left fingertip geoms."""
    mujoco.mj_forward(scene.model, scene.data)
    inner_id, outer_id = left_grasp_geom_ids(scene.model)
    return (scene.data.geom_xpos[inner_id] + scene.data.geom_xpos[outer_id]) * 0.5


def jacobian_grasp_midpoint_step(
    scene: ObjectScene,
    arm: ArmControlMap,
    target_pos: np.ndarray,
    *,
    gain: float = 0.7,
    damping: float = 0.04,
    max_step: float = 0.04,
) -> float:
    """Run one IK step using the midpoint between the two left fingertips."""
    mujoco.mj_forward(scene.model, scene.data)
    current_pos = left_grasp_midpoint_pos(scene)
    error = target_pos - current_pos

    inner_id, outer_id = left_grasp_geom_ids(scene.model)
    jacp_inner = np.zeros((3, scene.model.nv))
    jacr_inner = np.zeros((3, scene.model.nv))
    jacp_outer = np.zeros((3, scene.model.nv))
    jacr_outer = np.zeros((3, scene.model.nv))
    mujoco.mj_jacGeom(scene.model, scene.data, jacp_inner, jacr_inner, inner_id)
    mujoco.mj_jacGeom(scene.model, scene.data, jacp_outer, jacr_outer, outer_id)

    jacp = (jacp_inner + jacp_outer) * 0.5
    arm_jacobian = jacp[:, arm.dof_ids]
    lhs = arm_jacobian @ arm_jacobian.T + (damping**2) * np.eye(3)
    dq = arm_jacobian.T @ np.linalg.solve(lhs, error)
    dq = np.clip(gain * dq, -max_step, max_step)

    q_target = scene.data.qpos[arm.qpos_ids] + dq
    for ctrl_id, value in zip(arm.ctrl_ids, q_target):
        scene.data.ctrl[ctrl_id] = value
    return float(np.linalg.norm(error))


def run_ik_with_gripper(
    scene: ObjectScene,
    arm: ArmControlMap,
    target_pos: np.ndarray,
    *,
    gripper_value: float,
    threshold: float = 0.035,
    max_steps: int = 700,
    record: bool = False,
    phase_name: str = "move",
    history: dict[str, list] | None = None,
    control_point: str = "site",
) -> tuple[bool, float]:
    """Move one arm toward a target while holding a gripper command."""
    distance = float("nan")
    for _ in range(max_steps):
        scene.data.ctrl[:] = scene.home_ctrl
        set_gripper(scene, arm.segment, gripper_value)
        if control_point == "grasp_midpoint":
            distance = jacobian_grasp_midpoint_step(
                scene,
                arm,
                target_pos,
                gain=0.7,
                damping=0.04,
                max_step=0.04,
            )
        else:
            distance = jacobian_ik_step(
                scene.model,
                scene.data,
                arm,
                target_pos,
                gain=0.7,
                damping=0.04,
                max_step=0.04,
            )
        set_gripper(scene, arm.segment, gripper_value)
        if record and history is not None:
            append_pick_state(scene, gripper_value, phase_name, history)
        if distance < threshold:
            return True, distance
        mujoco.mj_step(scene.model, scene.data)
    return False, distance


def close_gripper(
    scene: ObjectScene,
    arm: ArmControlMap,
    *,
    steps: int = 320,
    record: bool = False,
    history: dict[str, list] | None = None,
) -> None:
    """Close the gripper slowly while holding the current arm target."""
    current_ctrl = scene.data.ctrl.copy()
    for step in range(steps):
        alpha = step / max(steps - 1, 1)
        scene.data.ctrl[:] = current_ctrl
        value = (1.0 - alpha) * gripper_ctrl_value(arm.segment, False) + (
            alpha * gripper_ctrl_value(arm.segment, True)
        )
        set_gripper(scene, arm.segment, value)
        if record and history is not None:
            append_pick_state(scene, value, "close", history)
        mujoco.mj_step(scene.model, scene.data)


def hold_gripper(
    scene: ObjectScene,
    arm: ArmControlMap,
    *,
    steps: int = 150,
    record: bool = False,
    history: dict[str, list] | None = None,
) -> None:
    """Hold the current pose with the gripper closed."""
    current_ctrl = scene.data.ctrl.copy()
    closed = gripper_ctrl_value(arm.segment, True)
    for _ in range(steps):
        scene.data.ctrl[:] = current_ctrl
        set_gripper(scene, arm.segment, closed)
        if record and history is not None:
            append_pick_state(scene, closed, "hold", history)
        mujoco.mj_step(scene.model, scene.data)


def cube_left_finger_contact(scene: ObjectScene) -> bool:
    """Return True if the cube contacts a left finger geom."""
    for geom1, geom2, _ in contact_names(scene):
        names = f"{geom1} {geom2}"
        if "target_cube_geom" in names and "left" in names and "finger" in names:
            return True
    return False


def cube_left_nonfinger_contact(scene: ObjectScene) -> bool:
    """Return True if cube contacts left palm/wrist/arm geoms instead of fingers."""
    bad_keywords = ("ee_base", "link", "wrist")
    for geom1, geom2, _ in contact_names(scene):
        names = f"{geom1} {geom2}"
        if "target_cube_geom" not in names or "left" not in names:
            continue
        if "finger" in names:
            continue
        if any(keyword in names for keyword in bad_keywords):
            return True
    return False


def cube_left_nonfinger_contact_names(
    scene: ObjectScene,
) -> list[tuple[str, str, float]]:
    """Return cube contacts against left non-finger geoms."""
    bad_keywords = ("ee_base", "link", "wrist")
    bad_contacts = []
    for geom1, geom2, distance in contact_names(scene):
        names = f"{geom1} {geom2}"
        if "target_cube_geom" not in names or "left" not in names:
            continue
        if "finger" in names:
            continue
        if any(keyword in names for keyword in bad_keywords):
            bad_contacts.append((geom1, geom2, distance))
    return bad_contacts


def new_pick_history() -> dict[str, list]:
    """Create empty trajectory lists for pick recording."""
    return {
        "qpos": [],
        "qvel": [],
        "ctrl": [],
        "left_ee_pos": [],
        "right_ee_pos": [],
        "object_pos": [],
        "object_quat": [],
        "gripper_ctrl": [],
        "object_height": [],
        "ee_object_dist": [],
        "phase": [],
    }


def append_pick_state(
    scene: ObjectScene,
    gripper_value: float,
    phase_name: str,
    history: dict[str, list],
) -> None:
    """Append the current pick state into a history dict."""
    object_position = cube_pos(scene)
    ee_position = left_ee_pos(scene)
    history["qpos"].append(scene.data.qpos.copy())
    history["qvel"].append(scene.data.qvel.copy())
    history["ctrl"].append(scene.data.ctrl.copy())
    history["left_ee_pos"].append(ee_position)
    history["right_ee_pos"].append(right_ee_pos(scene))
    history["object_pos"].append(object_position)
    history["object_quat"].append(cube_quat(scene))
    history["gripper_ctrl"].append(gripper_value)
    history["object_height"].append(object_position[2])
    history["ee_object_dist"].append(
        float(np.linalg.norm(ee_position - object_position))
    )
    history["phase"].append(phase_name)


def run_pick_left(
    *,
    cube_position: np.ndarray | None = None,
    record: bool = False,
) -> PickResult:
    """Run a scripted left-arm pick attempt."""
    scene, arm = setup_pick_scene(cube_position)
    history = new_pick_history()
    object_start = cube_pos(scene)
    waypoints = left_pick_waypoints(object_start)
    open_value = gripper_ctrl_value("left", False)
    close_value = gripper_ctrl_value("left", True)

    run_ik_with_gripper(
        scene,
        arm,
        waypoints["pregrasp"],
        gripper_value=open_value,
        threshold=0.035,
        max_steps=700,
        record=record,
        phase_name="pregrasp",
        history=history,
        control_point="grasp_midpoint",
    )
    waypoints = left_pick_waypoints(cube_pos(scene))
    run_ik_with_gripper(
        scene,
        arm,
        waypoints["grasp"],
        gripper_value=open_value,
        threshold=0.035,
        max_steps=700,
        record=record,
        phase_name="grasp",
        history=history,
        control_point="grasp_midpoint",
    )
    close_gripper(scene, arm, record=record, history=history)
    object_after_close = cube_pos(scene)
    finger_contact = cube_left_finger_contact(scene)
    waypoints = left_pick_waypoints(cube_pos(scene))
    run_ik_with_gripper(
        scene,
        arm,
        waypoints["lift"],
        gripper_value=close_value,
        threshold=0.040,
        max_steps=900,
        record=record,
        phase_name="lift",
        history=history,
        control_point="grasp_midpoint",
    )
    hold_gripper(scene, arm, record=record, history=history)

    object_end = cube_pos(scene)
    lift_height = float(object_end[2] - object_start[2])
    final_dist = float(np.linalg.norm(left_ee_pos(scene) - object_end))
    success = lift_height > LIFT_SUCCESS_HEIGHT

    return PickResult(
        object_start=object_start,
        object_after_close=object_after_close,
        object_end=object_end,
        object_lift_height=lift_height,
        success=success,
        finger_contact=finger_contact,
        final_ee_object_dist=final_dist,
        qpos_history=np.array(history["qpos"]),
        qvel_history=np.array(history["qvel"]),
        ctrl_history=np.array(history["ctrl"]),
        left_ee_history=np.array(history["left_ee_pos"]),
        right_ee_history=np.array(history["right_ee_pos"]),
        object_pos_history=np.array(history["object_pos"]),
        object_quat_history=np.array(history["object_quat"]),
        gripper_history=np.array(history["gripper_ctrl"]),
        object_height_history=np.array(history["object_height"]),
        ee_object_dist_history=np.array(history["ee_object_dist"]),
        phase_history=np.array(history["phase"]),
    )


def sample_pick_positions(num_positions: int = 5, seed: int = 53) -> np.ndarray:
    """Sample left-side cube positions for early pick testing."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.39, 0.45, size=num_positions)
    y = rng.uniform(0.16, 0.22, size=num_positions)
    z = np.full(num_positions, 1.035)
    return np.column_stack([x, y, z])
