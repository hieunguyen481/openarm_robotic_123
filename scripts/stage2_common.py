"""Shared helpers for Stage 2 control and reach scripts."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from stage1_common import OUTPUT_DIR, load_model, name_or_empty, reset_to_keyframe


LEFT_SITE = "left_ee_control_point"
RIGHT_SITE = "right_ee_control_point"


@dataclass(frozen=True)
class ArmControlMap:
    """Resolved qpos/dof/ctrl indices for one OpenArm arm."""

    segment: str
    joint_names: list[str]
    actuator_names: list[str]
    qpos_ids: np.ndarray
    dof_ids: np.ndarray
    ctrl_ids: np.ndarray
    site_id: int


@dataclass
class ReachResult:
    """Logged result for one reach rollout."""

    target_pos: np.ndarray
    qpos_history: np.ndarray
    qvel_history: np.ndarray
    ctrl_history: np.ndarray
    ee_pos_history: np.ndarray
    distance_history: np.ndarray
    success: bool
    success_step: int


def resolve_arm(model: mujoco.MjModel, segment: str) -> ArmControlMap:
    """Resolve joint, dof, actuator, and site indices for an arm."""
    if segment not in {"left", "right"}:
        raise ValueError(f"Invalid segment: {segment}")

    joint_names = [f"openarm_{segment}_joint{i}" for i in range(1, 8)]
    actuator_names = [f"{segment}_joint{i}_ctrl" for i in range(1, 8)]
    site_name = LEFT_SITE if segment == "left" else RIGHT_SITE

    qpos_ids = []
    dof_ids = []
    ctrl_ids = []

    for joint_name, actuator_name in zip(joint_names, actuator_names):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
        )
        if joint_id == -1:
            raise ValueError(f"Joint not found: {joint_name}")
        if actuator_id == -1:
            raise ValueError(f"Actuator not found: {actuator_name}")

        qpos_ids.append(int(model.jnt_qposadr[joint_id]))
        dof_ids.append(int(model.jnt_dofadr[joint_id]))
        ctrl_ids.append(actuator_id)

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id == -1:
        raise ValueError(f"Site not found: {site_name}")

    return ArmControlMap(
        segment=segment,
        joint_names=joint_names,
        actuator_names=actuator_names,
        qpos_ids=np.array(qpos_ids, dtype=np.intp),
        dof_ids=np.array(dof_ids, dtype=np.intp),
        ctrl_ids=np.array(ctrl_ids, dtype=np.intp),
        site_id=site_id,
    )


def home_ctrl_from_qpos(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Build a full ctrl vector from the current qpos of actuator joints."""
    home_ctrl = np.zeros(model.nu)
    for i in range(model.nu):
        joint_id = int(model.actuator_trnid[i, 0])
        if joint_id >= 0:
            qpos_id = int(model.jnt_qposadr[joint_id])
            home_ctrl[i] = data.qpos[qpos_id]
    return clip_ctrl(model, home_ctrl)


def clip_ctrl(model: mujoco.MjModel, ctrl: np.ndarray) -> np.ndarray:
    """Clip a ctrl vector to actuator ranges."""
    clipped = ctrl.copy()
    for i in range(model.nu):
        if model.actuator_ctrllimited[i]:
            low, high = model.actuator_ctrlrange[i]
            clipped[i] = np.clip(clipped[i], low, high)
    return clipped


def make_ready_model() -> tuple[mujoco.MjModel, mujoco.MjData, np.ndarray]:
    """Load the model, reset to home, and return stable home controls."""
    model, data = load_model()
    reset_to_keyframe(model, data)
    home_ctrl = home_ctrl_from_qpos(model, data)
    data.ctrl[:] = home_ctrl
    mujoco.mj_forward(model, data)
    return model, data, home_ctrl


def site_position(
    model: mujoco.MjModel, data: mujoco.MjData, site_name: str
) -> np.ndarray:
    """Return the world position of a site."""
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id == -1:
        raise ValueError(f"Site not found: {site_name}")
    mujoco.mj_forward(model, data)
    return data.site_xpos[site_id].copy()


def hold_ctrl(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    ctrl: np.ndarray,
    steps: int,
) -> None:
    """Apply one ctrl vector for a number of simulation steps."""
    data.ctrl[:] = clip_ctrl(model, ctrl)
    for _ in range(steps):
        mujoco.mj_step(model, data)


def set_arm_ctrl(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmControlMap,
    target_qpos: np.ndarray,
) -> None:
    """Write a 7D arm target into the full ctrl vector."""
    for ctrl_id, value in zip(arm.ctrl_ids, target_qpos):
        data.ctrl[ctrl_id] = value
    data.ctrl[:] = clip_ctrl(model, data.ctrl)


def jacobian_ik_step(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmControlMap,
    target_pos: np.ndarray,
    *,
    gain: float = 0.5,
    damping: float = 0.05,
    max_step: float = 0.03,
) -> float:
    """Run one damped least-squares Jacobian IK update for an arm."""
    mujoco.mj_forward(model, data)
    current_pos = data.site_xpos[arm.site_id].copy()
    error = target_pos - current_pos

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, arm.site_id)

    arm_jacobian = jacp[:, arm.dof_ids]
    lhs = arm_jacobian @ arm_jacobian.T + (damping**2) * np.eye(3)
    dq = arm_jacobian.T @ np.linalg.solve(lhs, error)
    dq = np.clip(gain * dq, -max_step, max_step)

    q_target = data.qpos[arm.qpos_ids] + dq
    set_arm_ctrl(model, data, arm, q_target)
    return float(np.linalg.norm(error))


def run_reach_episode(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    home_ctrl: np.ndarray,
    arm: ArmControlMap,
    target_pos: np.ndarray,
    *,
    threshold: float = 0.03,
    max_steps: int = 700,
    gain: float = 0.5,
    damping: float = 0.05,
    max_step: float = 0.03,
) -> ReachResult:
    """Run a reach episode and log state/action/distance histories."""
    qpos_history = []
    qvel_history = []
    ctrl_history = []
    ee_pos_history = []
    distance_history = []
    success = False
    success_step = -1

    for step in range(max_steps):
        data.ctrl[:] = home_ctrl
        distance = jacobian_ik_step(
            model,
            data,
            arm,
            target_pos,
            gain=gain,
            damping=damping,
            max_step=max_step,
        )

        qpos_history.append(data.qpos.copy())
        qvel_history.append(data.qvel.copy())
        ctrl_history.append(data.ctrl.copy())
        ee_pos_history.append(data.site_xpos[arm.site_id].copy())
        distance_history.append(distance)

        if distance < threshold:
            success = True
            success_step = step
            break

        mujoco.mj_step(model, data)

    return ReachResult(
        target_pos=target_pos.copy(),
        qpos_history=np.array(qpos_history),
        qvel_history=np.array(qvel_history),
        ctrl_history=np.array(ctrl_history),
        ee_pos_history=np.array(ee_pos_history),
        distance_history=np.array(distance_history),
        success=success,
        success_step=success_step,
    )


def run_random_reach_targets(
    *,
    segment: str,
    num_targets: int = 20,
    threshold: float = 0.02,
    min_start_distance: float = 0.04,
    seed: int = 7,
    max_steps: int = 700,
    gain: float = 0.7,
    damping: float = 0.04,
    max_step: float = 0.04,
    max_attempts_per_target: int = 100,
) -> tuple[list[list[object]], dict[str, float]]:
    """Run random nearby reach targets while skipping already-close targets."""
    rng = np.random.default_rng(seed)
    offset_low = np.array([-0.04, -0.04, -0.03])
    offset_high = np.array([0.06, 0.06, 0.05])

    rows: list[list[object]] = []
    final_distances = []
    successes = []
    steps_to_success = []
    skipped_close = 0

    for target_id in range(num_targets):
        accepted = False
        attempts = 0
        while not accepted:
            attempts += 1
            if attempts > max_attempts_per_target:
                raise RuntimeError(
                    f"Could not sample a target for {segment} after "
                    f"{max_attempts_per_target} attempts"
                )

            model, data, home_ctrl = make_ready_model()
            arm = resolve_arm(model, segment)
            start_pos = data.site_xpos[arm.site_id].copy()
            offset = rng.uniform(offset_low, offset_high)
            if segment == "right":
                offset[1] *= -1.0
            target_pos = start_pos + offset
            start_distance = float(np.linalg.norm(target_pos - start_pos))
            if start_distance < min_start_distance:
                skipped_close += 1
                continue
            accepted = True

        result = run_reach_episode(
            model,
            data,
            home_ctrl,
            arm,
            target_pos,
            threshold=threshold,
            max_steps=max_steps,
            gain=gain,
            damping=damping,
            max_step=max_step,
        )

        final_distance = float(result.distance_history[-1])
        final_distances.append(final_distance)
        successes.append(result.success)
        steps_to_success.append(result.success_step if result.success else np.nan)
        rows.append(
            [
                target_id,
                *target_pos.tolist(),
                result.distance_history[0],
                final_distance,
                result.success,
                result.success_step,
                len(result.distance_history),
                attempts,
            ]
        )

    success_rate = float(np.mean(successes))
    mean_steps = float(np.nanmean(steps_to_success)) if any(successes) else float("nan")
    summary = {
        "num_targets": float(num_targets),
        "threshold": threshold,
        "min_start_distance": min_start_distance,
        "success_rate": success_rate,
        "mean_final_distance": float(np.mean(final_distances)),
        "min_final_distance": float(np.min(final_distances)),
        "max_final_distance": float(np.max(final_distances)),
        "mean_steps_to_success": mean_steps,
        "skipped_close_targets": float(skipped_close),
        "max_steps": float(max_steps),
        "gain": gain,
        "damping": damping,
        "max_step": max_step,
    }
    return rows, summary


def describe_sites(model: mujoco.MjModel, data: mujoco.MjData) -> str:
    """Return a text summary of EE site positions."""
    left_pos = site_position(model, data, LEFT_SITE)
    right_pos = site_position(model, data, RIGHT_SITE)
    return (
        f"{LEFT_SITE}: {left_pos}\n"
        f"{RIGHT_SITE}: {right_pos}\n"
        f"distance_between_ee: {np.linalg.norm(left_pos - right_pos):.6f}"
    )


def actuator_summary(model: mujoco.MjModel, ctrl_ids: np.ndarray) -> list[str]:
    """Return actuator names for a set of ctrl indices."""
    return [
        f"ctrl[{int(i)}] = {name_or_empty(model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(i))}"
        for i in ctrl_ids
    ]


OUTPUT_DIR.mkdir(exist_ok=True)
