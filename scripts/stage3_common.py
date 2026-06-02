"""Shared helpers for Stage 3 object interaction scripts."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from stage1_common import OUTPUT_DIR, ROOT, reset_to_keyframe
from stage2_common import (
    LEFT_SITE,
    RIGHT_SITE,
    ArmControlMap,
    home_ctrl_from_qpos,
    jacobian_ik_step,
)


OBJECT_XML_PATH = ROOT / "v2" / "cell_object.xml"
CUBE_BODY = "target_cube"
CUBE_JOINT = "target_cube_freejoint"
CUBE_GEOM = "target_cube_geom"
CUBE_INITIAL_POS = np.array([0.42, 0.19, 1.035])
CUBE_INITIAL_QUAT = np.array([1.0, 0.0, 0.0, 0.0])


@dataclass
class ObjectScene:
    """Loaded object scene with frequently used ids."""

    model: mujoco.MjModel
    data: mujoco.MjData
    home_ctrl: np.ndarray
    cube_body_id: int
    cube_joint_id: int
    cube_qpos_id: int
    cube_dof_id: int
    left_site_id: int
    right_site_id: int


@dataclass
class PushResult:
    """Summary and optional trajectory from one push rollout."""

    segment: str
    object_start: np.ndarray
    object_end: np.ndarray
    object_displacement: np.ndarray
    push_distance_x: float
    success: bool
    final_ee_pos: np.ndarray
    push_ik_success: bool
    push_final_distance: float
    qpos_history: np.ndarray
    qvel_history: np.ndarray
    ctrl_history: np.ndarray
    left_ee_history: np.ndarray
    right_ee_history: np.ndarray
    object_pos_history: np.ndarray
    object_quat_history: np.ndarray
    waypoint_distance_history: np.ndarray
    phase_history: np.ndarray


def load_object_scene() -> ObjectScene:
    """Load the object scene and return useful ids."""
    if not OBJECT_XML_PATH.exists():
        raise FileNotFoundError(
            f"{OBJECT_XML_PATH} not found. Run scripts/18_create_object_scene.py first."
        )

    model = mujoco.MjModel.from_xml_path(str(OBJECT_XML_PATH))
    data = mujoco.MjData(model)
    reset_to_keyframe(model, data)

    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, CUBE_BODY)
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, CUBE_JOINT)
    left_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, LEFT_SITE)
    right_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, RIGHT_SITE)

    if cube_body_id == -1:
        raise ValueError(f"Cannot find body: {CUBE_BODY}")
    if cube_joint_id == -1:
        raise ValueError(f"Cannot find joint: {CUBE_JOINT}")
    if left_site_id == -1:
        raise ValueError(f"Cannot find site: {LEFT_SITE}")
    if right_site_id == -1:
        raise ValueError(f"Cannot find site: {RIGHT_SITE}")

    cube_qpos_id = int(model.jnt_qposadr[cube_joint_id])
    cube_dof_id = int(model.jnt_dofadr[cube_joint_id])
    data.qpos[cube_qpos_id : cube_qpos_id + 3] = CUBE_INITIAL_POS
    data.qpos[cube_qpos_id + 3 : cube_qpos_id + 7] = CUBE_INITIAL_QUAT
    data.qvel[cube_dof_id : cube_dof_id + 6] = 0.0
    home_ctrl = home_ctrl_from_qpos(model, data)
    data.ctrl[:] = home_ctrl
    mujoco.mj_forward(model, data)

    return ObjectScene(
        model=model,
        data=data,
        home_ctrl=home_ctrl,
        cube_body_id=cube_body_id,
        cube_joint_id=cube_joint_id,
        cube_qpos_id=cube_qpos_id,
        cube_dof_id=cube_dof_id,
        left_site_id=left_site_id,
        right_site_id=right_site_id,
    )


def set_cube_pose(
    scene: ObjectScene,
    pos: np.ndarray,
    quat: np.ndarray | None = None,
) -> None:
    """Place the cube at a chosen pose and clear its velocity."""
    scene.data.qpos[scene.cube_qpos_id : scene.cube_qpos_id + 3] = pos
    scene.data.qpos[scene.cube_qpos_id + 3 : scene.cube_qpos_id + 7] = (
        CUBE_INITIAL_QUAT if quat is None else quat
    )
    scene.data.qvel[scene.cube_dof_id : scene.cube_dof_id + 6] = 0.0
    mujoco.mj_forward(scene.model, scene.data)


def settle(scene: ObjectScene, steps: int = 300) -> None:
    """Let physics settle while holding home control."""
    scene.data.ctrl[:] = scene.home_ctrl
    for _ in range(steps):
        mujoco.mj_step(scene.model, scene.data)
    mujoco.mj_forward(scene.model, scene.data)


def cube_pos(scene: ObjectScene) -> np.ndarray:
    """Return target cube world position."""
    mujoco.mj_forward(scene.model, scene.data)
    return scene.data.xpos[scene.cube_body_id].copy()


def cube_quat(scene: ObjectScene) -> np.ndarray:
    """Return target cube freejoint quaternion from qpos."""
    return scene.data.qpos[scene.cube_qpos_id + 3 : scene.cube_qpos_id + 7].copy()


def left_ee_pos(scene: ObjectScene) -> np.ndarray:
    """Return left end-effector site position."""
    mujoco.mj_forward(scene.model, scene.data)
    return scene.data.site_xpos[scene.left_site_id].copy()


def right_ee_pos(scene: ObjectScene) -> np.ndarray:
    """Return right end-effector site position."""
    mujoco.mj_forward(scene.model, scene.data)
    return scene.data.site_xpos[scene.right_site_id].copy()


def run_ik_to_pos(
    scene: ObjectScene,
    arm: ArmControlMap,
    target_pos: np.ndarray,
    *,
    threshold: float = 0.025,
    max_steps: int = 700,
    gain: float = 0.7,
    damping: float = 0.04,
    max_step: float = 0.04,
) -> tuple[bool, list[float]]:
    """Run IK toward one target and return success plus distance history."""
    distances: list[float] = []
    for _ in range(max_steps):
        scene.data.ctrl[:] = scene.home_ctrl
        distance = jacobian_ik_step(
            scene.model,
            scene.data,
            arm,
            target_pos,
            gain=gain,
            damping=damping,
            max_step=max_step,
        )
        distances.append(distance)
        if distance < threshold:
            return True, distances
        mujoco.mj_step(scene.model, scene.data)
    return False, distances


def push_waypoints(
    object_pos: np.ndarray,
    segment: str = "left",
) -> list[tuple[str, np.ndarray, float, int]]:
    """Return approach, touch, and push waypoints for a visually aligned +x push."""
    side_y = -0.02 if segment == "left" else 0.02
    return [
        ("approach", object_pos + np.array([-0.14, side_y, 0.08]), 0.035, 500),
        ("touch", object_pos + np.array([-0.10, side_y, 0.05]), 0.030, 500),
        ("push", object_pos + np.array([0.02, side_y, 0.05]), 0.035, 1000),
    ]


def run_push_episode(
    scene: ObjectScene,
    arm: ArmControlMap,
    *,
    success_threshold: float = 0.03,
    record: bool = False,
) -> PushResult:
    """Push the cube along +x with one arm, optionally logging the trajectory."""
    object_start = cube_pos(scene)
    qpos_history = []
    qvel_history = []
    ctrl_history = []
    left_ee_history = []
    right_ee_history = []
    object_pos_history = []
    object_quat_history = []
    waypoint_distance_history = []
    phase_history = []
    push_ik_success = False
    push_final_distance = float("nan")

    for phase_name, target_pos, threshold, max_steps in push_waypoints(
        object_start,
        arm.segment,
    ):
        for _ in range(max_steps):
            scene.data.ctrl[:] = scene.home_ctrl
            distance = jacobian_ik_step(
                scene.model,
                scene.data,
                arm,
                target_pos,
                gain=0.7,
                damping=0.04,
                max_step=0.04,
            )

            if record:
                qpos_history.append(scene.data.qpos.copy())
                qvel_history.append(scene.data.qvel.copy())
                ctrl_history.append(scene.data.ctrl.copy())
                left_ee_history.append(left_ee_pos(scene))
                right_ee_history.append(right_ee_pos(scene))
                object_pos_history.append(cube_pos(scene))
                object_quat_history.append(cube_quat(scene))
                waypoint_distance_history.append(distance)
                phase_history.append(phase_name)

            if distance < threshold and phase_name != "push":
                break

            mujoco.mj_step(scene.model, scene.data)

        if phase_name == "push":
            push_final_distance = distance
            push_ik_success = distance < threshold

    object_end = cube_pos(scene)
    object_displacement = object_end - object_start
    push_distance_x = float(object_displacement[0])
    final_ee = left_ee_pos(scene) if arm.segment == "left" else right_ee_pos(scene)

    return PushResult(
        segment=arm.segment,
        object_start=object_start,
        object_end=object_end,
        object_displacement=object_displacement,
        push_distance_x=push_distance_x,
        success=push_distance_x > success_threshold,
        final_ee_pos=final_ee,
        push_ik_success=push_ik_success,
        push_final_distance=float(push_final_distance),
        qpos_history=np.array(qpos_history),
        qvel_history=np.array(qvel_history),
        ctrl_history=np.array(ctrl_history),
        left_ee_history=np.array(left_ee_history),
        right_ee_history=np.array(right_ee_history),
        object_pos_history=np.array(object_pos_history),
        object_quat_history=np.array(object_quat_history),
        waypoint_distance_history=np.array(waypoint_distance_history),
        phase_history=np.array(phase_history),
    )


def sample_cube_positions(
    *,
    segment: str,
    num_positions: int = 20,
    seed: int = 11,
) -> np.ndarray:
    """Sample reachable cube positions on the table for push robustness tests."""
    if segment not in {"left", "right"}:
        raise ValueError(f"Invalid segment: {segment}")

    rng = np.random.default_rng(seed)
    x = rng.uniform(0.39, 0.47, size=num_positions)
    if segment == "left":
        y = rng.uniform(0.13, 0.23, size=num_positions)
    else:
        y = rng.uniform(-0.23, -0.13, size=num_positions)
    z = np.full(num_positions, CUBE_INITIAL_POS[2])
    return np.column_stack([x, y, z])


def run_random_push_objects(
    *,
    segment: str,
    num_positions: int = 20,
    seed: int = 11,
    success_threshold: float = 0.03,
) -> tuple[list[list[object]], dict[str, float]]:
    """Run repeated push trials with randomized cube positions."""
    from stage2_common import resolve_arm

    positions = sample_cube_positions(
        segment=segment,
        num_positions=num_positions,
        seed=seed,
    )
    rows: list[list[object]] = []
    push_distances = []
    successes = []

    for trial_id, requested_pos in enumerate(positions):
        scene = load_object_scene()
        set_cube_pose(scene, requested_pos)
        settle(scene)
        arm = resolve_arm(scene.model, segment)
        result = run_push_episode(
            scene,
            arm,
            success_threshold=success_threshold,
            record=False,
        )
        push_distances.append(result.push_distance_x)
        successes.append(result.success)
        rows.append(
            [
                trial_id,
                *requested_pos.tolist(),
                *result.object_start.tolist(),
                *result.object_end.tolist(),
                result.push_distance_x,
                result.success,
                result.push_ik_success,
                result.push_final_distance,
            ]
        )

    failed_cases = int(np.sum(np.logical_not(successes)))
    summary = {
        "num_positions": float(num_positions),
        "success_threshold": success_threshold,
        "success_rate": float(np.mean(successes)),
        "mean_push_distance_x": float(np.mean(push_distances)),
        "min_push_distance_x": float(np.min(push_distances)),
        "max_push_distance_x": float(np.max(push_distances)),
        "failed_cases": float(failed_cases),
        "seed": float(seed),
    }
    return rows, summary


def contact_names(scene: ObjectScene) -> list[tuple[str, str, float]]:
    """Return current contact geom names and distances."""
    contacts = []
    for i in range(scene.data.ncon):
        contact = scene.data.contact[i]
        geom1 = mujoco.mj_id2name(scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        geom2 = mujoco.mj_id2name(scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        contacts.append((geom1 or "", geom2 or "", float(contact.dist)))
    return contacts


def has_cube_contact(scene: ObjectScene) -> bool:
    """Return True if current contacts include the target cube geom."""
    return any(CUBE_GEOM in pair[:2] for pair in contact_names(scene))


def write_lines(path: Path, lines: list[str]) -> None:
    """Write text lines with UTF-8 encoding."""
    OUTPUT_DIR.mkdir(exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")
