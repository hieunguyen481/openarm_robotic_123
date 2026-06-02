"""Inspect OpenArm MuJoCo model structure and save it to outputs/model_info.txt."""

import mujoco

from stage1_common import OUTPUT_DIR, XML_PATH, load_model, name_or_empty

model, _data = load_model()
out_lines: list[str] = []


def log(text: object = "") -> None:
    """Print a line and collect it for the output file."""
    line = str(text)
    print(line)
    out_lines.append(line)


log("===== BASIC INFO =====")
log(f"XML_PATH = {XML_PATH}")
log(f"nq = {model.nq}")
log(f"nv = {model.nv}")
log(f"nu = {model.nu}")
log(f"nbody = {model.nbody}")
log(f"njnt = {model.njnt}")
log(f"ngeom = {model.ngeom}")
log(f"nsite = {model.nsite}")
log(f"ncam = {model.ncam}")

log("\n===== JOINTS =====")
for i in range(model.njnt):
    name = name_or_empty(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    qposadr = int(model.jnt_qposadr[i])
    dofadr = int(model.jnt_dofadr[i])
    jtype = int(model.jnt_type[i])
    limited = bool(model.jnt_limited[i])
    rng = model.jnt_range[i]
    log(
        f"{i:02d} | {name} | qposadr={qposadr} | dofadr={dofadr} "
        f"| type={jtype} | limited={limited} | range={rng}"
    )

log("\n===== ACTUATORS =====")
for i in range(model.nu):
    name = name_or_empty(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    joint_id = int(model.actuator_trnid[i, 0])
    joint_name = name_or_empty(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    ctrlrange = model.actuator_ctrlrange[i]
    ctrllimited = bool(model.actuator_ctrllimited[i])
    log(
        f"{i:02d} | {name} | joint={joint_name} | trnid={model.actuator_trnid[i]} "
        f"| ctrllimited={ctrllimited} | ctrlrange={ctrlrange}"
    )

log("\n===== BODIES =====")
for i in range(model.nbody):
    log(f"{i:02d} | {name_or_empty(model, mujoco.mjtObj.mjOBJ_BODY, i)}")

log("\n===== SITES =====")
for i in range(model.nsite):
    log(f"{i:02d} | {name_or_empty(model, mujoco.mjtObj.mjOBJ_SITE, i)}")

log("\n===== CAMERAS =====")
for i in range(model.ncam):
    log(f"{i:02d} | {name_or_empty(model, mujoco.mjtObj.mjOBJ_CAMERA, i)}")

OUTPUT_DIR.mkdir(exist_ok=True)
(OUTPUT_DIR / "model_info.txt").write_text("\n".join(out_lines), encoding="utf-8")
print("\nSaved to outputs/model_info.txt")
