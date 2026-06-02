"""Save the actuator-to-joint mapping to outputs/actuator_map.txt."""

import mujoco

from stage1_common import OUTPUT_DIR, load_model, name_or_empty

model, _data = load_model()
lines: list[str] = []


def log(text: object) -> None:
    """Print a line and collect it for the output file."""
    line = str(text)
    print(line)
    lines.append(line)


log("===== ACTUATOR TO JOINT MAP =====")

for i in range(model.nu):
    actuator_name = name_or_empty(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    joint_id = int(model.actuator_trnid[i, 0])
    joint_name = name_or_empty(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    ctrlrange = model.actuator_ctrlrange[i]
    log(
        f"ctrl[{i}] -> actuator={actuator_name} -> "
        f"joint={joint_name} -> ctrlrange={ctrlrange}"
    )

OUTPUT_DIR.mkdir(exist_ok=True)
(OUTPUT_DIR / "actuator_map.txt").write_text("\n".join(lines), encoding="utf-8")
print("\nSaved to outputs/actuator_map.txt")
