"""Inspect the object scene and confirm target cube ids exist."""

import mujoco

from stage3_common import CUBE_BODY, CUBE_GEOM, CUBE_JOINT, OBJECT_XML_PATH

model = mujoco.MjModel.from_xml_path(str(OBJECT_XML_PATH))

print("nq =", model.nq)
print("nv =", model.nv)
print("nu =", model.nu)
print("nbody =", model.nbody)
print("njnt =", model.njnt)
print("ngeom =", model.ngeom)

print("\n===== CHECK OBJECT IDS =====")
for obj_type, obj_name in [
    (mujoco.mjtObj.mjOBJ_BODY, CUBE_BODY),
    (mujoco.mjtObj.mjOBJ_JOINT, CUBE_JOINT),
    (mujoco.mjtObj.mjOBJ_GEOM, CUBE_GEOM),
]:
    obj_id = mujoco.mj_name2id(model, obj_type, obj_name)
    print(obj_name, "id =", obj_id)

print("\n===== BODIES CONTAINING target =====")
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if name and "target" in name:
        print(i, name)
