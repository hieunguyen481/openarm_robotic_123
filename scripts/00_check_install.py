"""Check the Stage 1 Python and package installation."""

import sys

import mujoco
import numpy as np

print("Python:", sys.version)
print("MuJoCo imported OK")
print("MuJoCo:", mujoco.__version__)
print("NumPy:", np.__version__)

try:
    import openarm_mujoco
    import openarm_mujoco.v2 as openarm_mujoco_v2

    print("openarm_mujoco imported OK")
    print("openarm_mujoco path:", openarm_mujoco.__file__)
    print("openarm_mujoco.v2 path:", openarm_mujoco_v2.__file__)
except Exception as exc:
    print("Cannot import openarm_mujoco:", repr(exc))
