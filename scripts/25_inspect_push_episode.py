"""Inspect the recorded push episode NPZ."""

import numpy as np

from stage1_common import OUTPUT_DIR

path = OUTPUT_DIR / "push_episode_000.npz"
data = np.load(path)

print("Keys:", list(data.keys()))
for key in data.keys():
    arr = data[key]
    print(key, arr.shape, arr.dtype)

print("push_distance_x =", float(data["push_distance_x"]))
print("success =", bool(data["success"]))
