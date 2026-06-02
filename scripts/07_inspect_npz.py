"""Inspect the logged Stage 1 NPZ dataset."""

import numpy as np

from stage1_common import OUTPUT_DIR

data = np.load(OUTPUT_DIR / "episode_000.npz")

print("Keys:", list(data.keys()))

for key in data.keys():
    arr = data[key]
    print(key, arr.shape, arr.dtype)
