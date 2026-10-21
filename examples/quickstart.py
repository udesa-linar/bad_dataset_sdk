"""Quick start: read a split, look at a synchronized front frame with the LiDAR projected on it, and evaluate.

    python quickstart.py /data/buenos_aires caba split_03 out/
writes out/front.png (cam1 | cam5, LiDAR points colored by range on cam1) and prints the agreement of the reference
trajectory with the RTK receiver's fixes of that split.
"""
import os
import sys

import cv2
import numpy as np

from badataset import Dataset, calib, evaluate, sync
from badataset.groundtruth import Reference

if len(sys.argv) != 5:
    sys.exit(__doc__)
root, seq_name, split_name, out = sys.argv[1:5]
os.makedirs(out, exist_ok=True)
seq = Dataset(root).sequence(seq_name, splits=[split_name])
split = seq.splits[0]

# 1. calibration
T = calib.tf_tree(split)
K, D, size = calib.intrinsics(split, 'cam1')
print('cam1-cam5 baseline %.4f m' % np.linalg.norm(calib.transform(T, 'cam1', 'cam5')[:3, 3]))

# 2. a front stereo pair with the LiDAR scan of its trigger (the first one of the split that has both)
f = next(fr for fr in sync.front_frames(seq) if fr.cloud is not None and fr.cam5 is not None)
uv, depth, _ = calib.project(f.cloud.xyz, calib.transform(T, 'lidar', 'cam1'), K, D, size)
img = f.cam1.copy()
colors = cv2.applyColorMap(np.clip(depth / 40 * 255, 0, 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO)[:, 0]
for (u, v), c in zip(uv.astype(int), colors):
    cv2.circle(img, (u, v), 1, tuple(int(x) for x in c), -1)
cv2.imwrite(os.path.join(out, 'front.png'), np.hstack([img, f.cam5]) if f.cam5 is not None else img)
print(f'trigger {f.t:.3f}: scan {f.cloud.t - f.t:+.3f} s later, {len(uv)} LiDAR points on cam1 -> {out}/front.png')

# 3. the reference trajectory against the RTK receiver's fixes
ref = Reference(seq)
r = evaluate.gnss_errors(seq, ref.traj, ref.origin)
for k in ('fixed', 'float', 'none'):
    if k in r:
        print(f'{k:6s} epochs: {r[k]["n"]:5d}, horizontal median {r[k]["h_median"]:.3f} m')
