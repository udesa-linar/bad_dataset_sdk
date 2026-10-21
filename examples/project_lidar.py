"""Project onto cam1 the LiDAR scan that contains the trigger of the front frame nearest to a time.

    python project_lidar.py /data/buenos_aires loop_caba 1774888760 out.png [--splits split_02]
"""
import argparse

import cv2
import numpy as np

from badataset import Dataset, calib, sync

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('root'); ap.add_argument('sequence'); ap.add_argument('time', type=float); ap.add_argument('out')
ap.add_argument('--splits', nargs='*')
a = ap.parse_args()
seq = Dataset(a.root).sequence(a.sequence, a.splits)
frames = [f for f in sync.front_frames(seq, start=a.time - 0.3, end=a.time + 0.3) if f.cloud is not None]
if not frames:
    raise SystemExit('no front frame with its LiDAR scan within 0.3 s of that time (the front pair has gaps in caba, see the '
                     'known issues of the dataset card)')
f = min(frames, key=lambda f: abs(f.t - a.time))
split = next(s for s in seq.splits if any(True for _ in s.messages('/cam1/camera_info', f.t - 0.2, f.t + 0.2)))
T = calib.tf_tree(split); K, D, size = calib.intrinsics(split, 'cam1')
uv, depth, _ = calib.project(f.cloud.xyz, calib.transform(T, 'lidar', 'cam1'), K, D, size)
colors = cv2.applyColorMap(np.clip(depth / 30 * 255, 0, 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO)[:, 0]
img = f.cam1.copy()
for (u, v), c in zip(uv.astype(int), colors):
    cv2.circle(img, (u, v), 2, tuple(int(x) for x in c), -1)
cv2.imwrite(a.out, img)
print(f'trigger {f.t:.3f}, scan ends {f.cloud.t - f.t:+.3f} s later, {len(uv)} points projected -> {a.out}')
