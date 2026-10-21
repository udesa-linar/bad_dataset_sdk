#!/usr/bin/env python3
"""Compare the output of the calibration scripts with the calibration released in the dataset.

Reads the results written by calibration/scripts (default calibration/output) and the released calibration of a
split (camera_info for the intrinsics, /tf_static for the extrinsics) and prints, per item, the difference:
  - intrinsics of cam1..cam5 (Kalibr: intrinsics/<cam>/ for cam2-4, stereo/ for cam1 and cam5): fx, fy, cx, cy [px],
    distortion;
  - stereo cam1-cam5 (Kalibr T_cn_cnm1) against the released cam1 -> cam5 transform: translation [cm], rotation [deg];
  - LiDAR-camera cam1..cam4 (koide calib.json, T_lidar_camera) against the released lidar -> camN; cam5 is placed
    in the released tree through cam1 and the stereo calibration, so its direct LiDAR-camera result is a cross-check
    (the paper reports 2.5 cm and 0.34 deg);
  - IMU noise (Allan deviation) against the values of the paper.
A rerun with the released correspondences and seeded Kalibr should reproduce the release to numerical precision.

Usage: verify.py --split /data/buenos_aires/caba/split_03 [--output calibration/output]
"""
import argparse, json, os, sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from badataset import calib                     # noqa: E402
from badataset.reader import Split              # noqa: E402

PAPER_IMU_NOISE = {'accelerometer_noise_density': 9.58e-4, 'gyroscope_noise_density': 4.37e-4,
                   'accelerometer_random_walk': 4.80e-6, 'gyroscope_random_walk': 1.24e-6}


def pose_diff(A, B):
    """translation [cm] and rotation [deg] between two 4x4 poses"""
    D = np.linalg.inv(A) @ B
    return 100 * np.linalg.norm(A[:3, 3] - B[:3, 3]), np.degrees(Rot.from_matrix(D[:3, :3]).magnitude())


def T_from_xyzq(v):
    M = np.eye(4); M[:3, :3] = Rot.from_quat(v[3:7]).as_matrix(); M[:3, 3] = v[:3]; return M


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--split', required=True, help='any released split with cameras/ and tf_static/')
    ap.add_argument('--output', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output'))
    a = ap.parse_args()
    split = Split(a.split); T = calib.tf_tree(split); out = a.output; ok = True

    print('Intrinsics [px] (output - released)')
    chains = {}
    st = os.path.join(out, 'stereo', 'aq_cam1_cam5-camchain.yaml')
    if os.path.exists(st):
        y = yaml.safe_load(open(st)); chains['cam1'], chains['cam5'] = y['cam0'], y['cam1']
    for c in ('cam2', 'cam3', 'cam4'):
        f = os.path.join(out, 'intrinsics', c, f'aq_{c}-camchain.yaml')
        if os.path.exists(f):
            chains[c] = yaml.safe_load(open(f))['cam0']
    for c in ('cam1', 'cam2', 'cam3', 'cam4', 'cam5'):
        if c not in chains:
            print(f'  {c}: not calibrated (run the scripts first)'); ok = False; continue
        K, D, _ = calib.intrinsics(split, c)
        fx, fy, cx, cy = chains[c]['intrinsics']; d = np.array(chains[c]['distortion_coeffs'])
        dk = np.array([fx - K[0, 0], fy - K[1, 1], cx - K[0, 2], cy - K[1, 2]])
        dd = np.max(np.abs(d - D[:4]))
        print(f'  {c}: dfx {dk[0]:+.3f}  dfy {dk[1]:+.3f}  dcx {dk[2]:+.3f}  dcy {dk[3]:+.3f}  max |d dist| {dd:.2e}')

    print('Stereo cam1 -> cam5')
    if os.path.exists(st):
        T_c5_c1 = np.array(y['cam1']['T_cn_cnm1'])
        dt, dr = pose_diff(np.linalg.inv(T_c5_c1), calib.transform(T, 'cam1', 'cam5'))
        print(f'  baseline {np.linalg.norm(T_c5_c1[:3, 3]):.4f} m; vs released: {dt:.2f} cm, {dr:.3f} deg')

    print('LiDAR -> camera')
    for c in ('cam1', 'cam2', 'cam3', 'cam4', 'cam5'):
        f = os.path.join(out, 'lidar_camera', c, 'calib.json')
        if not os.path.exists(f):
            print(f'  {c}: not calibrated'); ok = False; continue
        M = T_from_xyzq(np.array(json.load(open(f))['results']['T_lidar_camera']))
        dt, dr = pose_diff(M, calib.transform(T, 'lidar', c))
        note = '  (cross-check: the released cam5 is placed through cam1 and the stereo pair)' if c == 'cam5' else ''
        print(f'  {c}: {dt:.2f} cm, {dr:.3f} deg{note}')

    print('IMU noise (output vs paper)')
    f = os.path.join(out, 'allan', 'imu_noise.yaml')
    if os.path.exists(f):
        y = yaml.safe_load(open(f))
        for k, v in PAPER_IMU_NOISE.items():
            print(f'  {k}: {y[k]:.3e} (paper {v:.2e}, {100 * (y[k] / v - 1):+.1f} %)')
    else:
        print('  not computed'); ok = False
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
