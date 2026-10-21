"""The reference trajectory of a sequence, read from its ``ground_truth`` bags.

Each split with a reference has ``ground_truth/`` with ``/ground_truth/pose``
(``geometry_msgs/PoseWithCovarianceStamped``): the pose of the ``imu`` frame in a local East-North-Up frame
``enu`` at every estimator update (~14 Hz), with its covariance. The WGS-84 origin of ``enu`` is in the bag
metadata (``metadata.yaml``, ``custom_data``). Splits where the vehicle is parked have no reference.

    from badataset import Dataset
    from badataset.groundtruth import Reference
    ref = Reference(Dataset('/data/buenos_aires').sequence('loop_caba'))
    ref.traj                       # [N, 8] t x y z qx qy qz qw, IMU in ENU (TUM layout)
    ref.in_frame('lidar')          # the same trajectory for the LiDAR frame, through /tf_static
    ref.at([t0, t1])               # interpolated poses
    ref.save_tum('loop_caba_imu.txt')
"""
import os

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot

from . import calib, evaluate


class Reference:
    def __init__(self, sequence):
        self.sequence = sequence
        rows, cov, self.origin = [], [], None
        for split in sequence.splits:
            d = os.path.join(split.path, 'ground_truth')
            if not os.path.isdir(d):
                continue
            meta = yaml.safe_load(open(os.path.join(d, 'metadata.yaml')))['rosbag2_bagfile_information']
            c = meta.get('custom_data', {})
            o = (float(c['enu_origin_lat']), float(c['enu_origin_lon']), float(c['enu_origin_alt']))
            if self.origin is None:
                self.origin = o
            elif not np.allclose(o, self.origin):
                raise ValueError(f'{split.name}: ENU origin {o} differs from {self.origin}')
            for t, m in split.messages('/ground_truth/pose'):
                p, q = m.pose.pose.position, m.pose.pose.orientation
                rows.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w)); cov.append(np.array(m.pose.covariance).reshape(6, 6))
        if not rows:
            raise FileNotFoundError(f'{sequence.name}: no ground_truth bag downloaded')
        a = np.array(rows); order = np.argsort(a[:, 0], kind='stable')
        self.traj = a[order]
        self.covariance = np.array(cov)[order]       # [N, 6, 6]: x y z, then rotations about the enu axes
        self.frame = 'imu'

    @property
    def t(self):
        return self.traj[:, 0]

    def at(self, t):
        """poses at times t inside the span: (t [M], position [M,3], scipy Rotation, mask into t)"""
        return evaluate.interp(self.traj, np.atleast_1d(np.asarray(t, float)))

    def in_frame(self, frame, tf=None):
        """[N, 8] trajectory of `frame` (any frame of /tf_static) in enu, from the reference IMU pose"""
        tf = tf if tf is not None else self.sequence.tf_static()
        M = calib.transform(tf, self.frame, frame)           # pose of `frame` in the imu frame
        R = Rot.from_quat(self.traj[:, 4:8])
        p = self.traj[:, 1:4] + R.apply(M[:3, 3])
        q = (R * Rot.from_matrix(M[:3, :3])).as_quat()
        return np.c_[self.traj[:, 0], p, q]

    def save_tum(self, path, frame=None):
        evaluate.save_tum(path, self.traj if frame is None else self.in_frame(frame))

    def to_wgs84(self, enu):
        """[N,3] ENU points (m) -> [N,3] latitude, longitude [deg], ellipsoidal height [m] (WGS-84)"""
        lat0, lon0, h0 = self.origin
        return enu_to_wgs84(np.asarray(enu, float), lat0, lon0, h0)


_A, _E2 = 6378137.0, 6.69437999014e-3


def _ecef(lat, lon, h):
    la, lo = np.radians(lat), np.radians(lon); n = _A / np.sqrt(1 - _E2 * np.sin(la) ** 2)
    return np.stack([(n + h) * np.cos(la) * np.cos(lo), (n + h) * np.cos(la) * np.sin(lo), (n * (1 - _E2) + h) * np.sin(la)], -1)


def _enu_basis(lat0, lon0):
    la, lo = np.radians(lat0), np.radians(lon0)
    return np.array([[-np.sin(lo), np.cos(lo), 0],
                     [-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)],
                     [np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)]])


def wgs84_to_enu(lat, lon, h, lat0, lon0, h0):
    """WGS-84 (deg, deg, m) -> local ENU [m] about (lat0, lon0, h0)"""
    return (_ecef(lat, lon, h) - _ecef(lat0, lon0, h0)) @ _enu_basis(lat0, lon0).T


def enu_to_wgs84(enu, lat0, lon0, h0):
    x = _ecef(lat0, lon0, h0) + np.atleast_2d(enu) @ _enu_basis(lat0, lon0)
    lon = np.degrees(np.arctan2(x[:, 1], x[:, 0])); p = np.hypot(x[:, 0], x[:, 1])
    lat = np.degrees(np.arctan2(x[:, 2], p * (1 - _E2)))
    for _ in range(5):                                      # Bowring-style fixed point
        la = np.radians(lat); n = _A / np.sqrt(1 - _E2 * np.sin(la) ** 2)
        h = p / np.cos(la) - n; lat = np.degrees(np.arctan2(x[:, 2], p * (1 - _E2 * n / (n + h))))
    return np.c_[lat, lon, h]
