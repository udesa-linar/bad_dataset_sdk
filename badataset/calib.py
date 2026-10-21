"""Calibration: the released /tf_static tree and camera_info intrinsics, and LiDAR-to-camera projection."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def tf_tree(split):
    """{(parent, child): 4x4 pose of child in parent} from a split's tf_static bag"""
    T = {}
    for _, m in split.messages('/tf_static'):
        for t in m.transforms:
            M = np.eye(4); q = t.transform.rotation; p = t.transform.translation
            M[:3, :3] = Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix(); M[:3, 3] = [p.x, p.y, p.z]
            T[(t.header.frame_id, t.child_frame_id)] = M
    return T


def transform(T, a, b):
    """4x4 pose of frame b expressed in frame a (maps points from b to a), through the tree"""
    parent = {c: p for (p, c) in T}

    def to_root(x):
        M = np.eye(4); path = [x]
        while x in parent:
            M = T[(parent[x], x)] @ M; x = parent[x]; path.append(x)
        return M, path

    Ma, pa = to_root(a); Mb, pb = to_root(b)
    if pa[-1] != pb[-1]:
        raise ValueError(f'{a} and {b} are not connected')
    return np.linalg.inv(Ma) @ Mb


def intrinsics(split, cam):
    """(K 3x3, D [k1 k2 p1 p2 k3], (width, height)) of camN from its camera_info (plumb_bob)"""
    for _, m in split.messages(f'/{cam}/camera_info'):
        return np.array(m.k).reshape(3, 3), np.array(m.d), (m.width, m.height)


def project(points_lidar, T_lidar_cam, K, D, size, min_depth=0.5):
    """project LiDAR points (N,3) into a camera; returns (uv [M,2], depth [M], index into the input [M])"""
    M = np.linalg.inv(T_lidar_cam)
    pc = points_lidar @ M[:3, :3].T + M[:3, 3]
    idx = np.where(pc[:, 2] > min_depth)[0]
    uv, _ = cv2.projectPoints(pc[idx], np.zeros(3), np.zeros(3), K, D)
    uv = uv[:, 0]; w, h = size
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    return uv[ok], np.linalg.norm(pc[idx[ok]], axis=1), idx[ok]
