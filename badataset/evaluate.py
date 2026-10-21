"""Trajectory evaluation against the reference trajectory, as in the paper.

Trajectories are arrays [N, 8]: t [s], x, y, z [m], qx, qy, qz, qw (pose of the IMU in the reference
ENU frame), i.e. the TUM format. Use load_tum()/save_tum() to read and write them.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp


def load_tum(path):
    a = np.loadtxt(path, comments='#', usecols=range(8))
    _, i = np.unique(a[:, 0], return_index=True)
    return a[i]


def save_tum(path, traj):
    np.savetxt(path, traj, fmt='%.6f', header='timestamp tx ty tz qx qy qz qw')


def interp(traj, t):
    """pose of `traj` at times t (only those inside its span): (t, p [M,3], Rotation, mask)"""
    ok = (t >= traj[0, 0]) & (t <= traj[-1, 0]); t = t[ok]
    p = np.c_[[np.interp(t, traj[:, 0], traj[:, k]) for k in (1, 2, 3)]].T
    return t, p, Slerp(traj[:, 0], Rot.from_quat(traj[:, 4:8]))(t), ok


def rpe_kitti(ref, est, lengths=(100, 200, 400, 800), step_m=10.0):
    """KITTI-style relative error over path lengths: {L: {n, trans_pct, rot_deg_per_100m}}"""
    _, pe, Re, ok = interp(est, ref[:, 0])
    pr = ref[ok, 1:4]; Rr = Rot.from_quat(ref[ok, 4:8])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pr, axis=0), axis=1))]
    out = {}
    for L in lengths:
        te, re = [], []
        for i in np.searchsorted(s, np.arange(0, s[-1] - L, step_m)):
            j = np.searchsorted(s, s[i] + L)
            if j >= len(s): break
            dr = Rr[i].inv() * Rr[j]; dpr = Rr[i].inv().apply(pr[j] - pr[i])
            de = Re[i].inv() * Re[j]; dpe = Re[i].inv().apply(pe[j] - pe[i])
            te.append(np.linalg.norm(dr.inv().apply(dpe - dpr)) / L * 100)
            re.append(np.degrees((dr.inv() * de).magnitude()) / L * 100)
        out[L] = dict(n=len(te), trans_pct=float(np.mean(te)) if te else np.nan,
                      rot_deg_per_100m=float(np.mean(re)) if re else np.nan)
    return out


def ate_se3(ref, est):
    """absolute trajectory error after SE(3) (Umeyama, no scale) alignment of est to ref"""
    _, pe, _, ok = interp(est, ref[:, 0]); pr = ref[ok, 1:4]
    mr, me = pr.mean(0), pe.mean(0); U, _, Vt = np.linalg.svd((pe - me).T @ (pr - mr))
    D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T)); R = Vt.T @ D @ U.T
    e = np.linalg.norm(pr - ((pe - me) @ R.T + mr), axis=1)
    return dict(rmse_m=float(np.sqrt(np.mean(e ** 2))), median_m=float(np.median(e)), max_m=float(e.max()))


def outage_errors(ref, est, windows, step=1.0):
    """horizontal error of `est` w.r.t. `ref` along each GNSS outage window (t0, t1):
    returns the error at the end of each outage and the error curves (1 s steps)"""
    ends, curves = [], []
    for a, b in windows:
        tt = np.arange(a, b + 1e-6, step)
        t1, p1, _, _ = interp(ref, tt); t2, p2, _, _ = interp(est, tt)
        if len(t1) != len(tt) or len(t2) != len(tt): continue
        e = np.linalg.norm((p2 - p1)[:, :2], axis=1); ends.append(e[-1]); curves.append(e)
    return np.array(ends), np.array(curves)


def gnss_errors(sequence, traj, origin, frame='imu', receiver=None, tf=None):
    """Error of a trajectory with respect to the GNSS fixes of the RTK receiver, by carrier-phase status (protocol
    (iii) of the paper: for methods that use GNSS, evaluate on RTK-fixed epochs they did not use).

    traj: [N, 8] TUM array of `frame` in the local ENU frame whose WGS-84 origin is `origin` (lat, lon, alt), e.g.
    Reference.origin. The pose is moved to the antenna of the receiver with /tf_static.
    Returns {'fixed'|'float'|'none'|'all': {n, h_median, h_rmse, h_p95, v_median_abs}} in meters, and the
    per-epoch arrays under 'epochs'.
    """
    from . import calib, gnss
    from .groundtruth import wgs84_to_enu
    fx = gnss.fixes(sequence, receiver)
    tf = tf if tf is not None else sequence.tf_static()
    lever = calib.transform(tf, frame, fx['frame'])[:3, 3]
    t, p, R, ok = interp(traj, fx['t'])
    ant = p + R.apply(lever)
    g = wgs84_to_enu(fx['lat'][ok], fx['lon'][ok], fx['alt'][ok], *origin)
    e = ant - g; h = np.linalg.norm(e[:, :2], axis=1); carr = fx['carr'][ok]
    out = {'epochs': dict(t=t, err=e, carr=carr, hacc=fx['hacc'][ok])}
    for name, m in [('all', np.ones_like(carr, bool)), ('fixed', carr == 2), ('float', carr == 1), ('none', carr == 0)]:
        if m.sum():
            out[name] = dict(n=int(m.sum()), h_median=float(np.median(h[m])), h_rmse=float(np.sqrt(np.mean(h[m] ** 2))),
                             h_p95=float(np.percentile(h[m], 95)), v_median_abs=float(np.median(np.abs(e[m, 2]))))
    return out
