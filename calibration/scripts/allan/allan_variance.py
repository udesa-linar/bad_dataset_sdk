#!/usr/bin/env python3
"""Allan deviation of a static IMU CSV (from extract_imu_csv.py) and the noise parameters in the IEEE-952 / Kalibr
convention, per axis and averaged per sensor:

  noise density     N  value at tau = 1 s of the slope -1/2 line fitted to the white-noise region
                       (points with local log-log slope within 0.15 of -1/2, tau <= 10 s)
  random walk       K  value at tau = 3 s of the slope +1/2 line fitted to the rate-random-walk region
                       (points with local slope within 0.15 of +1/2, tau >= 10 s). A 10 h static recording
                       barely reaches that region: when fewer than 5 points qualify the value is flagged unreliable
  bias instability  B  minimum of the Allan deviation / 0.664
Also reported: the Allan deviation read directly at tau = 1 s.

  python3 allan_variance.py ../../output/allan/imu_2100-0700.csv   # -> output/allan/allan_2100-0700/

Outputs (in --out, default allan_<csv name> next to the CSV):
  allan_deviation.csv  tau [s], ADEV accel x y z [m/s^2], gyro x y z [rad/s]  (overlapping estimator)
  allan_deviation.png  log-log plot
  imu_noise.yaml       Kalibr IMU noise model (mean of the three axes)
  allan_results.json   every per-axis value and fit range

Requirements: pip install numpy matplotlib
"""
import argparse, json, os
import numpy as np

NAMES = ['acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z']


def overlapping_adev(y, fs, n_tau=400):
    """overlapping Allan deviation of the columns of y (sampled at fs) for log-spaced cluster sizes"""
    n = y.shape[0]
    cs = np.vstack([np.zeros((1, y.shape[1])), np.cumsum(y, 0)])
    m = np.unique(np.round(np.logspace(0, np.log10(n // 3), n_tau)).astype(int))
    adev = np.empty((m.size, y.shape[1]))
    for i, mi in enumerate(m):
        avg = (cs[mi:] - cs[:-mi]) / mi
        d = avg[mi:] - avg[:-mi]
        adev[i] = np.sqrt(0.5 * np.mean(d * d, 0))
    return m / fs, adev


def noise_params(tau, adev):
    lt, la = np.log10(tau), np.log10(adev)
    s = np.gradient(la, lt)
    w = (np.abs(s + 0.5) < 0.15) & (tau <= 10)
    r = (np.abs(s - 0.5) < 0.15) & (tau >= 10)
    N = float(10 ** np.median(la[w] + 0.5 * lt[w])) if w.any() else float('nan')
    K = float(10 ** np.median(la[r] - 0.5 * lt[r]) * np.sqrt(3)) if r.any() else float('nan')
    i = int(np.argmin(adev))
    return dict(noise_density=N, random_walk=K, random_walk_reliable=bool(r.sum() >= 5),
                bias_instability=float(adev[i] / 0.664), bias_instability_tau_s=float(tau[i]),
                adev_at_1s=float(10 ** np.interp(0.0, lt, la)),
                white_fit=dict(points=int(w.sum()), tau_range=[float(tau[w].min()), float(tau[w].max())] if w.any() else None),
                random_walk_fit=dict(points=int(r.sum()), tau_range=[float(tau[r].min()), float(tau[r].max())] if r.any() else None))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', help='CSV from extract_imu_csv.py (t, gx, gy, gz, ax, ay, az)')
    ap.add_argument('--out', help='output directory (default allan_<csv name>)')
    a = ap.parse_args()
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.csv)), 'allan_' + os.path.splitext(os.path.basename(a.csv))[0].removeprefix('imu_'))
    os.makedirs(out, exist_ok=True)
    d = np.loadtxt(a.csv, delimiter=',', skiprows=1)
    t = d[:, 0]; dts = np.diff(t)
    fs = 1.0 / np.median(dts)
    if (dts > 1.5 / fs).any():
        print(f'WARNING: {int((dts > 1.5 / fs).sum())} gaps longer than 1.5 samples; the estimator assumes uniform sampling')
    y = np.c_[d[:, 4:7], d[:, 1:4]]                                   # accel xyz, gyro xyz
    print(f'{a.csv}: {len(t)} samples, {(t[-1] - t[0]) / 3600:.3f} h at {fs:.3f} Hz')
    tau, adev = overlapping_adev(y, fs)
    np.savetxt(f'{out}/allan_deviation.csv', np.c_[tau, adev], fmt='%.8g', delimiter=',',
               header='tau_s,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z', comments='')
    res = {nm: noise_params(tau, adev[:, j]) for j, nm in enumerate(NAMES)}
    mean = lambda key, ax: float(np.nanmean([res[n][key] for n in NAMES if n.startswith(ax)]))  # noqa: E731
    kal = dict(accelerometer_noise_density=mean('noise_density', 'acc'), accelerometer_random_walk=mean('random_walk', 'acc'),
               gyroscope_noise_density=mean('noise_density', 'gyr'), gyroscope_random_walk=mean('random_walk', 'gyr'))
    rel = {s: all(res[n]['random_walk_reliable'] for n in NAMES if n.startswith(s)) for s in ('acc', 'gyr')}
    with open(f'{out}/imu_noise.yaml', 'w') as f:
        f.write(f'# Xsens MTi-610, Allan deviation of {os.path.basename(a.csv)} ({(t[-1] - t[0]) / 3600:.2f} h), mean of the three axes\n')
        if not all(rel.values()):
            f.write('# random walks: too few points on the +1/2 slope for '
                    + ', '.join({'acc': 'the accelerometer', 'gyr': 'the gyroscope'}[s] for s, v in rel.items() if not v)
                    + ' -> not reliable, see allan_results.json\n')
        for k, v in kal.items():
            f.write(f'{k}: {v:.6e}\n')
        f.write('rostopic: /imu/data\nupdate_rate: 100.0\n')
    json.dump(dict(csv=os.path.abspath(a.csv), samples=int(len(t)), hours=float((t[-1] - t[0]) / 3600), rate_hz=float(fs),
                   per_axis=res, kalibr_mean_of_axes=kal), open(f'{out}/allan_results.json', 'w'), indent=1)
    print(f"{'axis':6s} {'N (tau=1 s line)':>17s} {'ADEV(1 s)':>10s} {'K (tau=3 s line)':>17s} {'B':>10s} {'tau_B [s]':>9s}")
    for nm, v in res.items():
        print(f"{nm:6s} {v['noise_density']:17.3e} {v['adev_at_1s']:10.3e} {v['random_walk']:17.3e}"
              f"{'' if v['random_walk_reliable'] else '*':1s}{v['bias_instability']:9.3e} {v['bias_instability_tau_s']:9.0f}")
    print('* random walk from fewer than 5 points: not reliable\n' + open(f'{out}/imu_noise.yaml').read())
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    for j, nm in enumerate(NAMES):
        ax[j // 3].loglog(tau, adev[:, j], label=nm)
    for axi, unit in zip(ax, ['m/s²', 'rad/s']):
        axi.set_xlabel('tau [s]'); axi.set_ylabel(f'Allan deviation [{unit}]'); axi.grid(True, which='both', alpha=.3); axi.legend()
    fig.suptitle(f'Xsens MTi-610, static, {os.path.basename(a.csv)}')
    fig.savefig(f'{out}/allan_deviation.png', dpi=110)
    print('wrote', out)


if __name__ == '__main__':
    main()
