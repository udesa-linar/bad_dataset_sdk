#!/usr/bin/env python3
"""Extract /imu/data of the static IMU recording (./imu, ROS 2 mcap) to a CSV, optionally only a time window.

Columns: t [s, UNIX time of header.stamp], gx gy gz [rad/s], ax ay az [m/s^2]  (Xsens MTi-610, 100 Hz, IMU frame).
The header stamps are already corrected to GNSS/host time, so the window can be given in local wall-clock time.
No ROS installation is needed (the CDR messages are decoded directly).

  python3 extract_imu_csv.py                                   # 21:00-07:00 local (UTC-3) -> output/allan/imu_2100-0700.csv
  python3 extract_imu_csv.py --start 22:00 --end 06:00          # another window (end earlier = next day)
  python3 extract_imu_csv.py --all -o imu_all.csv               # the whole 16 h recording

Requirements: pip install mcap numpy
"""
import argparse, datetime as dt, os, struct, sys
import numpy as np
from mcap.reader import make_reader

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))                       # calibration/
OUTD = os.path.join(ROOT, 'output', 'allan')


def imu_rows(bag_dir):
    """(stamp_ns, gx, gy, gz, ax, ay, az) of every /imu/data message (sensor_msgs/Imu, CDR little endian)."""
    path = os.path.join(bag_dir, [f for f in os.listdir(bag_dir) if f.endswith('.mcap')][0])
    with open(path, 'rb') as f:
        for _, _, m in make_reader(f).iter_messages(topics=['/imu/data']):
            d = m.data
            sec, nsec = struct.unpack_from('<iI', d, 4)               # header.stamp after the 4-byte encapsulation
            n = struct.unpack_from('<I', d, 12)[0]                    # header.frame_id length
            o = 16 + n; o = 4 + (((o - 4) + 7) & ~7)                  # doubles are 8-aligned from the payload start
            v = struct.unpack_from('<28d', d, o)                      # orientation 4, cov 9, gyro 3, cov 9, accel 3
            yield sec * 1_000_000_000 + nsec, v[13], v[14], v[15], v[25], v[26], v[27]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True, help='bag directory: the release calib/allan/imu')
    ap.add_argument('--start', default='21:00', help='window start, local time HH:MM (default 21:00)')
    ap.add_argument('--end', default='07:00', help='window end, local time HH:MM (default 07:00, next day if earlier)')
    ap.add_argument('--utc-offset', type=float, default=-3.0, help='local time zone in hours (Buenos Aires: -3)')
    ap.add_argument('--all', action='store_true', help='export the whole recording, ignore --start/--end')
    ap.add_argument('-o', '--out', help='output CSV (default output/allan/imu_<start>-<end>.csv or imu_all.csv)')
    a = ap.parse_args()
    tz = dt.timezone(dt.timedelta(hours=a.utc_offset))
    rows = np.array(list(imu_rows(a.bag)), dtype=np.float64)
    t = rows[:, 0].astype(np.int64)
    t0 = dt.datetime.fromtimestamp(t[0] / 1e9, tz)
    if a.all:
        k = np.ones(t.size, bool); out = a.out or os.path.join(OUTD, 'imu_all.csv')
    else:
        h0, h1 = [dt.time(*map(int, s.split(':'))) for s in (a.start, a.end)]
        w0 = dt.datetime.combine(t0.date(), h0, tz)                  # first occurrence of --start in the recording
        if w0 < t0 - dt.timedelta(minutes=1): w0 += dt.timedelta(days=1)
        w1 =dt.datetime.combine(w0.date() + dt.timedelta(days=1 if h1 <= h0 else 0), h1, tz)
        k = (t >= w0.timestamp() * 1e9) & (t < w1.timestamp() * 1e9)
        out = a.out or os.path.join(OUTD, f'imu_{a.start.replace(":", "")}-{a.end.replace(":", "")}.csv')
        print(f'window {w0:%Y-%m-%d %H:%M} -> {w1:%Y-%m-%d %H:%M} (UTC{a.utc_offset:+g})')
    if not k.any():
        sys.exit('no samples in the window; the recording covers '
                 f'{t0:%Y-%m-%d %H:%M} -> {dt.datetime.fromtimestamp(t[-1] / 1e9, tz):%Y-%m-%d %H:%M}')
    sel = rows[k]
    dtm = np.diff(t[k]) / 1e6
    print(f'{k.sum()} samples, {(t[k][-1] - t[k][0]) / 3.6e12:.3f} h, median dt {np.median(dtm):.3f} ms, '
          f'max dt {dtm.max():.1f} ms, gaps > 15 ms: {(dtm > 15).sum()}')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, 'w') as f:
        f.write('t,gx,gy,gz,ax,ay,az\n')
        np.savetxt(f, np.c_[t[k] / 1e9, sel[:, 1:]], fmt=['%.6f'] + ['%.9e'] * 6, delimiter=',')
    print('wrote', out)


if __name__ == '__main__':
    main()
