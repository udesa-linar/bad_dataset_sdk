"""Export topics of a released calibration recording (calib/<name>/<group>/, ROS 2 mcap) for the
calibration tools. Message contents and header stamps are unchanged; the bag time of each message is
set to its header stamp (the corrected GNSS/host time), which is what Kalibr pairs and orders by.

  kalibr  ROS 1 bag (Kalibr runs in ROS 1 Noetic). Images can be decimated to --rate Hz; cam1/cam5
          share their trigger stamps, so the same triggers are kept for both. Other topics are copied whole.
  koide   ROS 2 sqlite3 bags readable by ROS 2 Humble (direct_visual_lidar_calibration), one bag per
          window: the tool takes the first image of a bag and accumulates every point cloud of it.

Usage: export_bags.py kalibr <calib_dir> <out.bag> --topics T [T ...] [--rate HZ]
       export_bags.py koide  <calib_dir> <out_dir> --images T [T ...] [--points T] --windows N --window-s S
"""
import argparse, glob, os, shutil
import numpy as np
from rosbags.rosbag2 import Reader as Reader2, Writer as Writer2
from rosbags.rosbag1 import Writer as Writer1
from rosbags.typesys import Stores, get_typestore

TS2 = get_typestore(Stores.ROS2_JAZZY); TS1 = get_typestore(Stores.ROS1_NOETIC); TSH = get_typestore(Stores.ROS2_HUMBLE)


def to_ros1(msg, mt):
    """ROS 2 message -> the ROS 1 type of the same name (adds header.seq = 0, copies every other field)"""
    import dataclasses
    cls = TS1.types[mt]; kw = {}
    for f in dataclasses.fields(cls):
        if f.name == 'seq' and mt == 'std_msgs/msg/Header':
            kw['seq'] = 0; continue
        v = getattr(msg, f.name)
        if dataclasses.is_dataclass(v) and not isinstance(v, type):
            v = to_ros1(v, v.__msgtype__)
        kw[f.name] = v
    return cls(**kw)


def raw_stamp_ns(raw):
    """header.stamp straight from the CDR bytes (4-byte encapsulation, then int32 sec, uint32 nanosec)"""
    sec, nsec = np.frombuffer(raw[4:12], dtype='<i4')[0], np.frombuffer(raw[8:12], dtype='<u4')[0]
    return int(sec) * 1_000_000_000 + int(nsec)


def groups_with(calib_dir, topics):
    out = []
    metas = [f'{calib_dir}/metadata.yaml'] if os.path.exists(f'{calib_dir}/metadata.yaml') else sorted(glob.glob(f'{calib_dir}/*/metadata.yaml'))
    for meta in metas:
        d = os.path.dirname(meta)
        with Reader2(d) as r:
            if any(c.topic in topics for c in r.connections):
                out.append(d)
    return out


def iter_raw(calib_dir, topics, windows=None):
    """(topic, msgtype, stamp_ns, raw cdr) of the given topics, group after group, in bag order. With windows =
    [(start_ns, end_ns), ...] (bag time) only those intervals are read, through the index of the bag (fast seek)."""
    for d in groups_with(calib_dir, topics):
        with Reader2(d) as r:
            con = [c for c in r.connections if c.topic in topics]
            for start, stop in (windows or [(None, None)]):
                for c, t, raw in r.messages(connections=con, start=start, stop=stop):
                    yield c.topic, c.msgtype, raw_stamp_ns(raw), raw


def kalibr(a):
    images = [t for t in a.topics if 'image' in t]
    if os.path.exists(a.out): os.remove(a.out)
    kept, last = {}, {}
    with Writer1(a.out) as w:
        conns = {}
        for topic, mt, st, raw in iter_raw(a.calib_dir, set(a.topics)):
            if topic in images and a.rate:
                slot = int(st * a.rate // 1_000_000_000)
                if last.get(topic) == slot: continue
                last[topic] = slot
            if topic not in conns: conns[topic] = w.add_connection(topic, mt, typestore=TS1)
            w.write(conns[topic], st, TS1.serialize_ros1(to_ros1(TS2.deserialize_cdr(raw, mt), mt), mt))
            kept[topic] = kept.get(topic, 0) + 1
    print('wrote', a.out, kept)


def koide(a):
    """one pass: windows of --window-s seconds spread over the recording; per camera and window a bag with
    the first image (+ its camera_info) of the window and every LiDAR cloud of the window"""
    import yaml
    info = yaml.safe_load(open(f'{a.calib_dir}/lidar/metadata.yaml'))['rosbag2_bagfile_information']
    t0 = info['starting_time']['nanoseconds_since_epoch']; t1 = t0 + info['duration']['nanoseconds']
    lo = t0 + (a.from_s if a.from_s is not None else 10.0) * 1e9            # seconds from the bag start
    hi = (t0 + a.to_s * 1e9) if a.to_s is not None else t1 - 10e9
    if a.ranges:                                                   # explicit windows: s0-s1,s2-s3,... (seconds from start)
        rr = sorted(tuple(map(float, x.split('-'))) for x in a.ranges.split(','))   # windows must be in time order
        starts = np.array([t0 + int(x0 * 1e9) for x0, _ in rr], dtype=np.int64)
        ends = np.array([t0 + int(x1 * 1e9) for _, x1 in rr], dtype=np.int64)
    else:
        starts = np.linspace(lo, hi - a.window_s * 1e9, a.windows).astype(np.int64)
        ends = starts + int(a.window_s * 1e9)
    cams = {img: img.replace('image_raw/compressed', 'camera_info') for img in a.images}
    if os.path.exists(a.out): shutil.rmtree(a.out)
    W, C, got = {}, {}, set()
    def writer(cam, i):
        k = (cam, i)
        if k not in W:
            d = f"{a.out}/{cam.split('/')[1]}/window_{i:02d}"; os.makedirs(os.path.dirname(d), exist_ok=True)
            W[k] = Writer2(d, version=8); W[k].open(); C[k] = {}
        return W[k], C[k]
    topics = set(cams) | set(cams.values()) | {a.points}
    n = {}
    try:
        for topic, mt, st, raw in iter_raw(a.calib_dir, topics, [(int(x) - 2_000_000_000, int(y) + 2_000_000_000) for x, y in zip(starts, ends)]):
            i = int(np.searchsorted(starts, st, side='right')) - 1
            if i < 0 or st > ends[i]: continue
            if topic == a.points: targets = list(cams)
            elif topic in cams:
                if (topic, i) in got: continue                        # the tool reads one image per bag
                got.add((topic, i)); targets = [topic]
            else:
                img = [k for k, v in cams.items() if v == topic][0]
                if (img + '#info', i) in got: continue
                got.add((img + '#info', i)); targets = [img]
            msg = TS2.deserialize_cdr(raw, mt)
            for cam in targets:
                w, c = writer(cam, i)
                if topic not in c: c[topic] = w.add_connection(topic, mt, typestore=TSH)
                w.write(c[topic], st, TSH.serialize_cdr(msg, mt)); n[(cam, topic)] = n.get((cam, topic), 0) + 1
    finally:
        for w in W.values(): w.close()
    for k, v in sorted(n.items()): print(k, v)
    print('wrote', a.out, len(W), 'bags')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    k = sub.add_parser('kalibr'); k.add_argument('calib_dir'); k.add_argument('out'); k.add_argument('--topics', nargs='+', required=True)
    k.add_argument('--rate', type=float, default=0.0)
    o = sub.add_parser('koide'); o.add_argument('calib_dir'); o.add_argument('out')
    o.add_argument('--images', nargs='+', required=True); o.add_argument('--points', default='/velodyne_points')
    o.add_argument('--windows', type=int, default=6); o.add_argument('--window-s', type=float, default=3.0)
    o.add_argument('--from-s', type=float); o.add_argument('--to-s', type=float)   # restrict the windows to [from, to] s
    o.add_argument('--ranges', help='explicit windows, e.g. 20-30,60-70 (seconds from the bag start)')
    a = ap.parse_args()
    {'kalibr': kalibr, 'koide': koide}[a.cmd](a)
