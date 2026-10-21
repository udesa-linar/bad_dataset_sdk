"""Tests against a released split. Set BADATASET_ROOT to the dataset folder (<root>/<sequence>/split_XX/...);
they use caba/split_03 (1.8 min, every sensor group) and are skipped if it is not there."""
import os

import numpy as np
import pytest

from badataset import Dataset, calib, evaluate, groundtruth, gnss, sync

ROOT = os.environ.get('BADATASET_ROOT', '')
SEQ, SPLIT = 'caba', 'split_03'
pytestmark = pytest.mark.skipif(not os.path.isdir(os.path.join(ROOT, SEQ, SPLIT)), reason='test split not available')


@pytest.fixture(scope='module')
def seq():
    return Dataset(ROOT).sequence(SEQ, splits=[SPLIT])


def test_tf_tree(seq):
    T = seq.tf_static()
    frames = {c for _, c in T} | {p for p, _ in T}
    for f in ['lidar', 'cam1', 'cam2', 'cam3', 'cam4', 'cam5', 'imu', 'gps_front', 'gps_back', 'base_link', 'microphone']:
        assert f in frames
    M = calib.transform(T, 'lidar', 'cam1')
    assert np.allclose(M[:3, :3] @ M[:3, :3].T, np.eye(3), atol=1e-9)
    assert np.allclose(calib.transform(T, 'cam1', 'lidar'), np.linalg.inv(M))
    b = np.linalg.norm(calib.transform(T, 'cam1', 'cam5')[:3, 3])
    assert 0.53 < b < 0.55                                 # stereo baseline


def test_intrinsics_and_images(seq):
    K, D, size = calib.intrinsics(seq.splits[0], 'cam1')
    assert K.shape == (3, 3) and size == (1224, 1026)
    t, im = next(seq.images('cam1'))
    assert im.shape == (1026, 1224, 3)
    t4, im4 = next(seq.images('cam4'))
    assert im4.shape == (770, 1024, 3)


def test_cloud_and_projection(seq):
    t, xyz, inten, ring = next(seq.clouds())
    assert xyz.shape[1] == 3 and len(xyz) > 100000 and ring.max() <= 63
    T = seq.tf_static(); K, D, size = calib.intrinsics(seq.splits[0], 'cam1')
    uv, depth, idx = calib.project(xyz, calib.transform(T, 'lidar', 'cam1'), K, D, size)
    assert len(uv) > 1000 and (uv[:, 0] < size[0]).all() and (depth > 0).all()


def test_reference(seq):
    ref = groundtruth.Reference(seq)
    assert ref.traj.shape[1] == 8 and len(ref.traj) > 1000 and np.all(np.diff(ref.t) > 0)
    assert np.allclose(np.linalg.norm(ref.traj[:, 4:8], axis=1), 1, atol=1e-6)
    lid = ref.in_frame('lidar')
    lever = np.linalg.norm(calib.transform(seq.tf_static(), 'imu', 'lidar')[:3, 3])
    assert np.allclose(np.linalg.norm(lid[:, 1:4] - ref.traj[:, 1:4], axis=1), lever, atol=1e-6)
    t, p, R, ok = ref.at([ref.t[10], ref.t[10] + 0.01])
    assert ok.all() and np.allclose(p[0], ref.traj[10, 1:4])


def test_wgs84_roundtrip():
    lat0, lon0, h0 = -34.44528, -58.53061, 20.6
    enu = np.array([[0, 0, 0], [1000.0, -2000.0, 15.0]])
    back = groundtruth.wgs84_to_enu(*groundtruth.enu_to_wgs84(enu, lat0, lon0, h0).T, lat0, lon0, h0)
    assert np.allclose(back, enu, atol=1e-4)


def test_gnss_and_reference_agreement(seq):
    fx = gnss.fixes(seq)
    assert set(np.unique(fx['carr'])) <= {0, 1, 2} and fx['frame'] == 'gps_back'
    ref = groundtruth.Reference(seq)
    r = evaluate.gnss_errors(seq, ref.traj, ref.origin)
    assert r['all']['n'] > 100 and r['all']['h_median'] < 2.0   # reference vs fixes of the RTK receiver


def test_front_frames(seq):
    n = 0
    for f in sync.front_frames(seq):
        assert f.cam1.shape == (1026, 1224, 3)
        if f.cloud is not None:
            assert 0 <= f.cloud.t - f.t <= 0.12
        n += 1
        if n == 3:
            break
    assert n == 3


def test_every_topic_decodes(seq):
    """the first message of every topic of every downloaded group decodes (embedded definitions, no ROS packages)"""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    split = seq.splits[0]; checked = 0
    for group in sorted(os.listdir(split.path)):
        with open(split.mcap(group), 'rb') as f:
            r = make_reader(f, decoder_factories=[DecoderFactory()])
            topics = {c.topic for c in r.get_summary().channels.values()}; seen = set()
            for _, ch, _, m in r.iter_decoded_messages():
                seen.add(ch.topic)
                if seen == topics:
                    break
            assert seen == topics, (group, topics - seen)
            checked += len(topics)
    assert checked >= 40


def test_audio(seq):
    t0, rate, x = seq.audio(end=None)
    assert rate == 44100 and x.dtype == np.int16 and len(x) > rate * 60   # split_03 lasts 1.8 min
    t1, _, y = seq.audio(start=t0 + 10, end=t0 + 11)
    assert 10 <= t1 - t0 <= 10.1 and abs(len(y) - rate) <= 2048


SFSI = os.path.join(ROOT, 'sanfer_sanisidro', 'split_01')


@pytest.mark.skipif(not os.path.isdir(SFSI), reason='sanfer_sanisidro not available')
def test_sfsi_rtk_receiver():
    """in sanfer_sanisidro the RTK solution is on /ublox_rover (hood, gps_front): the receivers were swapped"""
    s = Dataset(ROOT).sequence('sanfer_sanisidro', splits=['split_01'])
    fx = gnss.fixes(s)
    assert fx['receiver'] == '/ublox_rover' and fx['frame'] == 'gps_front'
    ref = groundtruth.Reference(s)
    r = evaluate.gnss_errors(s, ref.traj, ref.origin)
    assert r['fixed']['n'] > 100 and r['fixed']['h_median'] < 0.2
