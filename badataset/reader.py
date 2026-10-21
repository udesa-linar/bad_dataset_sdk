"""Reading the Buenos Aires Dataset without ROS.

Layout:  <root>/<sequence>/split_XX/<group>/<group>_0.mcap   (group: cameras, lidar, imu_gps, dataspeed,
wheel, microphone, ground_truth, tf_static). MCAP embeds the message definitions, so every topic
(including ublox_msgs and dbw_mkz_msgs) is decoded by mcap-ros2-support without the ROS packages.

    from badataset import Dataset
    ds = Dataset('/data/buenos_aires')
    seq = ds.sequence('loop_sanisidro')
    for t, msg in seq.messages('/imu/data'):            # all splits, in time order
        ...
    for t, img in seq.images('cam1', start=t0, end=t0 + 10):   # decoded BGR images
        ...
    for t, xyz, intensity, ring in seq.clouds(start=t0, end=t0 + 1):
        ...
Times are seconds (float) of the message header stamp (GNSS-referenced UTC, see the paper).
"""
import glob
import inspect
import json
import os
from dataclasses import dataclass

import cv2
import numpy as np
from mcap.reader import McapReader, make_reader
from mcap_ros2.decoder import DecoderFactory

GROUP_OF = {
    'cam': 'cameras', '/velodyne_points': 'lidar', '/imu/data': 'imu_gps', '/ublox': 'imu_gps',
    '/dataspeed': 'dataspeed', '/wheel': 'wheel', '/microphone': 'microphone', '/tf_static': 'tf_static',
    '/diagnostics': 'diagnostics', '/rosout': 'diagnostics', '/ground_truth': 'ground_truth',
}
SEQUENCES = ['sanfer_sanisidro', 'loop_sanisidro', 'caba', 'loop_caba']


def group_of(topic):
    for k, g in GROUP_OF.items():
        if topic.startswith(k) or topic.startswith('/' + k):
            return g
    raise KeyError(f'unknown topic {topic}')


_NEW_API = 'ending_at' in inspect.signature(McapReader.iter_decoded_messages).parameters


def _window(start, end):
    """reader keyword arguments for the recording-time interval [start, end] (seconds, both inclusive)"""
    kw = {}
    if start is not None:
        kw['starting_at' if _NEW_API else 'start_time'] = int(start * 1e9)
    if end is not None:
        kw['ending_at' if _NEW_API else 'end_time'] = int(end * 1e9) + (0 if _NEW_API else 1)
    return kw


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


@dataclass
class Split:
    path: str

    @property
    def name(self):
        return os.path.basename(self.path)

    def mcap(self, group):
        f = glob.glob(os.path.join(self.path, group, '*.mcap'))
        if not f:
            raise FileNotFoundError(f'{group} not downloaded for {self.path}')
        return f[0]

    def messages(self, topic, start=None, end=None):
        """yield (header time [s], decoded message); start/end filter on the recording (log) time"""
        kw = _window(start, end)
        with open(self.mcap(group_of(topic)), 'rb') as f:
            r = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, _, _, m in r.iter_decoded_messages(topics=[topic], **kw):
                yield (stamp(m) if hasattr(m, 'header') else None), m

    def messages_logtime(self, topic, start=None, end=None):
        """yield (recording time [s], decoded message): for messages without a header (e.g. the audio)"""
        kw = _window(start, end)
        with open(self.mcap(group_of(topic)), 'rb') as f:
            r = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, _, msg, m in r.iter_decoded_messages(topics=[topic], **kw):
                yield msg.log_time * 1e-9, m


class Sequence:
    def __init__(self, root, name, splits=None):
        self.name = name
        self.splits = [Split(p) for p in sorted(glob.glob(os.path.join(root, name, 'split_*')))
                       if splits is None or os.path.basename(p) in splits]
        if not self.splits:
            raise FileNotFoundError(f'no splits for {name} under {root}')

    def messages(self, topic, start=None, end=None):
        for s in self.splits:
            yield from s.messages(topic, start, end)

    def images(self, cam, start=None, end=None):
        """cam in cam1..cam5; yields (time, BGR image)"""
        for t, m in self.messages(f'/{cam}/image_raw/compressed', start, end):
            yield t, cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)

    def clouds(self, start=None, end=None):
        """yields (time, xyz [N,3] float32, intensity [N], ring [N]) of each LiDAR revolution"""
        for t, m in self.messages('/velodyne_points', start, end):
            off = {f.name: f.offset for f in m.fields}
            dt = np.dtype({'names': ['x', 'y', 'z', 'i', 'r'], 'formats': ['<f4', '<f4', '<f4', '<f4', '<u2'],
                           'offsets': [off['x'], off['y'], off['z'], off['intensity'], off['ring']], 'itemsize': m.point_step})
            a = np.frombuffer(bytes(m.data), dt)
            yield t, np.c_[a['x'], a['y'], a['z']], a['i'], a['r']

    def audio(self, start=None, end=None):
        """(t0 [s], sample rate [Hz], int16 samples [N]) of the microphone between start and end.

        Each /microphone/audio/raw message holds 1024 mono pcm_s16le samples; its recording time is the time of its
        first sample, aligned to the cameras by hand (see the paper, which also lists the sequences where the audio is
        silent, clipped or not aligned). Gaps between messages are not filled: check t0 against the sample count."""
        rate = None
        for s in self.splits:
            for _, m in s.messages_logtime('/microphone/audio/info'):
                rate = json.loads(m.data)['sample_rate']; break
            if rate: break
        t0, chunks = None, []
        for s in self.splits:
            for t, m in s.messages_logtime('/microphone/audio/raw', start, end):
                if t0 is None: t0 = t
                chunks.append(np.frombuffer(bytes(m.data), '<i2'))
        if t0 is None:
            raise ValueError('no audio in the requested interval')
        return t0, rate, np.concatenate(chunks)

    def tf_static(self):
        """the released transform tree of the first split, as {(parent, child): 4x4}"""
        from .calib import tf_tree
        return tf_tree(self.splits[0])


class Dataset:
    def __init__(self, root):
        self.root = root

    def sequences(self):
        return [s for s in SEQUENCES if os.path.isdir(os.path.join(self.root, s))]

    def sequence(self, name, splits=None):
        """splits: optional list of split names (e.g. ['split_03']) to use only those"""
        return Sequence(self.root, name, splits)
