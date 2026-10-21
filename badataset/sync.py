"""Synchronized access across sensors.

The front pair (cam1, cam5) is triggered when the LiDAR points forward, and both images of a trigger carry the same
stamp (the GNSS time of the trigger). A LiDAR scan is stamped at the end of its revolution, so the scan that contains
the trigger instant is the first one stamped after the images, within one revolution (~0.1 s).

    from badataset import sync
    for f in sync.front_frames(seq, start=t0, end=t0 + 5):
        f.t, f.cam1, f.cam5, f.cloud.xyz, f.cloud.t
"""
from dataclasses import dataclass

import numpy as np

REVOLUTION = 0.1          # s, LiDAR at 10 Hz


@dataclass
class Cloud:
    t: float
    xyz: np.ndarray
    intensity: np.ndarray
    ring: np.ndarray


@dataclass
class FrontFrame:
    t: float              # trigger time (stamp of both images)
    cam1: np.ndarray      # BGR
    cam5: np.ndarray      # BGR, or None if cam5 has no image of this trigger
    cloud: Cloud          # scan containing the trigger instant, or None


def front_frames(sequence, start=None, end=None, with_cloud=True):
    """yield FrontFrame for every cam1 image in [start, end] (recording time), with the cam5 image of the same
    trigger and the LiDAR scan that contains it"""
    pad = 2 * REVOLUTION
    right = {round(t, 4): im for t, im in sequence.images('cam5', start, None if end is None else end + pad)}
    clouds = []
    if with_cloud:
        clouds = [Cloud(t, x, i, r) for t, x, i, r in sequence.clouds(start, None if end is None else end + pad)]
    ct = np.array([c.t for c in clouds])
    for t, im in sequence.images('cam1', start, end):
        c = None
        if len(ct):
            k = np.searchsorted(ct, t)
            if k < len(ct) and ct[k] - t <= REVOLUTION * 1.2:
                c = clouds[k]
        yield FrontFrame(t, im, right.get(round(t, 4)), c)


def nearest(times, t):
    """index into the sorted array `times` of the element nearest to each t, and its distance [s]"""
    times = np.asarray(times); t = np.atleast_1d(t)
    k = np.clip(np.searchsorted(times, t), 1, len(times) - 1)
    k = np.where(np.abs(times[k] - t) < np.abs(times[k - 1] - t), k, k - 1)
    return k, np.abs(times[k] - t)
