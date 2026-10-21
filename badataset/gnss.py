"""GNSS fixes of the two u-blox receivers with their carrier-phase status.

The receiver with the absolute RTK solution is ``/ublox_moving_base`` (trunk lid, frame ``gps_back``) in the March
sequences and ``/ublox_rover`` (hood, ``gps_front``) in ``sanfer_sanisidro``, whose receivers were connected to the
opposite antennas (see the paper). Each ``fix`` is matched to the ``NAV-PVT`` message of the same receiver received
closest in time, which carries the carrier-phase solution (``flags`` bits 6-7: 0 none, 1 float, 2 fixed) and the
receiver's horizontal accuracy estimate.

    from badataset import gnss
    fx = gnss.fixes(seq)                     # receiver chosen per sequence
    fx['t'], fx['lat'], fx['carr'], fx['hacc'], fx['frame']
"""
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

RTK_RECEIVER = {'sanfer_sanisidro': '/ublox_rover'}
CARR = {0: 'none', 1: 'float', 2: 'fixed'}


def rtk_receiver(sequence_name):
    return RTK_RECEIVER.get(sequence_name, '/ublox_moving_base')


def fixes(sequence, receiver=None):
    """dict of arrays: t [s, header stamp = GNSS epoch], lat, lon, alt (ellipsoidal) [deg, deg, m], carr (0/1/2),
    hacc [m, receiver estimate]; and frame (the antenna frame of the messages)"""
    rx = receiver or rtk_receiver(sequence.name)
    fx, pv, frame = [], [], None
    for split in sequence.splits:
        with open(split.mcap('imu_gps'), 'rb') as f:
            r = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, ch, msg, m in r.iter_decoded_messages(topics=[rx + '/fix', rx + '/navpvt']):
                if ch.topic.endswith('/fix'):
                    fx.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.latitude, m.longitude, m.altitude, msg.log_time * 1e-9))
                    frame = m.header.frame_id
                else:
                    pv.append((msg.log_time * 1e-9, (m.flags >> 6) & 3, m.h_acc * 1e-3))
    if not fx:
        raise ValueError(f'{sequence.name}: no {rx}/fix messages')
    fx = np.array(fx); pv = np.array(pv); fx = fx[np.argsort(fx[:, 0])]; pv = pv[np.argsort(pv[:, 0])]
    j = np.clip(np.searchsorted(pv[:, 0], fx[:, 4]), 1, len(pv) - 1)
    j = np.where(np.abs(pv[j, 0] - fx[:, 4]) < np.abs(pv[j - 1, 0] - fx[:, 4]), j, j - 1)
    return dict(t=fx[:, 0], lat=fx[:, 1], lon=fx[:, 2], alt=fx[:, 3], carr=pv[j, 1].astype(int), hacc=pv[j, 2],
                frame=frame, receiver=rx)
