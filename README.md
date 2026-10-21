# badataset: tools for the Buenos Aires Dataset

Read the Buenos Aires Dataset without ROS, use its calibration, evaluate trajectories against its reference, and
reproduce its calibration from the released calibration recordings.

```bash
pip install -e .                # reading, calibration utilities, evaluation
pip install -e ".[calib]"       # + what the calibration scripts need (rosbags, matplotlib)
```

The dataset layout is `<root>/<sequence>/split_XX/<group>/<group>_0.mcap`, one ROS 2 MCAP bag per sensor group
(`cameras`, `lidar`, `imu_gps`, `dataspeed`, `wheel`, `microphone`, `ground_truth`, `tf_static`), and
`<root>/calib/` for the calibration recordings. Only the groups you downloaded are needed.

## Reading

```python
from badataset import Dataset

ds = Dataset('/data/buenos_aires')
seq = ds.sequence('loop_caba')                         # or ds.sequence('caba', splits=['split_03'])

for t, imu in seq.messages('/imu/data'):               # any topic, every split in order; t = header stamp [s]
    w = imu.angular_velocity
for t, img in seq.images('cam1', start=t0, end=t0 + 5):          # decoded BGR images
    ...
for t, xyz, intensity, ring in seq.clouds(start=t0, end=t0 + 1):  # one LiDAR revolution per item
    ...
```

Every `header.stamp` is GNSS-referenced UTC (see the paper for how each stream is referenced). `start`/`end` filter
on the recording time. MCAP embeds the message definitions, so the u-blox and Dataspeed messages are decoded without
their ROS packages.

## Synchronized front frames

The front pair is triggered when the LiDAR points forward, and both images of a trigger carry the same stamp.

```python
from badataset import sync
for f in sync.front_frames(seq, start=t0, end=t0 + 2):
    f.t, f.cam1, f.cam5, f.cloud.xyz        # cloud: the LiDAR scan that contains the trigger instant
```

## Calibration

```python
from badataset import calib
split = seq.splits[0]
T = calib.tf_tree(split)                                  # {(parent, child): 4x4}
T_lidar_cam1 = calib.transform(T, 'lidar', 'cam1')        # pose of cam1 in the LiDAR frame
K, D, size = calib.intrinsics(split, 'cam1')              # plumb_bob
uv, depth, idx = calib.project(xyz, T_lidar_cam1, K, D, size)
```

See `examples/project_lidar.py`.

## Reference trajectory

```python
from badataset.groundtruth import Reference
ref = Reference(seq)
ref.traj                  # [N, 8] t x y z qx qy qz qw: IMU pose in the local ENU frame, ~14 Hz
ref.origin                # WGS-84 origin of the ENU frame
ref.in_frame('lidar')     # the same trajectory for any frame of /tf_static
ref.save_tum('loop_caba_imu.txt')
```

## Evaluation

```python
from badataset import evaluate as E
est = E.load_tum('mine.txt')                     # IMU pose in the same ENU frame
E.rpe_kitti(ref.traj, est)                       # relative error over 100/200/400/800 m: % and deg/100 m
E.ate_se3(ref.traj, est)                         # absolute error after SE(3) alignment
E.gnss_errors(seq, est, ref.origin)              # error against the RTK receiver's fixes, by fixed / float / none
```

The reference is accurate to a few centimeters at RTK-fixed epochs but only to about a meter where the GNSS solution
is RTK-float; use the RTK-fixed epochs for centimeter-level comparisons (see the paper). Applied to the reference
itself, `gnss_errors` reproduces the table of the paper (SI loops: 0.043 m median on 924 fixed epochs).

## Reproducing the calibration

`calibration/` reruns every calibration of the paper from the released `calib/` recordings and compares the result
with the released calibration. See [calibration/README.md](calibration/README.md).

## Tests

```bash
BADATASET_ROOT=/data/buenos_aires pytest     # uses caba/split_03; skipped if it is not there
```

## License

The code is released under the [Apache License 2.0](LICENSE). The Buenos Aires Dataset itself is released under
CC BY-NC-SA 4.0 (see the dataset card).
