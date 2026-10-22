# badataset: tools for the Buenos Aires Dataset

Read the Buenos Aires Dataset without ROS, use its calibration, evaluate trajectories against its reference, and
reproduce its calibration from the released calibration recordings.

```bash
pip install -e .                # reading, calibration utilities, evaluation
pip install -e ".[calib]"       # + what the calibration scripts need (rosbags, matplotlib)
pip install -e ".[download]"    # + the download helper (huggingface_hub)
```

## Downloading

The data are hosted on Hugging Face ([udesa-linar/buenos-aires-dataset](https://huggingface.co/datasets/udesa-linar/buenos-aires-dataset)).
Download everything, or only the sequences, sensor groups and splits you need:

```bash
badataset-download /data/buenos_aires                                    # the whole release (2.9 TB)
badataset-download /data/buenos_aires --sequences loop_caba --groups imu_gps ground_truth tf_static
badataset-download /data/buenos_aires --sequences caba --splits split_03  # one split, every group
badataset-download /data/buenos_aires --calib                            # only the calibration recordings
```

or `badataset.download.download(root, sequences=..., groups=..., splits=..., calib=...)` from Python. An interrupted
download resumes; `--revision` pins a version of the release.

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
BADATASET_ROOT=/data/buenos_aires pytest     # the release tests use caba/split_03 and are skipped if it is not there
```

## Citation

If you use the dataset or this toolkit, please cite:

```bibtex
@misc{roca2026buenosaires,
  title   = {The Buenos Aires Dataset: Multimodal Driving Data from a Latin American Metropolis
             with GNSS-Referenced Time Synchronization},
  author  = {Roca, Agust{\'i}n and Castro, Gast{\'o}n and Rosito, Claus and Torre, Gabriel and
             Mas, Ignacio and Giribet, Juan I.},
  howpublished = {\url{https://udesa-linar.github.io/buenos-aires-dataset/}},
  year    = {2026}
}
```

## License

The code is released under the [Apache License 2.0](LICENSE). The Buenos Aires Dataset itself is released under
CC BY-NC-SA 4.0 (see the dataset card).
