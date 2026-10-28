# MINS configuration of the reference trajectory

The reference trajectory of the dataset (`ground_truth/` bags) was computed with MINS (multisensor-aided inertial
navigation, Lee et al. 2023), fusing the MTi-610, the front stereo pair, the LiDAR, the rear wheel speeds and the
fixes of both GNSS receivers, with every calibration fixed at its released value and every time offset zero. The same
configuration, `rtk_driving_noise/`, is used for the four sequences:

| Setting | Value |
|---|---|
| IMU white noise | Allan-variance model of `calibration/` times 100 (accelerometer) and 3 (gyroscope), the level measured while driving; bias random walks as in the Allan model |
| GNSS weighting | fixed 0.5 m horizontal, 5 m vertical for both receivers (not the covariance they report); chi2 gate multiplier 9999 |
| Initialization | static, IMU only (`imu_thresh` 0.3); ENU alignment after 50 m of travel |

The extrinsics in these files are those of the released `/tf_static` and the camera intrinsics those of
`camera_info`.

## MINS version

MINS ROS 2 port with the GNSS and wheel changes used for the dataset: https://github.com/gigcastro/MINS, branch
`multi_bags`, commit `074945a1ee2e8eb37fabf5563514c54f6d6392cf`.

## Running

MINS reads the release bags directly. Every split is given, `split_00` included, so that MINS initializes while the
vehicle is still parked; `sys_bag_start` (seconds after the beginning of the sequence) is

| Sequence | `sys_bag_start` |
|---|---|
| `sanfer_sanisidro` | 0 |
| `loop_sanisidro` | 145 (started at 0, the filter breaks down after a long stop with the engine off) |
| `caba` | 135 (the vehicle reverses out of its parking spot at 56-106 s, and the wheel speeds are unsigned) |
| `loop_caba` | 0 |

For example, for `loop_caba`:

```bash
bags=$(for s in /data/buenos_aires/loop_caba/split_*; do printf '%s:' $s/imu_gps $s/cameras $s/lidar $s/wheel; done)
ros2 run mins bag configs/mins/rtk_driving_noise/config.yaml --ros-args -p sys_path_bag:="${bags%:}" \
    -p sys_bag_start:=0.0 -p sys_path_trajectory:=loop_caba_mins.txt
```

Where the engine idles, the IMU vibration exceeds the static-initialization threshold, so MINS can initialize later
than the start, in a short stop or while moving; the MINS trajectories begin 186, 201, 304 and 65 s after the
start of `sanfer_sanisidro`, `loop_sanisidro`, `caba` and `loop_caba`. The `ground_truth` bags of the release have no
`split_00` in the March sequences (the vehicle is parked or leaving its parking spot).
