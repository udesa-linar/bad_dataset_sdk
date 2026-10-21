# MINS configuration of the reference trajectory

The reference trajectory of the dataset (`ground_truth/` bags) was computed with MINS (multisensor-aided inertial
navigation, Lee et al. 2023), fusing the MTi-610, the front stereo pair, the LiDAR, the rear wheel speeds and the
fixes of both GNSS receivers, with every calibration fixed at its released value and every time offset zero.

| Config | Sequences | GNSS weighting |
|---|---|---|
| `rtk/` | `loop_sanisidro`, `caba`, `loop_caba` | covariance reported by each receiver |
| `rtk_sfsi/` | `sanfer_sanisidro` | fixed 0.05 m horizontal, 0.1 m vertical (its receivers report 0.7 m / 1.2 m while their fixes vary by 1-3 cm; see the paper) |

The extrinsics in these files are those of the released `/tf_static` and the camera intrinsics those of
`camera_info`; the IMU noise is the Allan-variance model of `calibration/`.

## MINS version

MINS ROS 2 port with the GNSS and wheel changes used for the dataset: https://github.com/gigcastro/MINS, branch
`multi_bags`, commit `074945a1ee2e8eb37fabf5563514c54f6d6392cf`.

## Running

MINS reads the release bags directly; the March runs start at `split_01` (`split_00` is parked) and
`sanfer_sanisidro` 13 s after its start. For example, for `loop_caba`:

```bash
bags=$(for s in /data/buenos_aires/loop_caba/split_0[1-9]; do printf '%s:' $s/imu_gps $s/cameras $s/lidar $s/wheel; done)
ros2 run mins bag configs/mins/rtk/config.yaml --ros-args -p sys_path_bag:="${bags%:}" -p sys_bag_start:=0.0 \
    -p sys_path_trajectory:=loop_caba_mins.txt
```
