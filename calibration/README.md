# Reproducing the calibration

Every calibration of the paper can be recomputed from the released calibration recordings (`calib/`) with the
scripts here, and compared with the calibration released in the dataset with `verify.py`.

| Calibration | Script | Input (`calib/`) | Tool | Output |
|---|---|---|---|---|
| IMU noise model (Allan deviation) | `scripts/calibrate_allan.sh` | `allan/imu` | own (`scripts/allan/`) | `allan/` |
| Intrinsics cam2, cam3, cam4 | `scripts/calibrate_intrinsics.sh` | `cams_calib/cam2`, `cam3`, `cam4` | Kalibr | `intrinsics/<cam>/` |
| Stereo cam1 + cam5 | `scripts/calibrate_stereo.sh` | `cams_calib/cam1_cam5` | Kalibr | `stereo/` |
| LiDAR-camera cam1..cam5 | `scripts/calibrate_lidar_camera.sh` | `lidar_cam_calib/` | koide3 direct_visual_lidar_calibration | `lidar_camera/<cam>/` |

cam5 is placed in the released transform tree through cam1 and the stereo calibration; its direct LiDAR-camera
calibration is a cross-check.

## Requirements

- The SDK with the calibration extra: `pip install -e ".[calib]"`. The recordings are read from their MCAP files;
  `rosbags` only writes the ROS 1 bag that Kalibr reads and the ROS 2 Humble bags that the koide toolbox reads.
- Docker, with two images:
  - `kalibr:latest`: Kalibr (ROS 1 Noetic), built from https://github.com/ethz-asl/kalibr at commit `1f60227` (the one
    used for the paper) with its `Dockerfile_ros1_20_04`:
    ```bash
    git clone https://github.com/ethz-asl/kalibr && git -C kalibr checkout 1f60227442d25e36365ef5f72cd80b9666d73467
    docker build -t kalibr:latest -f kalibr/Dockerfile_ros1_20_04 kalibr
    ```
  - `koide-vlcal:release`: `docker build -t koide-vlcal:release calibration/scripts/koide` (the koide3 toolbox
    with a virtual display and a helper that seeds the pose from the correspondences).

Kalibr shuffles its views with an unseeded random generator; `scripts/kalibr_seed/sitecustomize.py` seeds it so that
a run is reproducible. Kalibr itself is not modified.

## Running

```bash
export CALIB_DATA=/data/buenos_aires/calib      # the downloaded calib/ folder
export CALIB_OUTPUT=$PWD/calibration/output     # default
calibration/scripts/calibrate_allan.sh
calibration/scripts/calibrate_intrinsics.sh
calibration/scripts/calibrate_stereo.sh
calibration/scripts/calibrate_lidar_camera.sh   # needs the intrinsics and the stereo calibration above
python calibration/verify.py --split /data/buenos_aires/caba/split_03
```

`verify.py` prints, for each camera, the difference of the intrinsics with the released `camera_info`, and for each
transform the translation [cm] and rotation [deg] with respect to the released `/tf_static`, plus the IMU noise
against the values of the paper.

## LiDAR-camera correspondences

The LiDAR-camera extrinsics were estimated from 2D-3D correspondences picked by hand in the koide toolbox, on two
stops of the vehicle in front of the checkerboard per camera. Those correspondences are released in
`resources/lidar_camera_correspondences/<cam>.yaml` (the pairs left more than 15 px from the pose were removed, as in
the paper), so the estimate can be reproduced without picking them again. The stop windows (seconds after the start of
`lidar_cam_calib`) are in `scripts/calibrate_lidar_camera.sh`.
