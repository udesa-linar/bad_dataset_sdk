#!/bin/bash
# LiDAR-camera extrinsics of cam1-cam5 with koide3 direct_visual_lidar_calibration, from the 2D-3D correspondences
# picked by hand in koide's initial_guess_manual (resources/lidar_camera_correspondences/<cam>.yaml).
#
#   1 windows    two short stops of the vehicle in lidar_cam_calib with the checkerboard close in front of the camera
#                (windows_of below), exported to Humble-readable ROS 2 bags with the camera image, its camera_info and
#                the LiDAR scans only                                     -> <out>/lidar_camera/<cam>/bags/window_XX
#   2 preprocess koide's preprocess (static accumulation of each window), with the intrinsics of
#                calibrate_intrinsics.sh (cam2-4) / calibrate_stereo.sh (cam1, cam5) -> <out>/lidar_camera/<cam>/window_XX.*
#   3 estimate   koide's pose estimation (PoseEstimation, as the "Estimate" button of initial_guess_manual) on the
#                correspondences                 -> <out>/lidar_camera/<cam>/calib.json (results.T_lidar_camera:
#                x y z qx qy qz qw, camera pose in the LiDAR frame)
# cam5 (same stops as cam1) is a cross-check only: in the released transform tree cam5 is placed through cam1 and the
# stereo calibration.
#
# Usage: ./calibrate_lidar_camera.sh [cam1..cam5|all] [out_dir]        (defaults: all, ../output)
# Needs the intrinsics in <out_dir> (run calibrate_intrinsics.sh and calibrate_stereo.sh first) and the Docker image
# koide-vlcal:release (docker build -t koide-vlcal:release calibration/scripts/koide).
set -eu
source "$(dirname "$0")/common.sh"
need_rosbags
CAMS=${1:-all}; [ "$CAMS" = all ] && CAMS="cam1 cam2 cam3 cam4 cam5"
OUT=$(realpath -m "${2:-$OUTPUTS}")
windows_of() { case $1 in cam1|cam5) echo 106.5-107.8,125.1-126.2;; cam2) echo 242.1-242.9,273.1-273.9;;
                          cam3) echo 492.1-492.9,505.2-507.3;; cam4) echo 351.1-352.9,381.1-382.4;; esac; }
for c in $CAMS; do
  case $c in cam1) y=$OUT/stereo/aq_cam1_cam5-camchain.yaml; k=cam0;; cam5) y=$OUT/stereo/aq_cam1_cam5-camchain.yaml; k=cam1;; *) y=$OUT/intrinsics/$c/aq_$c-camchain.yaml; k=cam0;; esac
  [ -f $y ] || { log "$c SKIPPED: no intrinsics ($y)"; continue; }
  CORR=$CONFIGS/lidar_camera_correspondences/$c.yaml
  [ -f $CORR ] || { log "$c SKIPPED: no correspondences ($CORR)"; continue; }
  d=$OUT/lidar_camera/$c; rm -rf $d; mkdir -p $d
  # 1 windows
  log "$c windows $(windows_of $c) s"
  $PY $SRC/export_bags.py koide $BAGS/lidar_cam_calib $d/bags.tmp --ranges $(windows_of $c) --images /$c/image_raw/compressed
  mv $d/bags.tmp/$c $d/bags; rm -rf $d/bags.tmp
  # 2 preprocess
  read K DC < <($PY -c "
import yaml; c=yaml.safe_load(open('$y'))['$k']
print(','.join(f'{v:.6f}' for v in c['intrinsics']), ','.join(f'{v:.8f}' for v in c['distortion_coeffs']+[0.0]))")
  log "$c preprocess"
  koide_run $d "ros2 run direct_visual_lidar_calibration preprocess --data_path /data/bags --dst_path /data \
      --image_topic /$c/image_raw/compressed --camera_info_topic /$c/camera_info --points_topic /velodyne_points \
      --camera_model plumb_bob --camera_intrinsics $K --camera_distortion_coeffs $DC > /data/preprocess.log 2>&1"
  # 3 estimate from the correspondences
  $PY - $d/calib.json $CORR <<'PYEOF'
import json, sys, yaml
p, corr = sys.argv[1:]
c = json.load(open(p)); c.setdefault('results', {})
c['results']['manual_correspondences'] = [list(q['camera_uv']) + list(q['lidar_xyz']) for q in yaml.safe_load(open(corr))['correspondences']]
json.dump(c, open(p, 'w'), indent=2)
PYEOF
  log "$c koide pose estimation from $(basename $CORR)"
  koide_run $d "ros2 run direct_visual_lidar_calibration initial_guess_from_points /data > /data/estimate.log 2>&1"
  $PY - $d/calib.json <<'PYEOF'
import json, sys
p = sys.argv[1]; c = json.load(open(p)); c['results']['T_lidar_camera'] = c['results']['init_T_lidar_camera']
json.dump(c, open(p, 'w'), indent=2); print('T_lidar_camera', [round(v, 4) for v in c['results']['T_lidar_camera']])
PYEOF
done
