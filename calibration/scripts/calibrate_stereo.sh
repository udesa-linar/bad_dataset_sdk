#!/bin/bash
# Front stereo pair (cam1 + cam5) with Kalibr: intrinsics of both cameras and the cam1-cam5 transform, from the
# AprilGrid subset cams_calib/cam1_cam5 (front-camera stamps = GNSS trigger time, identical for both cameras).
# Usage: ./calibrate_stereo.sh [out_dir]      (default ../output)
# Output: <out>/stereo/aq_cam1_cam5-camchain.yaml (cam0 = cam1, cam1 = cam5, T_cn_cnm1 = cam1 -> cam5),
#         -results-cam.txt, -report-cam.pdf, kalibr.log.   Env: RATE (default 4 Hz)
set -eu
source "$(dirname "$0")/common.sh"
need_rosbags
OUT=$(realpath -m "${1:-$OUTPUTS}"); RATE=${RATE:-4}; d=$OUT/stereo; mkdir -p $d
[ -f $d/aq_cam1_cam5.bag ] || { log "export cam1_cam5 ($RATE Hz)"; $PY $SRC/export_bags.py kalibr $BAGS/cams_calib/cam1_cam5 $d/aq_cam1_cam5.bag --rate $RATE --topics /cam1/image_raw/compressed /cam5/image_raw/compressed; }
cp $CONFIGS/target_aprilgrid.yaml $d/
log "Kalibr stereo cam1 + cam5"
kalibr_run $d "rosrun kalibr kalibr_calibrate_cameras --bag aq_cam1_cam5.bag --target target_aprilgrid.yaml \
    --models pinhole-radtan pinhole-radtan --topics /cam1/image_raw/compressed /cam5/image_raw/compressed \
    --dont-show-report > kalibr.log 2>&1" && log "done: $d" || log "FAILED (see $d/kalibr.log)"
grep -E "cam[01] |projection|distortion|reprojection|q:|t:" $d/aq_cam1_cam5-results-cam.txt 2>/dev/null || true
