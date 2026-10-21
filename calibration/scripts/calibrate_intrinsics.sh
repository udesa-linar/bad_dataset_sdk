#!/bin/bash
# Intrinsics of the lateral and rear cameras (cam2, cam3, cam4) with Kalibr, pinhole + radial-tangential model,
# from the AprilGrid subsets cams_calib/<cam>.
# Usage: ./calibrate_intrinsics.sh [cam2|cam3|cam4|all] [out_dir]      (default: all, ../output)
# Output: <out>/intrinsics/<cam>/aq_<cam>-camchain.yaml (intrinsics, distortion), -results-cam.txt (reprojection
#         error), -report-cam.pdf, kalibr.log.   Env: RATE (Hz of the images used, default 4)
set -eu
source "$(dirname "$0")/common.sh"
need_rosbags
CAMS=${1:-all}; [ "$CAMS" = all ] && CAMS="cam2 cam3 cam4"
OUT=$(realpath -m "${2:-$OUTPUTS}"); RATE=${RATE:-4}
for c in $CAMS; do
  d=$OUT/intrinsics/$c; mkdir -p $d
  [ -f $d/aq_$c.bag ] || { log "export $c ($RATE Hz)"; $PY $SRC/export_bags.py kalibr $BAGS/cams_calib/$c $d/aq_$c.bag --rate $RATE --topics /$c/image_raw/compressed; }
  cp $CONFIGS/target_aprilgrid.yaml $d/
  log "Kalibr intrinsics $c"
  kalibr_run $d "rosrun kalibr kalibr_calibrate_cameras --bag aq_$c.bag --target target_aprilgrid.yaml --models pinhole-radtan \
      --topics /$c/image_raw/compressed --dont-show-report > kalibr.log 2>&1" && log "$c done: $d" || log "$c FAILED (see $d/kalibr.log)"
  grep -E "projection|distortion|reprojection" $d/aq_$c-results-cam.txt 2>/dev/null || true
done
