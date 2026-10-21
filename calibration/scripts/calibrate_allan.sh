#!/bin/bash
# IMU noise model of the Xsens MTi-610 from the static recording calib/allan/imu (16 h, 100 Hz):
#   1 extract_imu_csv.py  the 21:00-07:00 local-time segment (10 h) to a CSV   -> <out>/allan/imu_2100-0700.csv
#   2 allan_variance.py   overlapping Allan deviation, noise densities, random walks, bias instability
#                                                                         -> <out>/allan/allan_2100-0700/
#   3 the noise model used by the dataset (Kalibr format)                -> <out>/allan/imu_noise.yaml
# Usage: ./calibrate_allan.sh [out_dir]      (default calibration/output, or $CALIB_OUTPUT). Needs Python 3 with mcap, numpy, matplotlib.
set -eu
source "$(dirname "$0")/common.sh"
OUT=$(realpath -m "${1:-$OUTPUTS}"); d=$OUT/allan; mkdir -p $d
log "extract 21:00-07:00"; $PY $SRC/allan/extract_imu_csv.py --bag $BAGS/allan/imu --start 21:00 --end 07:00 -o $d/imu_2100-0700.csv
log "Allan deviation";     $PY $SRC/allan/allan_variance.py $d/imu_2100-0700.csv --out $d/allan_2100-0700
cp $d/allan_2100-0700/imu_noise.yaml $d/imu_noise.yaml
cat $d/imu_noise.yaml
