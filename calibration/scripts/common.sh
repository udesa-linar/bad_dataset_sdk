# Shared helpers of the calibration scripts (sourced). Requirements: Docker with the images
#   kalibr:latest       Kalibr (ROS 1 Noetic), e.g. built from https://github.com/ethz-asl/kalibr (Dockerfile_ros1_20_04)
#   koide-vlcal:release  docker build -t koide-vlcal:release calibration/scripts/koide   (stock koide + Xvfb + initial_guess_from_points)
# and Python 3 with the SDK installed with the calib extra (pip install -e ".[calib]": mcap, numpy, pyyaml, scipy, rosbags)
SRC=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)       # calibration/scripts
ROOT=$(dirname "$SRC")                                   # calibration
# CALIB_DATA: the downloaded calib/ folder of the release (cams_calib/, lidar_cam_calib/, allan/)
if [ -z "${CALIB_DATA:-}" ] || [ ! -d "$CALIB_DATA" ]; then
  echo "error: set CALIB_DATA to the release calib/ folder, e.g.  CALIB_DATA=/data/buenos_aires/calib $0"; exit 1
fi
BAGS=$(cd "$CALIB_DATA" && pwd)
CONFIGS=$ROOT/resources; OUTPUTS=${CALIB_OUTPUT:-$ROOT/output}
PY=${PY:-python3}
if ! $PY -c "import mcap, numpy, yaml, scipy" 2>/dev/null; then
  echo "error: $PY lacks mcap/numpy/pyyaml/scipy: pip install -e '.[calib]' in the SDK, or point PY to a Python"
  echo "       that has them, e.g.  PY=/path/to/venv/bin/python ./$(basename "$0") ..."; exit 1
fi
# rosbags is needed only to write the ROS 1 bag Kalibr reads and the ROS 2 Humble bags koide reads (export_bags.py);
# the data are always read from the released MCAP files
need_rosbags() {
  $PY -c "import rosbags" 2>/dev/null || { echo "error: $PY lacks rosbags (pip install rosbags), needed to export bags for Kalibr/koide"; exit 1; }
}
log() { echo "=== $* $(date '+%F %T')"; }
kalibr_run() {  # out_dir command...   (out_dir is mounted as /data)
  local wd=$1; shift
  # src/kalibr_seed/sitecustomize.py seeds Python's and NumPy's random generators (Kalibr shuffles views unseeded),
  # so that a run is reproducible; Kalibr itself is not modified
  docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -e ROS_HOME=/tmp/ros --entrypoint bash -v "$wd":/data \
    -v "$SRC/kalibr_seed":/kalibr_seed:ro kalibr:latest -c \
    "source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; export PYTHONPATH=/kalibr_seed:\$PYTHONPATH; export MPLBACKEND=Agg; cd /data; $*"
}
koide_run() {   # out_dir command...
  local wd=$1; shift
  # runs as root (the koide workspace is under /root); the outputs are handed back to the calling user
  docker run --rm -e ROS_HOME=/tmp/ros --entrypoint bash -v "$wd":/data koide-vlcal:release -c \
    "trap 'chown -R $(id -u):$(id -g) /data' EXIT; source /root/ros2_ws/install/setup.bash; cd /data; $*"
}
