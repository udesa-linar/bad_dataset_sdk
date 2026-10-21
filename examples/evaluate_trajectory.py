"""Evaluate an estimated trajectory against the reference trajectory of a sequence, with the metrics of the paper.

The estimate is a TUM file (t x y z qx qy qz qw) of the IMU pose in the reference ENU frame (the frame of
/ground_truth/pose; its WGS-84 origin is Reference.origin). For an estimate of another sensor frame, pass --frame.

    python evaluate_trajectory.py /data/buenos_aires loop_caba my_estimate.txt
    python evaluate_trajectory.py /data/buenos_aires caba est.txt --splits split_03 --frame lidar
"""
import argparse
import json

from badataset import Dataset, evaluate as E
from badataset.groundtruth import Reference

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('root'); ap.add_argument('sequence'); ap.add_argument('estimate')
ap.add_argument('--splits', nargs='*', help='only these splits (default: all)')
ap.add_argument('--frame', default='imu', help='frame of the estimate (default imu)')
a = ap.parse_args()

seq = Dataset(a.root).sequence(a.sequence, a.splits)
ref = Reference(seq)
gt = ref.traj if a.frame == 'imu' else ref.in_frame(a.frame)
est = E.load_tum(a.estimate)
g = E.gnss_errors(seq, est, ref.origin, frame=a.frame)
g.pop('epochs')
print(json.dumps(dict(rpe=E.rpe_kitti(gt, est), ate_se3=E.ate_se3(gt, est), gnss=g), indent=1))
