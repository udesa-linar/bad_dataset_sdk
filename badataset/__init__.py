"""Tools for the Buenos Aires Dataset.

reader       Dataset / Sequence / Split: messages, images, LiDAR clouds, without ROS
calib        /tf_static tree, camera intrinsics, LiDAR-to-camera projection
groundtruth  the reference trajectory from the ground_truth bags, in any sensor frame
gnss         fixes of the RTK receiver with their carrier-phase status
evaluate     relative / absolute trajectory errors, errors against the GNSS fixes
sync         front stereo pairs with the LiDAR scan that triggered them
download     fetch the whole release or only some sequences / sensor groups from Hugging Face
"""
from .reader import Dataset, Sequence, Split
from . import calib, evaluate, groundtruth, gnss, sync

__version__ = '0.1.0'
