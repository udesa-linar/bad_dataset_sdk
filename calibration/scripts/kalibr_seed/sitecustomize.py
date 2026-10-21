# Loaded by Python at start-up (mounted on PYTHONPATH by src/common.sh): fixes the seeds of Python's and NumPy's random
# generators, which Kalibr uses (unseeded) when it orders/selects views, so that every run gives identical results.
import random
random.seed(0)
try:
    import numpy
    numpy.random.seed(0)
except Exception:
    pass
