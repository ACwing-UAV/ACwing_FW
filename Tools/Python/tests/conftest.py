import os
import sys

import matplotlib

matplotlib.use("Agg")                                   # no windows during the tests
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

DATA = os.path.join(os.path.dirname(__file__), "data")
