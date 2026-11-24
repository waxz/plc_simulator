"""
The simulator's tests. Run them from the project folder (python_sim/):

    python -m unittest discover                    # all of them
    python -m unittest tests.test_wheel_sort       # one file
    python -m unittest tests.test_wheel_sort.RowsTest

The tests read layouts and write files relative to the project folder, and
some import helpers from each other (from test_sim_process import ...):
both folders go on the import path here.
"""
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _path in (_TESTS, _ROOT):
    if _path not in sys.path:
        sys.path.insert(0, _path)
