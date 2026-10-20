"""MuJoCo simulation of the balancer. Run the viewer from code/: `uv run mjpython -m sim.view`."""

import sys
from pathlib import Path

_ROOT = str(Path(__file__).resolve().parent.parent.parent)      # repo root, for the CAD package
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
