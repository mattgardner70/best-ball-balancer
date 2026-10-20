"""CadQuery models of every printed part. Build them all with `uv run python -m CAD.build`."""

import sys
from pathlib import Path

_CODE = str(Path(__file__).resolve().parent.parent / "code")   # the sim's parameters live in code/sim
if _CODE not in sys.path:
    sys.path.insert(0, _CODE)
