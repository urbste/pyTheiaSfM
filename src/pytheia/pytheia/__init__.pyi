"""
Low-level native module for pyTheia (Theia SfM). Import the public API via ``import pytheia`` or ``import pytheia as pt``; that package re-exports submodules ``io``, ``math``, ``matching``, ``mvs``, ``sfm``, and ``solvers`` from this extension. Type hints ship as ``.pyi`` stubs next to the binary (PEP 561).
"""
from __future__ import annotations
from . import io
from . import matching
from . import math
from . import mvs
from . import sfm
from . import solvers
__all__: list[str] = ['io', 'matching', 'math', 'mvs', 'sfm', 'solvers']
