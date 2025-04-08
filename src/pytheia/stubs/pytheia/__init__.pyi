"""Type stubs for the pytheia package.

This package provides Python bindings for TheiaSfM, a computer vision library for:
- Structure from Motion (SfM)
- Multi-View Stereo (MVS) 
- Feature Detection and Matching
- Geometric Vision and Camera Models
- Robust Optimization and Estimation
"""

from typing import Any, Dict, List, Optional, Tuple, Union, TypeVar, Sequence
import numpy as np
from numpy.typing import NDArray

# Version and metadata
__version__: str
__author__: str

# Type variables for type hints
_T = TypeVar('_T')
_FloatArray = NDArray[np.float64]
_IntArray = NDArray[np.int64]
Vector2d = NDArray[np.float64]  # shape (2,)
Vector3d = NDArray[np.float64]  # shape (3,)
Matrix3d = NDArray[np.float64]  # shape (3,3)
Matrix4d = NDArray[np.float64]  # shape (4,4)

# Import submodules
from pytheia.pytheia import io, math, matching, mvs, sfm, solvers

# Re-export submodules
__all__ = ['io', 'math', 'matching', 'mvs', 'sfm', 'solvers']