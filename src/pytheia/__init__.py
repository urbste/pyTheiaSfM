"""High-level **pyTheia** package: Structure-from-Motion and geometry for Python.

This is the public import path. The implementation lives in the compiled module
``pytheia.pytheia`` (pybind11). Typical usage::

    import pytheia as pt
    rec = pt.sfm.Reconstruction()

Submodules re-exported here: ``io``, ``math``, ``matching``, ``mvs``, ``sfm``,
``solvers``. Inline types and functions are documented via ``.pyi`` stubs
(:pep:`561`) next to the extension.

For narrative documentation see the project manual (MkDocs: Python overview and
C++-oriented API chapters), which describe the underlying Theia concepts.
"""

from importlib import metadata
from pathlib import Path

try:
    __version__ = metadata.version("pytheia")
except metadata.PackageNotFoundError:
    _vfile = Path(__file__).resolve().parents[2] / "VERSION"
    __version__ = (
        _vfile.read_text(encoding="utf-8").strip()
        if _vfile.is_file()
        else "0.0.0"
    )

from pytheia.pytheia import (io, math, matching, mvs, sfm, solvers)
import pytheia.pytheia as pytheia
