"""Shared pytest configuration for pyTheia tests."""

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
_SFM_TEST_DIR = _REPO_ROOT / "pytests" / "sfm"
_BUILD_LIB = _REPO_ROOT / "cmake_build" / "lib"

for _path in (_SFM_TEST_DIR, _BUILD_LIB):
    if _path.is_dir() and str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
