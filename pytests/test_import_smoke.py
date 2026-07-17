"""Smoke test: all public submodules are importable."""

import pytheia as pt


def test_public_submodules_exist():
    for sub in ("io", "matching", "math", "mvs", "sfm", "solvers"):
        assert hasattr(pt, sub), f"missing submodule: {sub}"


def test_extension_module_loads():
    import pytheia as pt

    assert hasattr(pt, "sfm")
    assert pt.sfm.Reconstruction is not None
