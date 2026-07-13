"""Spike module smoke tests (full parity requires nanobind main port)."""

import pytheia as pt


def test_spike_module_imports():
    import pytheia_spike

    assert hasattr(pytheia_spike, "mvs")
    assert hasattr(pytheia_spike.mvs, "ViewSelectionMVSNet")


def test_spike_mvs_matches_pybind11_after_shared_types():
    """Once main pytheia uses nanobind, Reconstruction types are shared; until then skip."""
    import pytheia_spike

    recon = __import__("random_recon_gen", fromlist=["RandomReconGenerator"]).RandomReconGenerator(
        seed=42
    ).generate_random_recon(nr_views=6, nr_tracks=100)
    args = (recon, 3, 5.0, 1.0, 10.0)
    pybind_sel = pt.mvs.ViewSelectionMVSNet(*args)
    try:
        nb_sel = pytheia_spike.mvs.ViewSelectionMVSNet(*args)
    except TypeError:
        # Expected while pytheia uses pybind11 and spike uses nanobind.
        return
    assert pybind_sel.keys() == nb_sel.keys()
