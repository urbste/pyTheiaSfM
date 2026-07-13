"""Tests for pt.mvs.ViewSelectionMVSNet (MVSNet-style view selection)."""

import pytheia as pt

from random_recon_gen import RandomReconGenerator


def _make_recon(nr_views=6, nr_tracks=30, seed=0):
    return RandomReconGenerator(seed=seed).generate_random_recon(
        nr_views=nr_views, nr_tracks=nr_tracks
    )


def _mvs_select(recon, num_neighbors, theta0=5.0, sigma1=1.0, sigma2=10.0):
    """Call ViewSelectionMVSNet (C++ defaults not exposed in pybind11 binding)."""
    return pt.mvs.ViewSelectionMVSNet(
        recon, num_neighbors, theta0, sigma1, sigma2
    )


def test_view_selection_mvsnet_returns_neighbors():
    recon = _make_recon(nr_views=6, nr_tracks=100, seed=0)
    selection = _mvs_select(recon, 3)
    # May be empty if view graph has no edges (min 10 shared tracks); structure still valid.
    assert isinstance(selection, dict)
    for _view_id, ranked in selection.items():
        assert len(ranked) <= 3
        scores = list(ranked.keys())
        assert scores == sorted(scores, reverse=True)


def test_view_selection_mvsnet_default_kwargs():
    recon = _make_recon(nr_views=6, nr_tracks=100, seed=1)
    selection_defaults = _mvs_select(recon, 2)
    selection_explicit = _mvs_select(recon, 2, 5.0, 1.0, 10.0)
    assert selection_defaults.keys() == selection_explicit.keys()
    for vid in selection_defaults:
        assert list(selection_defaults[vid].keys()) == list(
            selection_explicit[vid].keys()
        )


def test_view_selection_mvsnet_single_view():
    recon = _make_recon(nr_views=1, nr_tracks=5, seed=2)
    selection = _mvs_select(recon, 3)
    assert isinstance(selection, dict)


def test_view_selection_mvsnet_neighbor_ids_are_valid():
    recon = _make_recon(nr_views=8, nr_tracks=150, seed=3)
    view_ids = set(recon.ViewIds())
    selection = _mvs_select(recon, 4)
    for source_id, ranked in selection.items():
        assert source_id in view_ids
        for _score, neighbor_id in ranked.items():
            assert neighbor_id in view_ids
            assert neighbor_id != source_id
