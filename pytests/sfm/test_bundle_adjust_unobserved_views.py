"""Regression: bundle adjustment with estimated views that have no observations.

Such views are estimated but have no residual blocks. Their parameter blocks
used to be put into the Ceres linear solver ordering anyway, which failed with
"The program has N parameter blocks, but the parameter block ordering has N+4".
"""

import numpy as np
import pytest

import pytheia as pt
from random_recon_gen import RandomReconGenerator


def _options(**kwargs):
    opts = pt.sfm.BundleAdjustmentOptions()
    opts.num_threads = 1
    opts.verbose = False
    for name, value in kwargs.items():
        setattr(opts, name, value)
    return opts


def _unobserved_view_ids(recon):
    return [vid for vid in recon.ViewIds() if len(recon.View(vid).TrackIds()) == 0]


def _positions(recon, view_ids):
    return {vid: np.array(recon.View(vid).Camera().GetPosition()) for vid in view_ids}


def _generator_recon():
    return RandomReconGenerator(seed=1).generate_random_recon(nr_views=60, nr_tracks=6000)


def test_repro_generator_has_unobserved_views():
    recon = _generator_recon()
    assert len(_unobserved_view_ids(recon)) >= 1
    # The generator puts every view into one shared intrinsics group, so the
    # unobserved views share their intrinsics block with observed views.
    assert len(recon.CameraIntrinsicsGroupIds()) == 1


def test_bundle_adjust_reconstruction_with_unobserved_views():
    recon = _generator_recon()
    unobserved = _unobserved_view_ids(recon)
    before = _positions(recon, unobserved)

    summary = pt.sfm.BundleAdjustReconstruction(_options(), recon)

    assert summary.success
    # Views without residuals are not part of the problem and must not move.
    after = _positions(recon, unobserved)
    for vid in unobserved:
        np.testing.assert_array_equal(after[vid], before[vid])


def _run_entry_point(name, recon, unobserved):
    observed = [v for v in recon.ViewIds() if v not in unobserved]
    tracks = list(recon.TrackIds())
    opts = _options()
    if name == "BundleAdjustView":
        return pt.sfm.BundleAdjustView(recon, opts, unobserved[0])
    if name == "BundleAdjustViews":
        return pt.sfm.BundleAdjustViews(recon, opts, unobserved + observed[:5])
    if name == "BundleAdjustViewWithCov":
        return pt.sfm.BundleAdjustViewWithCov(recon, opts, unobserved[0])
    if name == "BundleAdjustViewsWithCov":
        return pt.sfm.BundleAdjustViewsWithCov(recon, opts, unobserved + observed[:5])
    if name == "BundleAdjustPartialReconstruction":
        return pt.sfm.BundleAdjustPartialReconstruction(
            opts, set(unobserved + observed[:10]), set(tracks[:200]), recon)
    if name == "BundleAdjustPartialViewsConstant":
        return pt.sfm.BundleAdjustPartialViewsConstant(
            opts, unobserved + observed[:10], observed[10:20], recon)
    if name == "BundleAdjustPartialViewsConstantWithRelativePoseEdges":
        return pt.sfm.BundleAdjustPartialViewsConstantWithRelativePoseEdges(
            opts, unobserved + observed[:10], observed[10:20], [], recon)
    if name == "BundleAdjustReconstructionWithConstantTracks":
        return pt.sfm.BundleAdjustReconstructionWithConstantTracks(
            opts, set(tracks[:50]), recon)
    if name == "BundleAdjustReconstructionWithRelativePoseEdges":
        return pt.sfm.BundleAdjustReconstructionWithRelativePoseEdges(opts, [], recon)
    raise ValueError(name)


@pytest.mark.parametrize("name", [
    "BundleAdjustView",
    "BundleAdjustViews",
    "BundleAdjustViewWithCov",
    "BundleAdjustViewsWithCov",
    "BundleAdjustPartialReconstruction",
    "BundleAdjustPartialViewsConstant",
    "BundleAdjustPartialViewsConstantWithRelativePoseEdges",
    "BundleAdjustReconstructionWithConstantTracks",
    "BundleAdjustReconstructionWithRelativePoseEdges",
])
def test_entry_points_handle_unobserved_views(name):
    recon = _generator_recon()
    unobserved = _unobserved_view_ids(recon)
    before = _positions(recon, unobserved)

    result = _run_entry_point(name, recon, unobserved)

    summary = result[0] if isinstance(result, tuple) else result
    assert np.isfinite(summary.final_cost)
    after = _positions(recon, unobserved)
    for vid in unobserved:
        np.testing.assert_array_equal(after[vid], before[vid])
