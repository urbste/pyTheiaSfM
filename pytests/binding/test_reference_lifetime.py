"""Binding ownership tests for reference_internal return policies.

These exercises nanobind migration risk sites: mutable references into parent
objects must remain valid while the parent lives.
"""

import gc

import numpy as np
import pytheia as pt

from random_recon_gen import RandomReconGenerator


def test_mutable_camera_mutation_visible_via_view():
    recon = RandomReconGenerator(seed=1).generate_random_recon(
        nr_views=3, nr_tracks=10
    )
    vid = next(iter(recon.ViewIds()))
    cam = recon.MutableView(vid).MutableCamera()
    cam.SetPosition(np.array([1.0, 2.0, 3.0]))
    assert np.allclose(recon.View(vid).Camera().GetPosition(), [1.0, 2.0, 3.0])


def test_mutable_track_mutation_round_trip():
    recon = pt.sfm.Reconstruction()
    view_id = recon.AddView("v0", 0.0)
    recon.MutableView(view_id).SetIsEstimated(True)
    track_id = recon.AddTrack()
    point = np.array([1.0, 2.0, 3.0, 1.0], dtype=np.float64)
    recon.MutableTrack(track_id).SetPoint(point)
    recon.MutableTrack(track_id).SetIsEstimated(True)
    assert np.allclose(recon.Track(track_id).Point()[:3], [1.0, 2.0, 3.0])


def test_camera_intrinsics_reference_mutation():
    camera = pt.sfm.Camera()
    camera.SetFocalLength(500.0)
    intrinsics = camera.CameraIntrinsics()
    intrinsics.SetFocalLength(750.0)
    assert camera.FocalLength() == 750.0


def test_mutable_camera_intrinsics_prior_on_view():
    recon = RandomReconGenerator(seed=4).generate_random_recon(
        nr_views=2, nr_tracks=5
    )
    vid = next(iter(recon.ViewIds()))
    prior = recon.MutableView(vid).MutableCameraIntrinsicsPrior()
    prior.image_width = 800
    prior.image_height = 600
    assert recon.View(vid).CameraIntrinsicsPrior().image_width == 800


def test_get_feature_reference_matches_mutable_track():
    gen = RandomReconGenerator(seed=0)
    gen.generate_random_recon(nr_views=2, nr_tracks=8)
    recon = gen.recon
    v0 = list(recon.ViewIds())[0]
    feat = None
    for tid in recon.TrackIds():
        feat = recon.View(v0).GetFeature(tid)
        if feat is not None:
            break
    assert feat is not None, "expected at least one shared observation in view 0"
    assert len(feat.point) == 2


def _build_view_graph_from_recon(recon):
    """Build a ViewGraph with edges between all view pairs (like view_graph_test)."""
    view_graph = pt.sfm.ViewGraph()
    vids = list(recon.ViewIds())
    for i, id1 in enumerate(vids):
        ri = recon.View(id1).Camera().GetOrientationAsAngleAxis()
        for id2 in vids[i:]:
            rj = recon.View(id2).Camera().GetOrientationAsAngleAxis()
            two_view_info = pt.sfm.TwoViewInfo()
            two_view_info.focal_length_1 = 1.0
            two_view_info.focal_length_2 = 1.0
            two_view_info.position_2 = np.zeros(3, dtype=np.float64)
            two_view_info.rotation_2 = pt.math.RelativeRotationFromTwoRotations(
                ri, rj
            )
            view_graph.AddEdge(id1, id2, two_view_info)
    return view_graph


def test_view_graph_get_edge_reference():
    gen = RandomReconGenerator(seed=6)
    gen.generate_random_recon(nr_views=4, nr_tracks=12)
    recon = gen.recon
    vids = list(recon.ViewIds())
    view_graph = _build_view_graph_from_recon(recon)
    id1, id2 = vids[0], vids[1]
    assert view_graph.HasEdge(id1, id2)
    edge = view_graph.GetEdge(id1, id2)
    assert edge is not None
    edge.focal_length_1 = 42.0
    edge2 = view_graph.GetEdge(id1, id2)
    assert edge2.focal_length_1 == 42.0


def test_view_graph_neighbor_ids_reference():
    gen = RandomReconGenerator(seed=7)
    gen.generate_random_recon(nr_views=4, nr_tracks=12)
    recon = gen.recon
    vids = list(recon.ViewIds())
    view_graph = _build_view_graph_from_recon(recon)
    neighbors = view_graph.GetNeighborIdsForView(vids[0])
    assert len(neighbors) >= 1


def test_reference_internal_keeps_parent_alive():
    """reference_internal extends parent lifetime through child references."""
    recon = RandomReconGenerator(seed=8).generate_random_recon(
        nr_views=2, nr_tracks=5
    )
    vid = next(iter(recon.ViewIds()))
    cam = recon.MutableView(vid).MutableCamera()
    cam.SetPosition(np.array([0.0, 0.0, 1.0]))
    del recon
    gc.collect()
    assert np.allclose(cam.GetPosition(), [0.0, 0.0, 1.0])
