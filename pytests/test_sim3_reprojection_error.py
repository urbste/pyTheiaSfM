#!/usr/bin/env python3
"""
Tests for Sim(3) camera poses in pyTheiaSfM.

Verifies that the Sim(3) pose helpers (GetSim3LieFromView,
RelativeSim3BetweenViews, Sim3FromRotationTranslationScale, ...) agree with the
regular camera model, i.e. that reprojecting through the Sim(3) pose gives the
same pixels as Camera.ProjectPoint.
"""

import numpy as np
import pytest
import pytheia as pt


def _add_view(recon, name, position, angle_axis):
    view_id = recon.AddView(name, 0, float(recon.NumViews()))
    camera = recon.MutableView(view_id).MutableCamera()
    camera.SetFocalLength(1000.0)
    camera.SetPrincipalPoint(500.0, 400.0)
    camera.SetPosition(np.asarray(position, dtype=float))
    camera.SetOrientationFromAngleAxis(np.asarray(angle_axis, dtype=float))
    recon.MutableView(view_id).SetIsEstimated(True)
    return view_id


def _project_with_sim3(sim3_lie, camera, point_3d):
    """Projects a world point using the 4x4 Sim(3) camera-from-world matrix."""
    T_c_w = pt.sfm.Sim3ToHomogeneousMatrix(sim3_lie)
    point_camera = T_c_w[:3, :3] @ point_3d + T_c_w[:3, 3]
    K = camera.GetCalibrationMatrix()
    pixel = K @ (point_camera / point_camera[2])
    return pixel[:2]


def test_sim3_reprojection_error_basic():
    """Projecting through the Sim(3) pose matches Camera.ProjectPoint."""
    recon = pt.sfm.Reconstruction()
    view_id = _add_view(recon, "test_view", [1.0, 2.0, 3.0], [0.1, -0.2, 0.05])
    view = recon.View(view_id)
    camera = view.Camera()

    sim3_lie = pt.sfm.GetSim3LieFromView(view)
    # A view's camera pose has unit scale, i.e. log(scale) == 0.
    assert abs(sim3_lie[6]) < 1e-12

    point_3d = np.array([1.5, 2.2, 8.0])
    depth, expected_pixel = camera.ProjectPoint(np.append(point_3d, 1.0))
    assert depth > 0.0

    pixel = _project_with_sim3(sim3_lie, camera, point_3d)
    np.testing.assert_allclose(pixel, expected_pixel, atol=1e-8)


def test_sim3_pose_consistency():
    """RelativeSim3BetweenViews equals S_i^{-1} * S_j."""
    recon = pt.sfm.Reconstruction()
    view1_id = _add_view(recon, "view1", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    view2_id = _add_view(recon, "view2", [1.0, 0.2, -0.3], [0.05, 0.1, -0.02])
    view1 = recon.View(view1_id)
    view2 = recon.View(view2_id)

    # Relative pose of a view to itself is the identity.
    np.testing.assert_allclose(
        pt.sfm.RelativeSim3BetweenViews(view1, view1), np.zeros(7), atol=1e-12)

    S1 = pt.sfm.Sim3ToHomogeneousMatrix(pt.sfm.GetSim3LieFromView(view1))
    S2 = pt.sfm.Sim3ToHomogeneousMatrix(pt.sfm.GetSim3LieFromView(view2))
    S12 = pt.sfm.Sim3ToHomogeneousMatrix(
        pt.sfm.RelativeSim3BetweenViews(view1, view2))
    np.testing.assert_allclose(S12, np.linalg.inv(S1) @ S2, atol=1e-10)

    assert not np.allclose(pt.sfm.GetSim3LieFromView(view1),
                           pt.sfm.GetSim3LieFromView(view2))


@pytest.mark.parametrize("scale", [0.5, 1.0, 1.5, 2.0])
def test_sim3_scale_handling(scale):
    """Rotation, translation and scale survive a round trip through Sim(3)."""
    angle = 0.3
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                         [np.sin(angle), np.cos(angle), 0.0],
                         [0.0, 0.0, 1.0]])
    translation = np.array([0.1, -0.4, 2.0])

    sim3_lie = pt.sfm.Sim3FromRotationTranslationScale(
        rotation, translation, scale)
    np.testing.assert_allclose(sim3_lie[6], np.log(scale), atol=1e-12)

    R, t, s = pt.sfm.Sim3ToRotationTranslationScale(sim3_lie)
    np.testing.assert_allclose(R, rotation, atol=1e-10)
    np.testing.assert_allclose(t, translation, atol=1e-10)
    assert s == pytest.approx(scale)

    T = pt.sfm.Sim3ToHomogeneousMatrix(sim3_lie)
    np.testing.assert_allclose(T[:3, :3], scale * rotation, atol=1e-10)
    np.testing.assert_allclose(T[:3, 3], translation, atol=1e-10)


def test_sim3_with_3d_points():
    """Observations created from Camera.ProjectPoint are reproduced by Sim(3)."""
    recon = pt.sfm.Reconstruction()
    view_id = _add_view(recon, "test_view", [0.0, 0.0, -1.0], [0.0, 0.1, 0.0])
    camera = recon.View(view_id).Camera()

    points_3d = [
        np.array([1.0, 0.0, 2.0]),
        np.array([0.0, 1.0, 2.0]),
        np.array([-1.0, 0.0, 2.0]),
        np.array([0.0, -1.0, 2.0]),
    ]
    for point_3d in points_3d:
        track_id = recon.AddTrack()
        recon.MutableTrack(track_id).SetPoint(np.append(point_3d, 1.0))
        recon.MutableTrack(track_id).SetIsEstimated(True)
        _, pixel = camera.ProjectPoint(np.append(point_3d, 1.0))
        assert recon.AddObservation(view_id, track_id, pt.sfm.Feature(pixel))

    assert len(recon.TrackIds()) == len(points_3d)

    view = recon.View(view_id)
    sim3_lie = pt.sfm.GetSim3LieFromView(view)
    for track_id in recon.TrackIds():
        feature = view.GetFeature(track_id)
        assert feature is not None
        point_3d = recon.Track(track_id).Point()[:3]
        pixel = _project_with_sim3(sim3_lie, camera, point_3d)
        np.testing.assert_allclose(pixel, feature.point, atol=1e-8)


if __name__ == "__main__":
    pytest.main([__file__])
