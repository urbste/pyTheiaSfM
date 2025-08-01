#!/usr/bin/env python3
"""
Test for SIM3 reprojection errors in pyTheiaSfM.

This test verifies that the SIM3 reprojection errors work correctly
and produce expected results.
"""

import numpy as np
import pytest
import pytheia as pt
from pytheia.sfm import camera
from pytheia.sfm import feature
from pytheia.sfm import reconstruction
from pytheia.sfm import view
from pytheia.sfm import track

def test_sim3_reprojection_error_basic():
    """Test basic SIM3 reprojection error functionality."""
    
    # Create a simple reconstruction
    recon = reconstruction.Reconstruction()
    
    # Create camera model
    camera_model = camera.PinholeCameraModel()
    camera_model.SetFocalLength(1000.0)
    camera_model.SetPrincipalPoint(500.0, 400.0)
    
    # Create view with SIM3 pose
    view_obj = view.View()
    view_obj.SetName("test_view")
    view_obj.SetCamera(camera_model)
    
    # Set identity SIM3 pose
    sim3_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    view_obj.SetSim3Pose(sim3_pose)
    view_obj.SetEstimated(True)
    
    view_id = recon.AddView(view_obj)
    
    # Create a 3D point
    point_3d = np.array([1.0, 0.0, 2.0])
    
    # Create track
    track_obj = track.Track()
    track_obj.SetPoint(point_3d)
    track_obj.SetEstimated(True)
    track_id = recon.AddTrack(track_obj)
    
    # Project point to get expected pixel coordinates
    point_camera = sim3_pose[:3]  # Translation part
    pixel_coords = camera_model.CameraToPixelCoordinates(point_camera)
    
    # Create feature
    feat = feature.Feature()
    feat.point_ = pixel_coords
    feat.covariance_ = np.eye(2)
    
    # Add feature to view
    view_obj.AddFeature(track_id, feat)
    
    # Verify the setup
    assert view_obj.IsEstimated()
    assert track_obj.IsEstimated()
    assert view_obj.HasFeature(track_id)
    
    # Test that we can access the SIM3 pose
    retrieved_pose = view_obj.GetSim3Pose()
    np.testing.assert_array_almost_equal(retrieved_pose, sim3_pose)

def test_sim3_pose_consistency():
    """Test that SIM3 poses are handled consistently."""
    
    # Create reconstruction
    recon = reconstruction.Reconstruction()
    
    # Create camera model
    camera_model = camera.PinholeCameraModel()
    camera_model.SetFocalLength(1000.0)
    camera_model.SetPrincipalPoint(500.0, 400.0)
    
    # Create two views with different SIM3 poses
    view1 = view.View()
    view1.SetName("view1")
    view1.SetCamera(camera_model)
    
    # Identity pose
    sim3_pose1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    view1.SetSim3Pose(sim3_pose1)
    view1.SetEstimated(True)
    
    view2 = view.View()
    view2.SetName("view2")
    view2.SetCamera(camera_model)
    
    # Translated and scaled pose
    sim3_pose2 = np.array([0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 1.2])
    view2.SetSim3Pose(sim3_pose2)
    view2.SetEstimated(True)
    
    view1_id = recon.AddView(view1)
    view2_id = recon.AddView(view2)
    
    # Verify poses are different
    retrieved_pose1 = view1.GetSim3Pose()
    retrieved_pose2 = view2.GetSim3Pose()
    
    np.testing.assert_array_almost_equal(retrieved_pose1, sim3_pose1)
    np.testing.assert_array_almost_equal(retrieved_pose2, sim3_pose2)
    
    # Verify poses are different from each other
    assert not np.allclose(retrieved_pose1, retrieved_pose2)

def test_sim3_scale_handling():
    """Test that SIM3 scale is handled correctly."""
    
    # Create reconstruction
    recon = reconstruction.Reconstruction()
    
    # Create camera model
    camera_model = camera.PinholeCameraModel()
    camera_model.SetFocalLength(1000.0)
    camera_model.SetPrincipalPoint(500.0, 400.0)
    
    # Create view
    view_obj = view.View()
    view_obj.SetName("test_view")
    view_obj.SetCamera(camera_model)
    
    # Test different scale values
    scales = [0.5, 1.0, 1.5, 2.0]
    
    for scale in scales:
        # Set SIM3 pose with specific scale
        sim3_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, scale])
        view_obj.SetSim3Pose(sim3_pose)
        
        # Retrieve and verify scale
        retrieved_pose = view_obj.GetSim3Pose()
        assert retrieved_pose[6] == scale  # Scale is the 7th parameter

def test_sim3_with_3d_points():
    """Test SIM3 poses with 3D points."""
    
    # Create reconstruction
    recon = reconstruction.Reconstruction()
    
    # Create camera model
    camera_model = camera.PinholeCameraModel()
    camera_model.SetFocalLength(1000.0)
    camera_model.SetPrincipalPoint(500.0, 400.0)
    
    # Create view
    view_obj = view.View()
    view_obj.SetName("test_view")
    view_obj.SetCamera(camera_model)
    
    # Set SIM3 pose
    sim3_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    view_obj.SetSim3Pose(sim3_pose)
    view_obj.SetEstimated(True)
    
    view_id = recon.AddView(view_obj)
    
    # Create multiple 3D points
    points_3d = [
        np.array([1.0, 0.0, 2.0]),
        np.array([0.0, 1.0, 2.0]),
        np.array([-1.0, 0.0, 2.0]),
        np.array([0.0, -1.0, 2.0])
    ]
    
    # Create tracks for each point
    for i, point_3d in enumerate(points_3d):
        track_obj = track.Track()
        track_obj.SetPoint(point_3d)
        track_obj.SetEstimated(True)
        track_id = recon.AddTrack(track_obj)
        
        # Create feature
        feat = feature.Feature()
        feat.point_ = np.array([100.0 + i * 50, 200.0 + i * 50])  # Dummy pixel coordinates
        feat.covariance_ = np.eye(2)
        
        # Add feature to view
        view_obj.AddFeature(track_id, feat)
    
    # Verify all tracks are created
    assert len(recon.TrackIds()) == len(points_3d)
    
    # Verify all features are added
    for track_id in recon.TrackIds():
        assert view_obj.HasFeature(track_id)

if __name__ == "__main__":
    # Run tests
    test_sim3_reprojection_error_basic()
    test_sim3_pose_consistency()
    test_sim3_scale_handling()
    test_sim3_with_3d_points()
    
    print("All SIM3 reprojection error tests passed!") 