"""Utility functions for testing."""
import numpy as np
import pytheia as pt

def assert_camera_poses_equal(camera1, camera2, position_tolerance=1e-8, rotation_tolerance=1e-8):
    """Assert that two cameras have the same pose within tolerance."""
    position1 = camera1.GetPosition()
    position2 = camera2.GetPosition()
    
    # Check positions
    assert np.allclose(position1, position2, atol=position_tolerance)
    
    # Check orientations (as rotation matrices)
    rotation1 = camera1.GetOrientationAsRotationMatrix()
    rotation2 = camera2.GetOrientationAsRotationMatrix()
    assert np.allclose(rotation1, rotation2, atol=rotation_tolerance)

def assert_point_clouds_equal(recon1, recon2, position_tolerance=1e-8):
    """Assert that two reconstructions have the same point cloud within tolerance."""
    tracks1 = [recon1.Track(track_id) for track_id in recon1.TrackIds()]
    tracks2 = [recon2.Track(track_id) for track_id in recon2.TrackIds()]
    
    # First check that we have the same number of tracks
    assert len(tracks1) == len(tracks2)
    
    # Check that all points in first reconstruction have a close match in second
    points1 = np.array([track.Point() for track in tracks1])
    points2 = np.array([track.Point() for track in tracks2])
    
    # For each point in points1, find the closest point in points2
    for pt1 in points1:
        distances = np.linalg.norm(points2 - pt1, axis=1)
        assert np.min(distances) < position_tolerance

def create_test_view_graph():
    """Create a simple view graph for testing."""
    view_graph = pt.sfm.ViewGraph()
    
    # Add views
    view_ids = [0, 1, 2]
    for view_id in view_ids:
        view_graph.AddView(view_id)
    
    # Create edges
    info_01 = pt.sfm.TwoViewInfo()
    info_01.focal_length_1 = 1000.0
    info_01.focal_length_2 = 1000.0
    info_01.position_2 = np.array([1.0, 0.0, 0.0])
    info_01.rotation_2 = np.identity(3)
    info_01.num_verified_matches = 100
    
    info_12 = pt.sfm.TwoViewInfo()
    info_12.focal_length_1 = 1000.0
    info_12.focal_length_2 = 1000.0
    info_12.position_2 = np.array([0.0, 1.0, 0.0])
    info_12.rotation_2 = np.identity(3)
    info_12.num_verified_matches = 120
    
    # Add edges
    view_graph.AddEdge(view_ids[0], view_ids[1], info_01)
    view_graph.AddEdge(view_ids[1], view_ids[2], info_12)
    
    return view_graph, view_ids