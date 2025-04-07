"""Global pytest configuration and fixtures."""
import os
import pytest
import numpy as np
import pytheia as pt

@pytest.fixture
def test_data_path():
    """Return the path to the test data directory."""
    return os.path.join(os.path.dirname(os.path.dirname(__file__)), "data")

@pytest.fixture
def small_reconstruction():
    """Create a small reconstruction with a few views and tracks."""
    recon = pt.sfm.Reconstruction()
    
    # Add a few views
    view_id1 = recon.AddView("view1.jpg", 0)
    view_id2 = recon.AddView("view2.jpg", 1)
    
    # Set camera parameters
    camera1 = recon.MutableView(view_id1).MutableCamera()
    camera1.SetCameraIntrinsicsModelType(pt.sfm.CameraIntrinsicsModelType.PINHOLE)
    camera1.SetFocalLength(1000)
    
    camera2 = recon.MutableView(view_id2).MutableCamera()
    camera2.SetCameraIntrinsicsModelType(pt.sfm.CameraIntrinsicsModelType.PINHOLE)
    camera2.SetFocalLength(1000)
    
    # Set views as estimated
    recon.MutableView(view_id1).SetIsEstimated(True)
    recon.MutableView(view_id2).SetIsEstimated(True)
    
    # Add a track
    track_id = recon.AddTrack()
    track = recon.MutableTrack(track_id)
    track.AddView(view_id1)
    track.AddView(view_id2)
    
    # Set 3D point
    track.SetPoint(np.array([1.0, 2.0, 3.0]))
    track.SetIsEstimated(True)
    
    return recon

@pytest.fixture
def temp_directory(tmpdir_factory):
    """Create a temporary directory for test outputs."""
    return tmpdir_factory.mktemp("test_output")