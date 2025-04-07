"""Tests for read_calibration functionality."""
import pytest
import pytheia as pt
import numpy as np
import tempfile
import os


def test_parseintrinsicpriorsfromjsonstr():
    """
    Python implementation of C++ test: ReadCalibrationTest.ParseIntrinsicPriorsFromJsonStr
    
    Tests parsing camera intrinsics priors from a JSON string.
    """
    # Create a calibration JSON string similar to the C++ test
    camera_intrinsics_priors_json = """
    {"priors" : [
        {"CameraIntrinsicsPrior": {
          "image_name" : "view_1.jpg",
          "focal_length" : 300,
          "width" : 480,
          "height" : 480,
          "principal_point" : [240, 240],
          "aspect_ratio" : 1.0,
          "skew" : 0.0,
          "radial_distortion_coeffs" : [0.1, 0.1],
          "camera_intrinsics_type" : "PINHOLE"
        }},
        {"CameraIntrinsicsPrior": {
          "image_name" : "view_2.jpg",
          "focal_length" : 350,
          "principal_point" : [240, 240],
          "aspect_ratio" : 1.5,
          "skew" : 0.25,
          "radial_distortion_coeffs" : [0.1],
          "camera_intrinsics_type" : "PINHOLE"
        }},
        {"CameraIntrinsicsPrior": {
          "image_name" : "view_3.jpg",
          "principal_point" : [240, 240],
          "camera_intrinsics_type" : "PINHOLE"
        }},
        {"CameraIntrinsicsPrior": {
          "image_name" : "view_4.jpg",
          "focal_length" : 300,
          "width" : 480,
          "height" : 480,
          "principal_point" : [240, 240],
          "aspect_ratio" : 1.0,
          "skew" : 0.0,
          "radial_distortion_coeffs" : [0.1, 0.1, 0.01],
          "tangential_distortion_coeffs" : [0.05, 0.05],
          "orientation" : [0.1, 0.1, 0.1],
          "position" : [1, 2.0, -3.0],
          "latitude" : 128.0,
          "longitude" : 256.0,
          "altitude" : 512.0,
          "camera_intrinsics_type" : "PINHOLE_RADIAL_TANGENTIAL"
        }}
    ]}
    """
    
    # Parse the JSON string
    view_to_prior = {}
    result = pt.io.ExtractCameraIntrinsicPriorsFromJson(camera_intrinsics_priors_json, view_to_prior)
    assert result is True
    
    # Check first camera
    assert "view_1.jpg" in view_to_prior
    prior1 = view_to_prior["view_1.jpg"]
    assert prior1.principal_point.is_set
    assert abs(prior1.image_width / 2.0 - prior1.principal_point.value[0]) < 1e-6
    assert abs(prior1.image_height / 2.0 - prior1.principal_point.value[1]) < 1e-6
    assert prior1.focal_length.is_set
    assert abs(prior1.focal_length.value[0] - 300) < 1e-6
    assert prior1.aspect_ratio.is_set
    assert abs(prior1.aspect_ratio.value[0] - 1.0) < 1e-6
    assert prior1.skew.is_set
    assert abs(prior1.skew.value[0] - 0.0) < 1e-6
    assert prior1.radial_distortion.is_set
    assert abs(prior1.radial_distortion.value[0] - 0.1) < 1e-6
    assert abs(prior1.radial_distortion.value[1] - 0.1) < 1e-6
    assert prior1.camera_intrinsics_model_type == "PINHOLE"

    # Check second camera
    assert "view_2.jpg" in view_to_prior
    prior2 = view_to_prior["view_2.jpg"]
    assert prior2.principal_point.is_set
    assert abs(prior2.image_width / 2.0 - prior2.principal_point.value[0]) < 1e-6
    assert abs(prior2.image_height / 2.0 - prior2.principal_point.value[1]) < 1e-6
    assert prior2.focal_length.is_set
    assert abs(prior2.focal_length.value[0] - 350) < 1e-6
    assert prior2.aspect_ratio.is_set
    assert abs(prior2.aspect_ratio.value[0] - 1.5) < 1e-6
    assert prior2.skew.is_set
    assert abs(prior2.skew.value[0] - 0.25) < 1e-6
    assert prior2.radial_distortion.is_set
    assert abs(prior2.radial_distortion.value[0] - 0.1) < 1e-6
    assert prior2.camera_intrinsics_model_type == "PINHOLE"

    # Check third camera
    assert "view_3.jpg" in view_to_prior
    prior3 = view_to_prior["view_3.jpg"]
    assert prior3.principal_point.is_set
    assert abs(prior3.image_width / 2.0 - prior3.principal_point.value[0]) < 1e-6
    assert abs(prior3.image_height / 2.0 - prior3.principal_point.value[1]) < 1e-6
    assert not prior3.aspect_ratio.is_set
    assert not prior3.skew.is_set
    assert not prior3.radial_distortion.is_set
    assert not prior3.focal_length.is_set
    assert prior3.camera_intrinsics_model_type == "PINHOLE"

    # Check fourth camera
    assert "view_4.jpg" in view_to_prior
    prior4 = view_to_prior["view_4.jpg"]
    assert prior4.principal_point.is_set
    assert abs(prior4.image_width / 2.0 - prior4.principal_point.value[0]) < 1e-6
    assert abs(prior4.image_height / 2.0 - prior4.principal_point.value[1]) < 1e-6
    assert prior4.focal_length.is_set
    assert abs(prior4.focal_length.value[0] - 300) < 1e-6
    assert prior4.aspect_ratio.is_set
    assert abs(prior4.aspect_ratio.value[0] - 1.0) < 1e-6
    assert prior4.skew.is_set
    assert abs(prior4.skew.value[0] - 0.0) < 1e-6
    assert prior4.radial_distortion.is_set
    assert abs(prior4.radial_distortion.value[0] - 0.1) < 1e-6
    assert abs(prior4.radial_distortion.value[1] - 0.1) < 1e-6
    assert abs(prior4.radial_distortion.value[2] - 0.01) < 1e-6
    assert prior4.tangential_distortion.is_set
    assert abs(prior4.tangential_distortion.value[0] - 0.05) < 1e-6
    assert abs(prior4.tangential_distortion.value[1] - 0.05) < 1e-6
    assert prior4.orientation.is_set
    assert abs(prior4.orientation.value[0] - 0.1) < 1e-6
    assert abs(prior4.orientation.value[1] - 0.1) < 1e-6
    assert abs(prior4.orientation.value[2] - 0.1) < 1e-6
    assert prior4.position.is_set
    assert abs(prior4.position.value[0] - 1) < 1e-6
    assert abs(prior4.position.value[1] - 2) < 1e-6
    assert abs(prior4.position.value[2] + 3.0) < 1e-6
    assert prior4.latitude.is_set
    assert abs(prior4.latitude.value[0] - 128.0) < 1e-6
    assert prior4.altitude.is_set
    assert abs(prior4.altitude.value[0] - 512.0) < 1e-6
    assert prior4.longitude.is_set
    assert abs(prior4.longitude.value[0] - 256.0) < 1e-6
    assert prior4.camera_intrinsics_model_type == "PINHOLE_RADIAL_TANGENTIAL"


def test_readcalibrationfromjsonfile():
    """
    Python implementation of C++ test: ReadCalibrationTest.ReadCalibrationFromJsonFile
    
    Tests reading camera intrinsics priors from a JSON file.
    """
    # Create a temporary JSON file with calibration data
    with tempfile.NamedTemporaryFile(mode='w+', suffix='.json', delete=False) as f:
        f.write("""
        {"priors" : [
            {"CameraIntrinsicsPrior": {
              "image_name" : "view_1.jpg",
              "focal_length" : 300,
              "width" : 480,
              "height" : 480,
              "principal_point" : [240, 240],
              "aspect_ratio" : 1.0,
              "skew" : 0.0,
              "radial_distortion_coeffs" : [0.1, 0.1],
              "camera_intrinsics_type" : "PINHOLE"
            }},
            {"CameraIntrinsicsPrior": {
              "image_name" : "view_2.jpg",
              "focal_length" : 350,
              "principal_point" : [240, 240],
              "aspect_ratio" : 1.5,
              "skew" : 0.25,
              "radial_distortion_coeffs" : [0.1],
              "camera_intrinsics_type" : "PINHOLE"
            }},
            {"CameraIntrinsicsPrior": {
              "image_name" : "view_3.jpg",
              "principal_point" : [240, 240],
              "camera_intrinsics_type" : "PINHOLE"
            }},
            {"CameraIntrinsicsPrior": {
              "image_name" : "view_4.jpg",
              "focal_length" : 300,
              "width" : 480,
              "height" : 480,
              "principal_point" : [240, 240],
              "aspect_ratio" : 1.0,
              "skew" : 0.0,
              "radial_distortion_coeffs" : [0.1, 0.1, 0.01],
              "tangential_distortion_coeffs" : [0.05, 0.05],
              "orientation" : [0.1, 0.1, 0.1],
              "position" : [1, 2.0, -3.0],
              "latitude" : 128.0,
              "longitude" : 256.0,
              "altitude" : 512.0,
              "camera_intrinsics_type" : "PINHOLE_RADIAL_TANGENTIAL"
            }}
        ]}
        """)
        calibration_file = f.name
    
    try:
        # Read the calibration file
        view_to_prior = {}
        result = pt.io.ReadCalibration(calibration_file, view_to_prior)
        assert result is True
        
        # Check that all views are present
        assert "view_1.jpg" in view_to_prior
        assert "view_2.jpg" in view_to_prior
        assert "view_3.jpg" in view_to_prior
        assert "view_4.jpg" in view_to_prior
        
        # Check first camera
        prior1 = view_to_prior["view_1.jpg"]
        assert prior1.principal_point.is_set
        assert abs(prior1.image_width / 2.0 - prior1.principal_point.value[0]) < 1e-6
        assert abs(prior1.image_height / 2.0 - prior1.principal_point.value[1]) < 1e-6
        assert prior1.focal_length.is_set
        assert abs(prior1.focal_length.value[0] - 300) < 1e-6
        assert prior1.aspect_ratio.is_set
        assert abs(prior1.aspect_ratio.value[0] - 1.0) < 1e-6
        
        # Check fourth camera to ensure all complex fields are properly read
        prior4 = view_to_prior["view_4.jpg"]
        assert prior4.tangential_distortion.is_set
        assert abs(prior4.tangential_distortion.value[0] - 0.05) < 1e-6
        assert prior4.orientation.is_set
        assert abs(prior4.orientation.value[0] - 0.1) < 1e-6
        assert prior4.position.is_set
        assert abs(prior4.position.value[0] - 1) < 1e-6
        assert prior4.latitude.is_set
        assert abs(prior4.latitude.value[0] - 128.0) < 1e-6
        assert prior4.camera_intrinsics_model_type == "PINHOLE_RADIAL_TANGENTIAL"
        
    finally:
        # Clean up the temporary file
        if os.path.exists(calibration_file):
            os.remove(calibration_file)
