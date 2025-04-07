"""Tests for write_calibration functionality."""
import pytest
import pytheia as pt
import numpy as np
import tempfile
import os


def test_writeandparseintrinsicpriors():
    """
    Python implementation of C++ test: WriteCalibrationTest.WriteAndParseIntrinsicPriors
    
    Tests writing calibration priors to a file and reading them back.
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
    
    # Parse expected priors from the JSON string
    expected_priors = {}
    result = pt.io.ExtractCameraIntrinsicPriorsFromJson(camera_intrinsics_priors_json, expected_priors)
    assert result is True
    
    # Create a temporary file to write and read the calibration
    temp_file = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
    temp_file.close()
    calibration_file = temp_file.name
    
    try:
        # Write the calibration to the file
        write_result = pt.io.WriteCalibration(calibration_file, expected_priors)
        assert write_result is True
        
        # Read the file back
        read_priors = {}
        read_result = pt.io.ReadCalibration(calibration_file, read_priors)
        assert read_result is True
        
        # Compare the two priors dictionaries
        assert len(expected_priors) == len(read_priors)
        for image_name, prior in expected_priors.items():
            assert image_name in read_priors
            
            camera_prior = read_priors[image_name]
            
            # Check basic entries
            assert camera_prior.camera_intrinsics_model_type == prior.camera_intrinsics_model_type
            assert camera_prior.image_width == prior.image_width
            assert camera_prior.image_height == prior.image_height
            
            assert camera_prior.focal_length.is_set == prior.focal_length.is_set
            if prior.focal_length.is_set:
                assert camera_prior.focal_length.value[0] == prior.focal_length.value[0]
                
            assert camera_prior.aspect_ratio.is_set == prior.aspect_ratio.is_set
            if prior.aspect_ratio.is_set:
                assert camera_prior.aspect_ratio.value[0] == prior.aspect_ratio.value[0]
                
            assert camera_prior.skew.is_set == prior.skew.is_set
            if prior.skew.is_set:
                assert camera_prior.skew.value[0] == prior.skew.value[0]
                
            assert camera_prior.principal_point.is_set == prior.principal_point.is_set
            if prior.principal_point.is_set:
                assert camera_prior.principal_point.value[0] == prior.principal_point.value[0]
                assert camera_prior.principal_point.value[1] == prior.principal_point.value[1]
                
            assert camera_prior.radial_distortion.is_set == prior.radial_distortion.is_set
            if prior.radial_distortion.is_set:
                for i in range(min(4, len(prior.radial_distortion.value))):
                    assert camera_prior.radial_distortion.value[i] == prior.radial_distortion.value[i]
                
            assert camera_prior.tangential_distortion.is_set == prior.tangential_distortion.is_set
            if prior.tangential_distortion.is_set:
                assert camera_prior.tangential_distortion.value[0] == prior.tangential_distortion.value[0]
                assert camera_prior.tangential_distortion.value[1] == prior.tangential_distortion.value[1]
                
            assert camera_prior.position.is_set == prior.position.is_set
            if prior.position.is_set:
                assert camera_prior.position.value[0] == prior.position.value[0]
                assert camera_prior.position.value[1] == prior.position.value[1]
                assert camera_prior.position.value[2] == prior.position.value[2]
                
            assert camera_prior.orientation.is_set == prior.orientation.is_set
            if prior.orientation.is_set:
                assert camera_prior.orientation.value[0] == prior.orientation.value[0]
                assert camera_prior.orientation.value[1] == prior.orientation.value[1]
                assert camera_prior.orientation.value[2] == prior.orientation.value[2]
                
            assert camera_prior.latitude.is_set == prior.latitude.is_set
            if prior.latitude.is_set:
                assert camera_prior.latitude.value[0] == prior.latitude.value[0]
                
            assert camera_prior.longitude.is_set == prior.longitude.is_set
            if prior.longitude.is_set:
                assert camera_prior.longitude.value[0] == prior.longitude.value[0]
                
            assert camera_prior.altitude.is_set == prior.altitude.is_set
            if prior.altitude.is_set:
                assert camera_prior.altitude.value[0] == prior.altitude.value[0]
                
    finally:
        # Clean up the temporary file
        if os.path.exists(calibration_file):
            os.remove(calibration_file)
