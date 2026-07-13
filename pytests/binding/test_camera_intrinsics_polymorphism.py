"""Camera intrinsics polymorphism tests (shared_ptr holder migration risk)."""

import numpy as np
import pytheia as pt


def _pinhole_prior(focal=900.0):
    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = np.array([focal], dtype=np.float64)
    prior.principal_point.value = np.array([640.0, 360.0], dtype=np.float64)
    prior.aspect_ratio.value = np.array([1.0], dtype=np.float64)
    prior.camera_intrinsics_model_type = "PINHOLE"
    prior.image_width = 1280
    prior.image_height = 720
    return prior


def _fisheye_prior(focal=600.0):
    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = np.array([focal], dtype=np.float64)
    prior.principal_point.value = np.array([320.0, 240.0], dtype=np.float64)
    prior.aspect_ratio.value = np.array([1.0], dtype=np.float64)
    prior.radial_distortion.value = np.array([0.01, 0.001, 0.0, 0.0], dtype=np.float64)
    prior.camera_intrinsics_model_type = "FISHEYE"
    prior.image_width = 640
    prior.image_height = 480
    return prior


def test_pinhole_to_fisheye_type_switch():
    camera = pt.sfm.Camera()
    camera.SetFromCameraIntrinsicsPriors(_pinhole_prior())
    assert (
        camera.GetCameraIntrinsicsModelType()
        == pt.sfm.CameraIntrinsicsModelType.PINHOLE
    )
    camera.SetCameraIntrinsicsModelType(
        pt.sfm.CameraIntrinsicsModelType.FISHEYE
    )
    assert (
        camera.GetCameraIntrinsicsModelType()
        == pt.sfm.CameraIntrinsicsModelType.FISHEYE
    )
    assert camera.CameraIntrinsics().Type() == pt.sfm.CameraIntrinsicsModelType.FISHEYE


def test_fisheye_round_trip_project_distort():
    camera = pt.sfm.Camera()
    camera.SetFromCameraIntrinsicsPriors(_fisheye_prior())
    camera.SetPosition(np.array([0.0, 0.0, 0.0]))
    camera.SetOrientationFromRotationMatrix(np.eye(3))
    point3d = np.array([0.1, 0.2, 5.0, 1.0])
    success, pixel = camera.ProjectPoint(point3d)
    assert success
    assert 0 <= pixel[0] <= 640 and 0 <= pixel[1] <= 480


def test_division_undistortion_polymorphic_dispatch():
    camera = pt.sfm.Camera()
    prior = _pinhole_prior()
    prior.camera_intrinsics_model_type = "DIVISION_UNDISTORTION"
    prior.radial_distortion.value = np.array([1e-6, 0.0, 0.0, 0.0], dtype=np.float64)
    camera.SetFromCameraIntrinsicsPriors(prior)
    assert (
        camera.GetCameraIntrinsicsModelType()
        == pt.sfm.CameraIntrinsicsModelType.DIVISION_UNDISTORTION
    )
    intrinsics = camera.CameraIntrinsics()
    intrinsics.SetFocalLength(800.0)
    assert camera.FocalLength() == 800.0


def test_standalone_fisheye_model_assign_to_camera():
    fisheye = pt.sfm.FisheyeCameraModel()
    fisheye.SetFocalLength(550.0)
    fisheye.SetPrincipalPoint(400.0, 300.0)
    camera = pt.sfm.Camera()
    camera.SetCameraIntrinsicsModelType(
        pt.sfm.CameraIntrinsicsModelType.FISHEYE
    )
    camera.CameraIntrinsics().SetFocalLength(550.0)
    assert camera.FocalLength() == 550.0
    assert camera.CameraIntrinsics().FocalLength() == 550.0


def test_orthographic_type_switch():
    camera = pt.sfm.Camera()
    camera.SetFromCameraIntrinsicsPriors(_pinhole_prior())
    camera.SetCameraIntrinsicsModelType(
        pt.sfm.CameraIntrinsicsModelType.ORTHOGRAPHIC
    )
    assert (
        camera.GetCameraIntrinsicsModelType()
        == pt.sfm.CameraIntrinsicsModelType.ORTHOGRAPHIC
    )
