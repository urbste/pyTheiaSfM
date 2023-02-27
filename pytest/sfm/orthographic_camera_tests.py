import pytheia as pt
import numpy as np
from test_utils import ReprojectionOrthographic


def test_InternalParameterGetterandSetter():

    camera = pt.sfm.OrthographicCameraModel()
    assert camera.Type() == pt.sfm.CameraIntrinsicsModelType.ORTHOGRAPHIC

    # Check that default values are set
    assert camera.FocalLength() == 1.0
    assert camera.AspectRatio() == 1.0
    assert camera.Skew() == 0.0
    assert camera.PrincipalPointX() == 0.0
    assert camera.PrincipalPointY() == 0.0
    assert camera.RadialDistortion1() == 0.0
    assert camera.RadialDistortion2() == 0.0

    # Set parameters to different values.
    camera.SetFocalLength(50000.0)
    camera.SetAspectRatio(0.9)
    camera.SetSkew(0.01)
    camera.SetPrincipalPoint(300.0, 400.0)
    camera.SetRadialDistortion(0.01, 0.001)

    # Check the values are updated
    assert camera.FocalLength() == 50000.0
    assert camera.AspectRatio() == 0.9
    assert camera.Skew() == 0.01
    assert camera.PrincipalPointX() == 300.0
    assert camera.PrincipalPointY() == 400.0
    assert camera.RadialDistortion1() == 0.01
    assert camera.RadialDistortion2() == 0.001


# Test to ensure that the camera intrinsics are being set appropriately.
def SetFromCameraintrinsicsPrior(prior):
    default_camera = pt.sfm.OrthographicCameraModel()
    camera = pt.sfm.OrthographicCameraModel()
    camera.SetFromCameraIntrinsicsPriors(prior)

    if(prior.focal_length.is_set):
        assert camera.FocalLength() == prior.focal_length.value[0]
    else:
        assert camera.FocalLength() == default_camera.FocalLength()

    if(prior.aspect_ratio.is_set):
        assert camera.AspectRatio() == prior.aspect_ratio.value[0]
    else:
        assert camera.AspectRatio() == default_camera.AspectRatio()

    if(prior.skew.is_set):
        assert camera.Skew() == prior.skew.value[0]
    else:
        assert camera.Skew() == default_camera.Skew()

    if(prior.principal_point.is_set):
        assert camera.PrincipalPointX() == prior.principal_point.value[0]
        assert camera.PrincipalPointY() == prior.principal_point.value[1]
    else:
        assert camera.PrincipalPointX() == default_camera.PrincipalPointX()
        assert camera.PrincipalPointY() == default_camera.PrincipalPointY()

    if(prior.radial_distortion.is_set):
        assert camera.RadialDistortion1() == prior.radial_distortion.value[0]
        assert camera.RadialDistortion2() == prior.radial_distortion.value[1]
    else:
        assert camera.RadialDistortion1() == default_camera.RadialDistortion1()
        assert camera.RadialDistortion2() == default_camera.RadialDistortion2()


'''
Gradually add one prior at a time and ensure that the method still works. We
test before and after setting the "is_set" member variable to true to ensure
that setting the value of priors when is_set=false is handled properly.
'''
def test_SetFromCameraIntrinsicsPriors():
    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = np.array([50000.0])
    prior.principal_point.value = np.array([400.0, 300.0])
    prior.aspect_ratio.value[0] = np.array([1.01])
    prior.skew.value[0] = np.array([0.01])
    prior.radial_distortion.value = np.array([0.01, 0.001, 0, 0])

    SetFromCameraintrinsicsPrior(prior)

    prior.focal_length.is_set = True
    SetFromCameraintrinsicsPrior(prior)

    prior.principal_point.is_set = True
    SetFromCameraintrinsicsPrior(prior)

    prior.aspect_ratio.is_set = True
    SetFromCameraintrinsicsPrior(prior)

    prior.skew.is_set = True
    SetFromCameraintrinsicsPrior(prior)

    prior.radial_distortion.is_set = True
    SetFromCameraintrinsicsPrior(prior)

def test_reprejection_nodistortion():

    kPrincipalPoint = np.array([600.0, 400.0])
    kFocalLength = 50000.
    camera = pt.sfm.OrthographicCameraModel()
    camera.SetFocalLength(kFocalLength)
    camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1])
    camera.SetRadialDistortion(0, 0)
    ReprojectionOrthographic(camera)

def test_reprejection_onedistortion():
    kPrincipalPoint = np.array([600.0, 400.0])
    kFocalLength = 50000.
    camera = pt.sfm.OrthographicCameraModel()
    camera.SetFocalLength(kFocalLength)
    camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1])
    camera.SetRadialDistortion(0.01, 0)
    ReprojectionOrthographic(camera)

def test_reprejection_twodistortion():
    kPrincipalPoint = np.array([600.0, 400.0])
    kFocalLength = 50000.
    camera = pt.sfm.OrthographicCameraModel()
    camera.SetFocalLength(kFocalLength)
    camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1])
    camera.SetRadialDistortion(0.01, 0.001)
    ReprojectionOrthographic(camera)

if __name__ == "__main__":
    test_InternalParameterGetterandSetter()
    test_SetFromCameraIntrinsicsPriors()
    test_reprejection_nodistortion()
    test_reprejection_onedistortion()
    test_reprejection_twodistortion()
    print("All test for the orthographic camera passed.")

