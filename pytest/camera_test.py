import sys

from pytheia.pytheia.sfm import CameraIntrinsicsModel 
from pytheia.pytheia.sfm import Camera
from pytheia.pytheia.sfm import PinholeCameraModel
from pytheia.pytheia.sfm import CameraIntrinsicsModelType
from pytheia.pytheia.sfm import CameraIntrinsicsPrior
from scipy.spatial.transform import Rotation as R
import numpy as np



def test_SetCameraIntrinsicsModelType():
    print("test_SetCameraIntrinsicsModelType()")

    camera = Camera()
    kFocalLength = 100.0
    print(camera.GetCameraIntrinsicsModelType())
    assert camera.GetCameraIntrinsicsModelType() == CameraIntrinsicsModelType.PINHOLE
    camera.FocalLength = kFocalLength   
    camera.SetCameraIntrinsicsModelType(CameraIntrinsicsModelType.PINHOLE)
    assert camera.GetCameraIntrinsicsModelType() == CameraIntrinsicsModelType.PINHOLE



def test_SetFromCameraIntrinsicsPrior():
    print("test_SetFromCameraIntrinsicsPrior()")

    camera = Camera()
    prior = CameraIntrinsicsPrior()
    prior.image_width = 1920
    prior.image_height = 1080
    camera.SetFromCameraIntrinsicsPriors(prior)
    assert camera.GetCameraIntrinsicsModelType()==CameraIntrinsicsModelType.PINHOLE
    assert camera.ImageHeight==prior.image_height
    assert camera.ImageWidth==prior.image_width

    #Set the prior for intrinsics model to Pinhole.
    prior.camera_intrinsics_model_type = "PINHOLE"
    camera.SetFromCameraIntrinsicsPriors(prior)
    assert camera.GetCameraIntrinsicsModelType()==CameraIntrinsicsModelType.PINHOLE
    

def test_ExternalParameterGettersandSetters():
    print("test_ExternalParameterGettersandSetters()")
    camera = Camera()
    kTolerance = 1e-15

    #Check that the default values are set
    assert np.linalg.norm(camera.Position) ==0.0
    assert np.linalg.norm(camera.GetOrientationAsAngleAxis()) ==0.0
    assert np.linalg.norm(camera.GetOrientationAsRotationMatrix()-np.eye(3)) ==0.0

    #Check that position getter/setters work.
    camera.Position = np.ones(3)
    assert np.linalg.norm(camera.Position - np.ones(3))==0.0

    #Check that angle axis getter/setters work.
    gt_angle_axis = np.array([1.0, 1.0, 1.0])
    gt_angle_axis = np.array([0.3, 0.7, 0.4])

    
    r = R.from_rotvec(gt_angle_axis)
    gt_rotation_matrix = r.as_matrix()
    #ceres::AngleAxisToRotationMatrix(gt_angle_axis.data(), gt_rotation_matrix.data())
    camera.SetOrientationFromRotationMatrix(gt_rotation_matrix)
    assert np.linalg.norm(camera.GetOrientationAsAngleAxis() - gt_angle_axis)<kTolerance
    assert np.linalg.norm(camera.GetOrientationAsRotationMatrix() - gt_rotation_matrix)<kTolerance

    #Check that rotation matrix getter/setters work.
    gt_angle_axis = np.array([0.3, 0.7, 0.4])
    r = R.from_rotvec(gt_angle_axis)
    gt_rotation_matrix = r.as_matrix()
    #ceres::AngleAxisToRotationMatrix(gt_angle_axis.data(), gt_rotation_matrix.data())
    camera.SetOrientationFromRotationMatrix(gt_rotation_matrix)
    assert np.linalg.norm(camera.GetOrientationAsAngleAxis() - gt_angle_axis)<kTolerance
    assert np.linalg.norm(camera.GetOrientationAsRotationMatrix() - gt_rotation_matrix)<kTolerance


def test_InternalParameterGettersandSetters():
    print("test_InternalParameterGettersandSetters()")

    camera = Camera()
    assert camera.FocalLength==1.0
    assert camera.PrincipalPointX==0.0
    assert camera.PrincipalPointY==0.0
    assert camera.GetCameraIntrinsicsModelType()==CameraIntrinsicsModelType.PINHOLE
    
    #Make sure the default intrinsics are sets for pinhole cameras
    pinhole_intrinsics = PinholeCameraModel()
    assert pinhole_intrinsics.GetParameter(0)==camera.FocalLength
    assert pinhole_intrinsics.GetParameter(1)==camera.CameraIntrinsics().AspectRatio
    assert pinhole_intrinsics.GetParameter(2)==camera.CameraIntrinsics().Skew
    assert pinhole_intrinsics.GetParameter(3)==camera.PrincipalPointX
    assert pinhole_intrinsics.GetParameter(4)==camera.PrincipalPointY
    assert pinhole_intrinsics.GetParameter(5)==camera.CameraIntrinsics().RadialDistortion1
    assert pinhole_intrinsics.GetParameter(6)==camera.CameraIntrinsics().RadialDistortion2


    #Set parameters to different values.
    camera.FocalLength=600.0
    camera.SetPrincipalPoint(300.0, 400.0)

    assert camera.FocalLength==600.0
    assert camera.PrincipalPointX==300.0
    assert camera.PrincipalPointY==400.0

"""
def Test_ProjectionMatrix():
    pass
def Test_Reprojection():
    pass
"""

if __name__ == "__main__":
    test_ExternalParameterGettersandSetters()
    test_InternalParameterGettersandSetters()
    test_SetCameraIntrinsicsModelType()
    test_SetFromCameraIntrinsicsPrior()
    print("All tests successfull.")
