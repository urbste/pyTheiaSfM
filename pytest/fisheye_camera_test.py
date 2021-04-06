
import sys
from pytheia.pytheia.sfm import CameraIntrinsicsModel 
from pytheia.pytheia.sfm import Camera
from pytheia.pytheia.sfm import FisheyeCameraModel
from pytheia.pytheia.sfm import CameraIntrinsicsModelType
from pytheia.pytheia.sfm import CameraIntrinsicsPrior

import numpy as np
import pytest

def test_InternalParameterGetterandSetter():

    camera = FisheyeCameraModel()
    assert camera.Type()== CameraIntrinsicsModelType.FISHEYE

    #Check that default values are set
    assert camera.FocalLength==1.0
    assert camera.AspectRatio==1.0
    assert camera.Skew==0.0
    assert camera.PrincipalPointX==0.0
    assert camera.PrincipalPointY==0.0
    assert camera.RadialDistortion1==0.0
    assert camera.RadialDistortion2==0.0
    assert camera.RadialDistortion3==0.0
    assert camera.RadialDistortion4==0.0

    #Set parameters to different values.
    camera.FocalLength=600.0
    camera.AspectRatio=0.9
    camera.Skew=0.01
    camera.SetPrincipalPoint(300.0, 400.0)
    camera.SetRadialDistortion(0.01, 0.001, 0.002, 0.003)

    #Check the values are updated
    assert camera.FocalLength==600.0
    assert camera.AspectRatio==0.9
    assert camera.Skew==0.01
    assert camera.PrincipalPointX==300.0
    assert camera.PrincipalPointY==400.0
    assert camera.RadialDistortion1==0.01
    assert camera.RadialDistortion2==0.001
    assert camera.RadialDistortion3==0.002
    assert camera.RadialDistortion4==0.003


#Test to ensure that the camera intrinsics are being set appropriately.

def SetFromCameraintrinsicsPrior(prior):
    default_camera = FisheyeCameraModel()
    camera = FisheyeCameraModel()
    camera.SetFromCameraIntrinsicsPriors(prior)
    
    if(prior.focal_length.is_set):
        assert camera.FocalLength==prior.focal_length.value[0]
    else:
        assert camera.FocalLength==default_camera.FocalLength

    if(prior.aspect_ratio.is_set):
        assert camera.AspectRatio==prior.aspect_ratio.value[0]
    else:
        assert camera.AspectRatio==default_camera.AspectRatio

    if(prior.skew.is_set):
        assert camera.Skew==prior.skew.value[0]
    else:
        assert camera.Skew==default_camera.Skew

    if(prior.principal_point.is_set):
        assert camera.PrincipalPointX==prior.principal_point.value[0]
        assert camera.PrincipalPointY==prior.principal_point.value[1]
    else:
        assert camera.PrincipalPointX==default_camera.PrincipalPointX
        assert camera.PrincipalPointY==default_camera.PrincipalPointY

    if(prior.radial_distortion.is_set):
        assert camera.RadialDistortion1==prior.radial_distortion.value[0]
        assert camera.RadialDistortion2==prior.radial_distortion.value[1]
        assert camera.RadialDistortion3==prior.radial_distortion.value[2]
        assert camera.RadialDistortion4==prior.radial_distortion.value[3]
    else:
        assert camera.RadialDistortion1==default_camera.RadialDistortion1
        assert camera.RadialDistortion2==default_camera.RadialDistortion2
        assert camera.RadialDistortion3==default_camera.RadialDistortion3
        assert camera.RadialDistortion4==default_camera.RadialDistortion4

'''
Gradually add one prior at a time and ensure that the method still works. We
test before and after setting the "is_set" member variable to true to ensure
that setting the value of priors when is_set=false is handled properly.
'''
def test_SetFromCameraIntrinsicsPriors():
  prior = CameraIntrinsicsPrior()
  prior.focal_length.value = np.array([1000.0])
  prior.principal_point.value = np.array([400.0, 300.0])
  prior.aspect_ratio.value[0] = np.array([1.01])
  prior.skew.value[0] = np.array([0.01])
  prior.radial_distortion.value = np.array([0.01, 0.001, 0.001, 0.001])

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


def Reprojection(camera):
    kTolerance = 1e-5
    kNormalizedTolerance = kTolerance / camera.FocalLength
    kImageWidth = 1200
    kImageHeight = 980
    kMinDepth = 2
    kMaxDepth = 25

    #Ensure the image -> camera -> image transformation works.
    for x in range(0, kImageWidth, 10):
        for y in range(0, kImageHeight, 10):
            pixel = np.array([x, y])
            #Get the normalized ray of that pixel.
            normalized_ray = camera.ImageToCameraCoordinates(pixel)

            #Test the reprojection at several depths.
            for depth in range(kMinDepth, kMaxDepth):
                #Convert it to a full 3D point in the camera coordinate system.
                point = normalized_ray * depth
                reprojected_pixel = camera.CameraToImageCoordinates(point)

                #Expect the reprojection to be close.
                assert np.linalg.norm(pixel - reprojected_pixel) < kTolerance
                #print( "gt pixel: " + str(pixel.T))
                #print("reprojected pixel: " + str(reprojected_pixel.T))


    #Ensure the camera -> image -> camera transformation works.
    ls = [-0.8+0.1*x for x in range(16)]
    for x in ls:
        for y in ls:
            for depth in range(kMinDepth, kMaxDepth): 
                point = np.array([x, y, depth])
                pixel = camera.CameraToImageCoordinates(point)

                #Get the normalized ray of that pixel.
                normalized_ray = camera.ImageToCameraCoordinates(pixel)

                #Convert it to a full 3D point in the camera coordinate system.
                reprojected_point = normalized_ray * depth

                #Expect the reprojection to be close.
                assert np.linalg.norm(point - reprojected_point) < kNormalizedTolerance
                #print( "gt pixel: " + str(point.T))
                #print("reprojected pixel: " + str(reprojected_point.T))   


def test_reprejection_nodistortion():

    kPrincipalPoint = np.array([600.0, 400.0])
    kFocalLength = 1200
    camera = FisheyeCameraModel()
    camera.FocalLength = kFocalLength
    camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1])
    camera.SetRadialDistortion(0, 0, 0, 0)
    Reprojection(camera)

def test_reprejection_onedistortion():
    kPrincipalPoint = np.array([600.0, 400.0])
    kFocalLength = 1200
    camera = FisheyeCameraModel()
    camera.FocalLength = kFocalLength
    camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1])
    camera.SetRadialDistortion(0.01, 0, 0, 0)
    Reprojection(camera)

def test_reprejection_twodistortion():
    kPrincipalPoint = np.array([600.0, 400.0])
    kFocalLength = 1200
    camera = FisheyeCameraModel()
    camera.FocalLength = kFocalLength
    camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1])
    camera.SetRadialDistortion(0.01, 0.001, 0, 0)
    Reprojection(camera)

def test_reprejection_threedistortion():
    kPrincipalPoint = np.array([600.0, 400.0])
    kFocalLength = 1200
    camera = FisheyeCameraModel()
    camera.FocalLength = kFocalLength
    camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1])
    camera.SetRadialDistortion(0.01, 0.001, 0.001, 0)
    Reprojection(camera)

def test_reprejection_fourdistortion():
    kPrincipalPoint = np.array([600.0, 400.0])
    kFocalLength = 1200
    camera = FisheyeCameraModel()
    camera.FocalLength = kFocalLength
    camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1])
    camera.SetRadialDistortion(0.01, 0.001, 0.001, 0.001)
    Reprojection(camera)

