import pytheia as pt
import numpy as np
import cv2

def create_gt_maps(cam_dist, cam_undist):

    # Initialize maps
    map_x = np.zeros((cam_dist.ImageHeight(), cam_dist.ImageWidth()), dtype=np.float32)
    map_y = np.zeros((cam_dist.ImageHeight(), cam_dist.ImageWidth()), dtype=np.float32)

    for c in range(cam_dist.ImageWidth()):
        for r in range(cam_dist.ImageHeight()):
            image_pt_undist = np.array([c, r]) 
            pt_in_undist_camera = cam_undist.CameraIntrinsics().ImageToCameraCoordinates(
                image_pt_undist)
            distorted_pt = cam_dist.CameraIntrinsics().CameraToImageCoordinates(
                pt_in_undist_camera)

            map_x[r, c] = distorted_pt[0]
            map_y[r, c] = distorted_pt[1]

    return map_x, map_y

def test_undistortion_map(camera_dist, camera_undist, img_dist, img_gt, maps_gt):
    
    maps = pt.sfm.ComputeUndistortionMap(camera_dist, camera_undist)
    mapsxy = np.asarray(maps)

    assert np.allclose(maps_gt, mapsxy, atol=1e-4)

    img_undist = cv2.remap(img_dist, mapsxy[:,:,0], mapsxy[:,:,1], 
                           cv2.INTER_LINEAR)
    
    assert np.allclose(img_gt, img_undist, atol=1e-4)

def test_undistort_reconstruction():
    pass

def test_undistort_camera(camera_dist, camera_undist_gt):

    camera_undist = pt.sfm.UndistortCamera(camera_dist, False)

    intr_undist = camera_undist.CameraIntrinsics()
    intr_undist_gt = camera_undist_gt.CameraIntrinsics()

    assert intr_undist.FocalLength() == intr_undist_gt.FocalLength()
    assert intr_undist.PrincipalPointX() == intr_undist_gt.PrincipalPointX()
    assert intr_undist.PrincipalPointY() == intr_undist_gt.PrincipalPointY()
    assert intr_undist.RadialDistortion1() == intr_undist_gt.RadialDistortion1()

    assert camera_undist.ImageWidth() == camera_undist_gt.ImageWidth()
    assert camera_undist.ImageHeight() == camera_undist_gt.ImageHeight()

if __name__ == "__main__":

    img = cv2.imread("data/image/distorted.png")

    # create the distorted camera model
    prior_dist = pt.sfm.CameraIntrinsicsPrior()
    prior_dist.focal_length.value = [485.15956467957056]
    prior_dist.aspect_ratio.value = [0.9805717422736094]
    prior_dist.principal_point.value = [478.4298, 277.5628]
    prior_dist.skew.value = [0.0]
    prior_dist.radial_distortion.value = [-1.201842919562806e-06, 0.0, 0.0, 0.0]
    prior_dist.camera_intrinsics_model_type = "DIVISION_UNDISTORTION"
    prior_dist.image_width = 960
    prior_dist.image_height = 540

    camera_dist = pt.sfm.Camera()
    camera_dist.SetFromCameraIntrinsicsPriors(prior_dist)

    # create the undistorted camera model
    prior_undist = pt.sfm.CameraIntrinsicsPrior()
    prior_undist.focal_length.value = [485.15956467957056]
    prior_undist.aspect_ratio.value = [1.0]
    prior_undist.principal_point.value = [478.4298, 277.5628]
    prior_undist.skew.value = [0.0]
    prior_undist.radial_distortion.value = [0.0, 0.0, 0.0, 0.0]
    prior_undist.camera_intrinsics_model_type = "PINHOLE"
    prior_undist.image_width = 960
    prior_undist.image_height = 540

    camera_undist = pt.sfm.Camera()
    camera_undist.SetFromCameraIntrinsicsPriors(prior_undist)

    # resize image
    img = cv2.resize(img, (camera_undist.ImageWidth(), camera_undist.ImageHeight()))
    maps_x, maps_y = create_gt_maps(camera_dist, camera_undist)
    # undistort
    img_undist_gt = cv2.remap(img, maps_x, maps_y, cv2.INTER_LINEAR)

    test_undistortion_map(camera_dist, camera_undist, img, img_undist_gt,
                          np.concatenate((maps_x[...,None], maps_y[...,None]), axis=-1))
    

    test_undistort_camera(camera_dist, camera_undist)

    camera_undist = pt.sfm.UndistortCamera(camera_dist, False)

    test_undistortion_map(camera_dist, camera_undist, img, img_undist_gt,
                          np.concatenate((maps_x[...,None], maps_y[...,None]), axis=-1))
    
    maps = pt.sfm.ComputeUndistortionMap(camera_dist, camera_undist)
    mapsxy = np.asarray(maps)
    img_new = cv2.remap(img, mapsxy[:,:,0], mapsxy[:,:,1], cv2.INTER_LINEAR)
    cv2.imshow("img_undist_gt", img_new)
    cv2.waitKey(0)
