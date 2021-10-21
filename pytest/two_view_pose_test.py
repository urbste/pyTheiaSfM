import pytheia as pt
import numpy as np
from random_recon_gen import RandomReconGenerator
from random_recon_gen import CameraPrior

class GroundTruthRelPose:
    def __init__(self, cam0, cam1):
        
        self.cam0 = cam0
        self.cam1 = cam1

        self.R_rel_gt = cam1.GetOrientationAsRotationMatrix() * cam0.GetOrientationAsRotationMatrix().T
        self.t_rel_gt =  cam0.GetOrientationAsRotationMatrix() @ (cam1.Position - cam0.Position)
        self.t_rel_gt /= np.linalg.norm(self.t_rel_gt)

def test_8ptFundamentalMatrix(img_pts0, img_pts1, gt_pose):

    success, F = pt.sfm.NormalizedEightPointFundamentalMatrix(img_pts0, img_pts1)
    print(F)
    # do some decompositions
    success_f, f0, f1 = pt.sfm.FocalLengthsFromFundamentalMatrix(F)
    # focal_length = pt.sfm.SharedFocalLengthsFromFundamentalMatrix(F)
    # proj_mats = pt.sfm.ProjectionMatricesFromFundamentalMatrix(F)
    assert success
    assert success_f
    assert np.linalg.norm(f0 - gt_pose.cam0.FocalLength) < 1e-4
    assert np.linalg.norm(f1 - gt_pose.cam1.FocalLength) < 1e-4


def test_7ptFundamentalMatrix(img_pts0, img_pts1, gt_pose):

    success, Fs = pt.sfm.SevenPointFundamentalMatrix(img_pts0[:7], img_pts1[:7])
    # do some decompositions
    min_f_dist = 10000000.
    F = None
    for i in range(len(Fs)):
        _, f0, f1 = pt.sfm.FocalLengthsFromFundamentalMatrix(Fs[i])
        print(pt.sfm.FocalLengthsFromFundamentalMatrix(Fs[i]))

        dist = np.linalg.norm(f0 - gt_pose.cam0.FocalLength)
        if dist < min_f_dist:
            min_f_dist = dist
            F = Fs[i]
    print(F)
    print(pt.sfm.FocalLengthsFromFundamentalMatrix(F))
    success_f, f0, f1 = pt.sfm.FocalLengthsFromFundamentalMatrix(F)
    print(np.linalg.norm(f0 - gt_pose.cam0.FocalLength))
    assert success
    assert success_f
    assert np.linalg.norm(f0 - gt_pose.cam0.FocalLength) < 1e-4
    assert np.linalg.norm(f1 - gt_pose.cam1.FocalLength) < 1e-4


def test_5ptEssentialMatrix(img_pts0, img_pts1, gt_pose):

    success, Es = pt.sfm.FivePointRelativePose(img_pts0, img_pts1)

    normalized_corr = []
    for p0, p1 in zip(img_pts0, img_pts1):
        normalized_corr.append(pt.sfm.Feature(p0), pt.sfm.Feature(p1))

    for sol in range(len(Es)):
        print(pt.sfm.GetBestPoseFromEssentialMatrix(Es[sol],normalized_corr))


    
#def test_PositionFromTwoRays():


def test_RelativePoseFromTwoPointsWithKnownRotation(R0, R1, img_pts0, img_pts1, gt_pose):

    unrotated_norm_img_pts0 = []
    unrotated_norm_img_pts1 = []
    for p0, p1 in zip(img_pts0, img_pts1):
        unrotated_norm_img_pts0.append(R0.T @ np.extend(p0,1))
        unrotated_norm_img_pts1.append(R1.T @ np.extend(p1,1))
    
    success, position2 = pt.sfm.RelativePoseFromTwoPointsWithKnownRotation(
        unrotated_norm_img_pts0, unrotated_norm_img_pts1)

# def test_FourPointRelativePosePartialRotation():


# def test_ThreePointRelativePosePartialRotation():


# def test_TwoPointPosePartialRotation():




if __name__ == "__main__":
    pinhole_cam = CameraPrior(500.,1.0, (1000,1000))
    gen = RandomReconGenerator(cam_prior=pinhole_cam)

    # generate a random scene
    gen.generate_random_recon(nr_views=2, nr_tracks=10)

    # get ground truth relative orientation and get observations
    view0 = gen.recon.View(0)
    view1 = gen.recon.View(1)
    cam0 = view0.Camera()
    cam1 = view1.Camera()
    principal_point0 = np.array([cam0.PrincipalPointX, cam0.PrincipalPointY])
    principal_point1 = np.array([cam1.PrincipalPointX, cam1.PrincipalPointY])

    # get observations visible in both cameras
    img_pts0 = []
    img_pts0_norm = []
    img_pts1 = []
    img_pts1_norm = []
    normalized_corr = []
    for t_id in gen.recon.TrackIds:
        feat1 = view0.GetFeature(t_id)
        feat2 = view1.GetFeature(t_id)
        if not feat1 or not feat2:
            continue
        img_pts0.append(feat1.point) # remove pp to recover focal lenghts
        img_pts1.append(feat2.point)
        img_pt0_norm = cam0.PixelToNormalizedCoordinates(feat1.point)[:2]
        img_pt1_norm = cam0.PixelToNormalizedCoordinates(feat1.point)[:2]
        img_pts0_norm.append(img_pt0_norm)
        img_pts1_norm.append(img_pt1_norm)
        normalized_corr.append(
            pt.matching.FeatureCorrespondence(
                pt.sfm.Feature(img_pt0_norm), pt.sfm.Feature(img_pt1_norm)))
    
    gt_pose = GroundTruthRelPose(cam0, cam1)

    #test_8ptFundamentalMatrix(img_pts0, img_pts1, gt_pose)

    test_7ptFundamentalMatrix(img_pts0, img_pts1, gt_pose)

    # F_gt = pt.sfm.ComposeFundamentalMatrix(cam0.FocalLength, 
    #     cam1.FocalLength, R_rel_gt, t_rel_gt)
    # rt = pt.sfm.NormalizedEightPointFundamentalMatrix(img_pts0, img_pts1)
    # focal_lenghts = pt.sfm.FocalLengthsFromFundamentalMatrix(rt[1])
    # focal_length = pt.sfm.SharedFocalLengthsFromFundamentalMatrix(rt[1])
    # proj_mats = pt.sfm.ProjectionMatricesFromFundamentalMatrix(rt[1])

    # # decompose
    # print("diff: ", F_gt - rt[1])
    # print(rt)
    # print(focal_lenghts)
    # print(focal_length)
    # print(proj_mats)
    # t_est = proj_mats[1][:3,3]/np.linalg.norm(proj_mats[1][:3,3])
    # print("t_estimage: ",t_est - t_rel_gt)
    # rt = pt.sfm.FivePointRelativePose(img_pts0, img_pts1)
    # for sol in range(len(rt[1])):
    #     print(pt.sfm.GetBestPoseFromEssentialMatrix(rt[1][sol],normalized_corr))


    #
    # RelativePoseFromTwoPointsWithKnownRotation
    # PositionFromTwoRays
    # SimTransformPartialRotation
    # FourPointRelativePosePartialRotation
    # ThreePointRelativePosePartialRotation
    # TwoPointPosePartialRotation


    distorted_cam = CameraPrior(500.,1.0, (1000,1000))
    distorted_cam.set_to_division_undistortion(distortion=1e-6)
    gen = RandomReconGenerator(cam_prior=distorted_cam)

# rt = pt.sfm.PoseFromThreePoints(pts2d_cam1.T, pts3d.T)
# print('PoseFromThreePoints')
# print(rt)
# rt = pt.sfm.NormalizedEightPointFundamentalMatrix(pts2d_cam1.T, pts2d_cam2.T)
# print('NormalizedEightPointFundamentalMatrix')
# print(rt)
# rt = pt.sfm.FivePointRelativePose(pts2d_cam1.T, pts2d_cam2.T)
# print('FivePointRelativePose')
# print(rt)
# rt = pt.sfm.FourPointPoseAndFocalLength(pts2d_cam1.T, pts3d.T)
# print('FourPointPoseAndFocalLength')
# print(rt)
# rt = pt.sfm.FourPointHomography(pts2d_cam1.T, pts2d_cam2.T)
# print('FourPointHomography')
# print(rt)
# rt = pt.sfm.FourPointsPoseFocalLengthRadialDistortion(pts2d_cam1.T, pts3d.T)
# print('FourPointsPoseFocalLengthRadialDistortion')
# print(rt)


# rt = pt.sfm.DlsPnp(pts2d_cam1.T, pts3d.T)
# print('DlsPnp')
# print(rt)
# rt = pt.sfm.SevenPointFundamentalMatrix(pts2d_cam1[:,0:7].T, pts2d_cam2[:,0:7].T)
# print('SevenPointFundamentalMatrix')
# print(rt)

# '''
# f1 = np.array([2,1], dtype='float64')
# f2 = np.array([4,3], dtype='float64')
# fc1 = FeatureCorrespondence(f1, f2)
# essential_matrix = np.array([[1,1,0],[1,0,1],[0,-1,1]], dtype= 'float64')
# rt = GetBestPoseFromEssentialMatrix(essential_matrix, fc1)
# print('GetBestPoseFromEssentialMatrix')
# print(rt)
# '''
# '''
# NormalizedEightPointFundamentalMatrix
# FivePointRelativePose
# FourPointPoseAndFocalLength
# FourPointsPoseFocalLengthRadialDistortion
# FourPointHomography
# DlsPnp
# SevenPointFundamentalMatrix

# RelativePoseFromTwoPointsWithKnownRotation
# PositionFromTwoRays
# SimTransformPartialRotation
# FourPointRelativePosePartialRotation
# ThreePointRelativePosePartialRotation
# TwoPointPosePartialRotation
# '''