import pytheia as pt
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R
from random_recon_gen import RandomReconGenerator
from random_recon_gen import CameraPrior


class GroundTruthRelPose:
    def __init__(self, cam0, cam1):

        self.cam0 = cam0
        self.cam1 = cam1

        R0 = cam0.GetOrientationAsRotationMatrix()
        R1 = cam1.GetOrientationAsRotationMatrix()
        self.R_rel_gt = R1 @ R0.T
        self.t_rel_gt = R0 @ (cam0.GetPosition() - cam1.GetPosition())
        self.t_rel_gt /= np.linalg.norm(self.t_rel_gt)
        self.p_rel_gt = -R0.T @ self.t_rel_gt


class TwoViewScene:
    """Two nearly fronto-parallel pinhole views observing 10 random points."""

    def __init__(self):
        pinhole_cam = CameraPrior(300., 1.0, (1000, 1000))
        gen = RandomReconGenerator(verbose=False, cam_prior=pinhole_cam)
        gen.generate_random_recon(
            nr_views=2, nr_tracks=10, cam_rot_max_angle=1e-3)

        view0 = gen.recon.View(0)
        view1 = gen.recon.View(1)
        cam0 = view0.Camera()
        cam1 = view1.Camera()

        # get observations visible in both cameras
        self.img_pts0_norm = []
        self.img_pts1_norm = []
        self.normalized_corrs = []
        for t_id in gen.recon.TrackIds():
            feat1 = view0.GetFeature(t_id)
            feat2 = view1.GetFeature(t_id)
            if not feat1 or not feat2:
                continue
            img_pt0_norm = cam0.PixelToNormalizedCoordinates(feat1.point)[:2]
            img_pt1_norm = cam1.PixelToNormalizedCoordinates(feat2.point)[:2]
            self.img_pts0_norm.append(img_pt0_norm)
            self.img_pts1_norm.append(img_pt1_norm)
            self.normalized_corrs.append(
                pt.matching.FeatureCorrespondence(
                    pt.sfm.Feature(img_pt0_norm), pt.sfm.Feature(img_pt1_norm)))

        self.gt_pose = GroundTruthRelPose(cam0, cam1)


@pytest.fixture(scope="module")
def scene():
    return TwoViewScene()


def _check_8ptFundamentalMatrix(pts0, pts1):

    success, F = pt.sfm.NormalizedEightPointFundamentalMatrix(pts0, pts1)
    # do some decompositions
    success_f, f0, f1 = pt.sfm.FocalLengthsFromFundamentalMatrix(F)
    # focal_length = pt.sfm.SharedFocalLengthsFromFundamentalMatrix(F)
    # proj_mats = pt.sfm.ProjectionMatricesFromFundamentalMatrix(F)
    assert success
    assert success_f
    assert np.linalg.norm(f0 - 1.0) < 1e-4
    assert np.linalg.norm(f1 - 1.0) < 1e-4


# def test_7ptFundamentalMatrix(pts0, pts1, gt_pose):

#     success, Fs = pt.sfm.SevenPointFundamentalMatrix(pts0[:7], pts1[:7])
#     # do some
#     min_f_dist = 10000000.
#     F = None
#     for i in range(len(Fs)):
#         _, f0, f1 = pt.sfm.FocalLengthsFromFundamentalMatrix(Fs[i])
#         print(pt.sfm.FocalLengthsFromFundamentalMatrix(Fs[i]))

#         dist = np.linalg.norm(f0 - gt_pose.cam0.FocalLength)
#         if dist < min_f_dist:
#             min_f_dist = dist
#             F = Fs[i]
#     print(F)
#     print(pt.sfm.FocalLengthsFromFundamentalMatrix(F))
#     success_f, f0, f1 = pt.sfm.FocalLengthsFromFundamentalMatrix(F)
#     print(np.linalg.norm(f0 - gt_pose.cam0.FocalLength))
#     assert success
#     assert success_f
#     assert np.linalg.norm(f0 - gt_pose.cam0.FocalLength) < 1e-4
#     assert np.linalg.norm(f1 - gt_pose.cam1.FocalLength) < 1e-4


def _check_5ptEssentialMatrix(img_pts0, img_pts1, normalized_corrs, gt_pose, max_error_deg):

    success, Es = pt.sfm.FivePointRelativePose(img_pts0[:5], img_pts1[:5])

    norm_t_gt = gt_pose.p_rel_gt / np.linalg.norm(gt_pose.p_rel_gt)
    # find closest to GT
    min_t_dist = 100000
    sol_idx = 0
    for sol in range(len(Es)):
        pose_res = pt.sfm.GetBestPoseFromEssentialMatrix(
            Es[sol], normalized_corrs)
        t_dist = np.arccos(np.dot(pose_res[2], norm_t_gt))
        if t_dist < min_t_dist:
            min_t_dist = t_dist
            sol_idx = sol
    pose_res = pt.sfm.GetBestPoseFromEssentialMatrix(
        Es[sol_idx], normalized_corrs)
    t_angle = np.arccos(np.dot(pose_res[2], norm_t_gt)) * 180. / np.pi
    assert success
    assert t_angle < max_error_deg

# def test_PositionFromTwoRays():


def _check_EstimateRelativeOrientation(normalized_corrs, gt_pose, max_error_deg):
    params = pt.solvers.RansacParameters()
    # Seed the RNG for a deterministic, reproducible test. Without this, RANSAC
    # uses a time-seeded generator, and with this scene's near-degenerate,
    # 10-point, near-zero-rotation configuration plus a tight error_thresh /
    # max_error_deg, an unlucky minimal sample can occasionally slip through
    # the (loose) inlier threshold with a poor pose estimate, making the test
    # flaky.
    params.rng = pt.solvers.RandomNumberGenerator(42)
    params.error_thresh = 1e-4
    params.max_iterations = 20
    params.min_iterations = 1

    norm_p_gt = gt_pose.p_rel_gt / np.linalg.norm(gt_pose.p_rel_gt)

    success, rel_ori, ransac_sum = pt.sfm.EstimateRelativePose(
        params, pt.sfm.RansacType(0), normalized_corrs)

    #r_dist = np.linalg.norm(cv2.Rodrigues(rel_ori.rotation.T @ gt_pose.R_rel_gt)[0])
    t_angle = np.arccos(np.dot(rel_ori.position, norm_p_gt)) * 180. / np.pi
    assert success
    assert t_angle < max_error_deg


# def test_FourPointRelativePosePartialRotation():


# def test_ThreePointRelativePosePartialRotation():


# def test_TwoPointPosePartialRotation():


def test_8ptFundamentalMatrix(scene):
    _check_8ptFundamentalMatrix(scene.img_pts0_norm, scene.img_pts1_norm)


def test_5ptEssentialMatrix(scene):
    _check_5ptEssentialMatrix(scene.img_pts0_norm, scene.img_pts1_norm,
                              scene.normalized_corrs, scene.gt_pose,
                              max_error_deg=0.1)


def test_EstimateRelativeOrientation(scene):
    _check_EstimateRelativeOrientation(
        scene.normalized_corrs, scene.gt_pose, max_error_deg=0.1)


if __name__ == "__main__":
    pytest.main([__file__])

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
