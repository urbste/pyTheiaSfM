
import pytheia as pt
import numpy as np
from noise_utils import add_noise_to_point
from scipy.spatial.transform import Rotation as R


def perspective_division(points):
    return np.divide(points.T, points[:, 2])[0:2, :].T


def generate_image_points(points3d,
                          projection_noise_std,
                          expected_rotation,
                          expected_translation):

    image_points1 = perspective_division(points3d)
    rotated_points_3d = expected_rotation @ points3d.T + \
        np.expand_dims(expected_translation, 1)
    image_points2 = perspective_division(rotated_points_3d.T)

    image_points1 = add_noise_to_point(image_points1, projection_noise_std)
    image_points2 = add_noise_to_point(image_points2, projection_noise_std)

    return image_points1, image_points2


def test_8ptFundamentalMatrix(pts0, pts1):
    success, F = pt.sfm.NormalizedEightPointFundamentalMatrix(pts0, pts1)
    assert success


def test_5ptEssentialMatrix(image_pts_1, image_pts_2, t_gt, max_error_deg):

    success, Es = pt.sfm.FivePointRelativePose(
        image_pts_1[:5, :], image_pts_2[:5, :])

    normalized_corrs = []
    for i in range(image_pts_1.shape[0]):
        normalized_corrs.append(pt.matching.FeatureCorrespondence(
            pt.sfm.Feature(image_pts_1[i, :]), pt.sfm.Feature(image_pts_2[i, :])))

    norm_t_gt = t_gt / np.linalg.norm(t_gt)
    # find closest to GT
    min_t_dist = 100000
    sol_idx = 0
    for sol in range(len(Es)):
        pose_res = pt.sfm.GetBestPoseFromEssentialMatrix(
            Es[sol], normalized_corrs)
        translation_est = -pose_res[1] @ pose_res[2]
        translation_est /= np.linalg.norm(translation_est)
        t_dist = np.arccos(np.dot(translation_est, norm_t_gt))
        if t_dist < min_t_dist:
            min_t_dist = t_dist
            sol_idx = sol
    pose_res = pt.sfm.GetBestPoseFromEssentialMatrix(
        Es[sol_idx], normalized_corrs)
    translation_est = -pose_res[1] @ pose_res[2]
    t_angle = np.arccos(np.dot(translation_est, norm_t_gt)) * 180. / np.pi
    r_dist = np.linalg.norm(R.from_matrix(pose_res[1].T @ R_gt).as_rotvec())
    assert success
    assert t_angle < max_error_deg
    assert r_dist < max_error_deg


def test_EstimateRelativeOrientation(image_pts_1, image_pts_2, R_gt, t_gt, max_error_deg):
    params = pt.solvers.RansacParameters()
    params.error_thresh = 1e-4
    params.max_iterations = 20
    params.min_iterations = 1

    # create normalized correspondences
    normalized_corrs = []
    for i in range(image_pts_1.shape[0]):
        normalized_corrs.append(pt.matching.FeatureCorrespondence(
            pt.sfm.Feature(image_pts_1[i, :]), pt.sfm.Feature(image_pts_2[i, :])))

    success, rel_ori, ransac_sum = pt.sfm.EstimateRelativePose(
        params, pt.sfm.RansacType(0), normalized_corrs)
    r_dist = np.linalg.norm(R.from_matrix(
        rel_ori.rotation.T @ R_gt).as_rotvec())
    print(r_dist)
    assert success
    assert r_dist < max_error_deg


if __name__ == "__main__":

    points_3d = np.array([[-1.0, 3.0, 3.0],
                          [-1.0, 1.0, 3.0],
                          [5.0, 2.0, 1.0],
                          [-1.0, 1.0, 2.0],
                          [2.0, 1.0, 3.0],
                          [-1.0, -3.0, 2.0],
                          [1.0, -2.0, 1.0],
                          [-2.0, 2.0, 3.0]], dtype=np.float32)

    R_gt = R.from_rotvec(13.0 * np.pi / 180
                         * np.array([0., 0., 1.])).as_matrix()
    t_gt = np.array([1.0, 0.5, 0.2])
    noise = 0.0 / 512.
    # without noise
    image_points_1, image_points_2 = generate_image_points(
        points_3d, noise, R_gt, t_gt)

    # check fundamtenal matrix estimation
    test_8ptFundamentalMatrix(image_points_1, image_points_2)
    # check essential matrix estimation
    test_5ptEssentialMatrix(
        image_points_1, image_points_2, t_gt, max_error_deg=1e-4)
    # test relative orientation function
    test_EstimateRelativeOrientation(
        image_points_1, image_points_2, R_gt, t_gt, max_error_deg=1e-4)

    # with noise
    noise = 1.0 / 512.
    image_points_1, image_points_2 = generate_image_points(
        points_3d, noise, R_gt, t_gt)

    # check fundamtenal matrix estimation
    test_8ptFundamentalMatrix(image_points_1, image_points_2)
    # check essential matrix estimation
    test_5ptEssentialMatrix(
        image_points_1, image_points_2, t_gt, max_error_deg=5.)
    # test relative orientation function
    test_EstimateRelativeOrientation(
        image_points_1, image_points_2, R_gt, t_gt, max_error_deg=5.)
