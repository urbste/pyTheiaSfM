// Implements MORE (Multi-view Optimization with Rotation-only Reprojection
// Error), Sec. 5.2 Eq. 21-23 and Alg. 2 of Li, Cai, Wu, arXiv:2611.11770.
// Parameters not given in the paper are pyTheia choices.
//
// MORE refines the global orientations (world-to-camera rotations) of all views
// with a reprojection-like error that involves only rotations. The camera
// translations are eliminated analytically, as in the paper: for each triplet
// (i, l, j) the world-frame baselines b_il and b_ij are the closed-form ART
// solution (Theorem 1, art_position_estimator.h) written as a function of the
// rotations. That function is evaluated inside the residual, so Ceres
// differentiates through it. The smallest eigenvector is computed with a
// templated closed-form eigenvalue and a cross product of rows, and the sign is
// fixed by a reference vector. Each track observation x_jk is paired with a
// reference pair (i, l) (or with a single view i for the two-view case) and
// contributes the residual
//   eps = pi(Y) - pi(x_jk),  Y = ||t_il x x_lk|| R_ij x_ik
//                              + ||x_lk x (R_il x_ik)|| t_ij,
// together with its symmetric counterpart with i and l exchanged (Eq. 23).
//
// The outer loop (num_outer_iterations, pyTheia choice) only updates the
// discrete and robust parts: the tuple selection, the observation filter of the
// Theorem 1 tracks (as in ART), the chirality references of the eigenvectors and
// the Cauchy IRLS weights of the tuples. Within one outer iteration the
// rotations are optimized jointly by Ceres. All parameter values not given in
// the paper are pyTheia choices; the paper does not specify its optimizer.
//
// Bibliography:
//@misc{li2026rotationonly,
//  title         = {Towards Rotation-Only Imaging Geometry: Multi-View Pose Estimation},
//  author        = {Li, Xinrui and Cai, Qi and Wu, Yuanxin},
//  year          = {2026},
//  eprint        = {2611.11770},
//  archivePrefix = {arXiv},
//  primaryClass  = {cs.CV}
//}
//@article{cai2023pose,
//  title   = {A Pose-only Solution to Visual Reconstruction and Navigation},
//  author  = {Cai, Qi and Zhang, Lilian and Wu, Yuanxin and Yu, Wenxian and Hu, Dewen},
//  journal = {IEEE Transactions on Pattern Analysis and Machine Intelligence},
//  volume  = {45}, number = {1}, pages = {73--86}, year = {2023}
//}
//@inproceedings{moulon2013global,
//  title     = {Global Fusion of Relative Motions for Robust, Accurate and Scalable Structure from Motion},
//  author    = {Moulon, Pierre and Monasse, Pascal and Marlet, Renaud},
//  booktitle = {IEEE International Conference on Computer Vision (ICCV)},
//  year      = {2013}
//}

#ifndef THEIA_SFM_GLOBAL_POSE_ESTIMATION_ROTATION_ONLY_REFINER_H_
#define THEIA_SFM_GLOBAL_POSE_ESTIMATION_ROTATION_ONLY_REFINER_H_

#include <Eigen/Core>
#include <ceres/cost_function.h>
#include <unordered_map>
#include <vector>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"

namespace theia {

struct RotationOnlyRefinerOptions {
  // Number of outer iterations. Each one rebuilds the tuples, the observation
  // filter, the chirality references and the IRLS weights from the current
  // rotations, then solves the rotations with Ceres (pyTheia choice).
  int num_outer_iterations = 5;
  // Maximum number of Ceres iterations per solve.
  int max_num_iterations = 50;
  // Cauchy scale in pixels, used for the IRLS weights 1 / (1 + s / c^2) with
  // c = loss_scale_px / f_j, f_j the focal length of the target view.
  double loss_scale_px = 2.0;
  // Maximum number of tuples (observations) per target view. The tuples with
  // the largest ||theta|| are kept.
  int max_observations_per_view = 500;
  // Maximum number of shared tracks used per triplet (or per view pair in the
  // two-view case). The first tracks by TrackId are used. Each track adds to
  // the cost of every Ceres residual evaluation of its triplet, so the default
  // is lower than the ART option (pyTheia choice; 50 matches the ETH3D runs).
  int max_tracks_per_triplet = 50;
  // Use the two-view tuples (l = j, translation from the P_ij eigenvector) for
  // observations without a valid triplet.
  bool use_two_view_tuples = true;
  // Only use reference pairs (i, l) that are edges of view_pairs.
  bool require_view_graph_edges = false;
  // Reference pairs examined per observation, in order of decreasing
  // ||theta^{il}|| (three-view) or ||theta^{ij}|| (two-view). Only pairs with a
  // valid filtered Theorem 1 solution that keeps the observation are accepted.
  int max_reference_candidates = 8;
  // Lower bound (pixels) of the residual gate of a tuple. The gate of an outer
  // iteration is max(tuple_gate_px, 3 * 1.4826 * MAD), where MAD is the median
  // absolute deviation of the residuals (px) of all candidate tuples of that
  // iteration. A tuple is accepted if both residual terms are below the gate.
  double tuple_gate_px = 3.0;
  // Epipolar verification of the observations before the refinement
  // (pyTheia addition): per view pair a robust (least median) two-view
  // translation at the initial rotations, and an observation is dropped if
  // its epipolar distance exceeds max(verify_observations_px, 5 * median
  // distance of the pair) for most of the other views of its track. Guards
  // against unverified mismatches; 0 disables it.
  double verify_observations_px = 4.0;
  int num_threads = 1;
};

// Refines the global orientations (world-to-camera angle-axis rotations) with
// the rotation-only reprojection error of MORE. Views without an orientation
// are not used. Returns false, leaving `orientations` untouched, if no tuple
// could be built or the Ceres solve failed.
bool RefineRotationsWithRotationOnlyError(
    const RotationOnlyRefinerOptions& options,
    const Reconstruction& reconstruction,
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    std::unordered_map<ViewId, Eigen::Vector3d>* orientations);

namespace rotation_only_internal {

// Theorem 1 evaluated in double precision as a function of the rotations
// (angle-axis) of the triplet (i, l, j), without observation filtering. The
// sign is fixed by `reference`. Used by the unit tests to check the templated
// formulation against SolveARTTriplet. Returns false if the result is not
// finite.
bool ComputeTheorem1Baselines(const std::vector<Eigen::Vector3d>& x_i,
                              const std::vector<Eigen::Vector3d>& x_l,
                              const std::vector<Eigen::Vector3d>& x_j,
                              const Eigen::Vector3d& angle_axis_i,
                              const Eigen::Vector3d& angle_axis_l,
                              const Eigen::Vector3d& angle_axis_j,
                              const Eigen::Vector3d& reference,
                              Eigen::Vector3d* b_il,
                              Eigen::Vector3d* b_ij);

// Cost function of one triplet (i, l, j) with the tuples equal to its Theorem 1
// tracks (unit IRLS weights). Parameter blocks: angle-axis of R_i, R_l, R_j.
// 4 residuals per track. The caller owns the result. Used by the unit tests to
// check the derivatives.
ceres::CostFunction* CreateTripletCostFunctionForTest(
    const std::vector<Eigen::Vector3d>& x_i,
    const std::vector<Eigen::Vector3d>& x_l,
    const std::vector<Eigen::Vector3d>& x_j,
    const Eigen::Vector3d& reference);

}  // namespace rotation_only_internal

}  // namespace theia

#endif  // THEIA_SFM_GLOBAL_POSE_ESTIMATION_ROTATION_ONLY_REFINER_H_
