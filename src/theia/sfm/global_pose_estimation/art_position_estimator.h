// Implements ART (Analytical Rotation-guided Translation), Sec. 5.3 / Alg. 3 of
// Li, Cai, Wu, arXiv:2611.11770. Parameters not given in the paper are pyTheia
// choices.
//
// Given global orientations (world-to-camera rotations) and normalized feature
// correspondences, ART estimates camera centers in a rotation-only fashion:
//  1. Each view triplet (eta, zeta, nu) with its shared tracks is solved in
//     closed form (Theorem 1): the two baselines b_ez and b_en of the triplet
//     are the eigenvector of a 3x3 matrix built from the LiGT constraint blocks,
//     which is known up to a per-triplet scale.
//  2. Per-triplet scales are linked through the triplets that share an edge,
//     and the global positions and scales are found with a robust (Huber IRLS)
//     sparse linear least-squares problem.
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

#ifndef THEIA_SFM_GLOBAL_POSE_ESTIMATION_ART_POSITION_ESTIMATOR_H_
#define THEIA_SFM_GLOBAL_POSE_ESTIMATION_ART_POSITION_ESTIMATOR_H_

#include <Eigen/Core>
#include <unordered_map>
#include <vector>

#include "theia/sfm/global_pose_estimation/position_estimator.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"

namespace theia {

// Result of the closed-form solution of one view triplet (Theorem 1).
struct ARTTripletSolution {
  // Unit-norm baseline b_ez = t_eta - t_zeta (reference edge), with the sign
  // fixed by the chirality check.
  Eigen::Vector3d b_ez = Eigen::Vector3d::Zero();
  // Baseline b_en = t_eta - t_nu = -T b_ez, same scale as b_ez.
  Eigen::Vector3d b_en = Eigen::Vector3d::Zero();
  // T = (C^T C)^-1 (C^T B) of Theorem 1.
  Eigen::Matrix3d T = Eigen::Matrix3d::Zero();
  // Eigenvalues of the symmetric matrix S, ascending.
  Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
  // eigenvalues(0) / eigenvalues(1). Smaller is better conditioned.
  double eigen_ratio = 0.0;
};

// Closed-form ART solution of one triplet (eta, zeta, nu), reference edge
// (eta, zeta). x_* are the normalized homogeneous feature rays (x, y, 1) of the
// shared tracks in the three views, R_* the world-to-camera rotations and
// weights the per-track weights omega_k > 0. Returns false if the solution is
// not defined (rank-deficient or degenerate configuration).
bool SolveARTTriplet(const std::vector<Eigen::Vector3d>& x_eta,
                     const std::vector<Eigen::Vector3d>& x_zeta,
                     const std::vector<Eigen::Vector3d>& x_nu,
                     const Eigen::Matrix3d& R_eta,
                     const Eigen::Matrix3d& R_zeta,
                     const Eigen::Matrix3d& R_nu,
                     const std::vector<double>& weights,
                     ARTTripletSolution* sol);

// Global camera position estimation from rotations and view triplets (ART).
//
// pyTheia additions to the paper's pipeline (see docs, "ART view coverage"):
// triplets are grouped into rigid components (shared edge, or two shared
// views between groups) instead of edge-connected ones; valid triplets that
// attach a new view rigidly are added back after selection; views outside
// the main component get bridging triplets and, failing that, the LiGT
// constraints of their tracks shared with the main component
// (min_bridge_tracks); each triplet's observation filter has a robust
// least-median start (num_robust_samples).
class ARTPositionEstimator : public PositionEstimator {
 public:
  struct Options {
    int num_threads = 1;
    int min_tracks_per_triplet = 10;
    int max_tracks_per_triplet = 200;
    int max_triplets_per_view = 50;
    // Also build candidate triplets from track co-visibility (view pairs
    // sharing >= min_tracks_per_triplet tracks), not only from triangles of
    // the verified view graph.
    bool use_track_covisibility_triplets = true;
    // Views outside the largest edge-connected component of candidate
    // triplets get extra candidates sharing at least this many tracks
    // (coverage fallback; 0 disables it).
    int min_tracks_per_triplet_fallback = 3;
    // Bridging rounds after triplet selection: views outside the selected
    // edge-connected component get candidate triplets (a, u, w) where (u, w)
    // is an edge of a selected triplet, sharing at least
    // min_tracks_per_triplet_fallback tracks. A valid bridge shares an edge
    // with the component, so it fixes the view's position at the common
    // scale. 0 disables bridging.
    int num_coverage_rounds = 3;
    // Views that triplets cannot attach to the main component (e.g. a group
    // of views sharing a single view with it) are attached with the LiGT
    // constraints (Cai et al. 2023) of the tracks they share with it, if
    // there are at least this many such tracks. 0 disables this.
    int min_bridge_tracks = 10;
    double max_eigen_ratio = 0.1;
    double min_c_conditioning = 1e-10;
    int num_obs_filter_iterations = 2;
    // Robust start of the observation filter: the per-triplet solution with
    // the lowest median residual among the least-squares one and solutions
    // from this many random subsets of 8 tracks (least median of squares).
    // Protects the filter against gross outliers. 0 disables it.
    int num_robust_samples = 20;
    double obs_mad_factor = 3.0;
    double triplet_conf_factor = 3.0;
    int num_irls_iterations = 10;
    double irls_scale_multiplier = 2.0;
    double irls_scale = 0.0;  // 0 = adaptive
    double irls_convergence_threshold = 1e-6;
  };

  ARTPositionEstimator(const Options& options,
                       const Reconstruction& reconstruction);

  // Estimates the positions. Returns false if no valid triplet was found.
  bool EstimatePositions(
      const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
      const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
      std::unordered_map<ViewId, Eigen::Vector3d>* positions) override;

  // python
  std::unordered_map<ViewId, Eigen::Vector3d> EstimatePositionsWrapper(
      const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
      const std::unordered_map<ViewId, Eigen::Vector3d>& orientation);

 private:
  const Options options_;
  const Reconstruction& reconstruction_;

  DISALLOW_COPY_AND_ASSIGN(ARTPositionEstimator);
};

// Same as SolveARTTriplet, with the observation filter of the ARTPositionEstimator
// (options.num_obs_filter_iterations, obs_mad_factor, min_tracks_per_triplet).
// If kept is not null, it receives the ascending indices of the tracks that
// survive the filter. Returns false if the triplet is not solvable.
bool SolveARTTripletFiltered(const std::vector<Eigen::Vector3d>& x_eta,
                             const std::vector<Eigen::Vector3d>& x_zeta,
                             const std::vector<Eigen::Vector3d>& x_nu,
                             const Eigen::Matrix3d& R_eta,
                             const Eigen::Matrix3d& R_zeta,
                             const Eigen::Matrix3d& R_nu,
                             const ARTPositionEstimator::Options& options,
                             ARTTripletSolution* sol,
                             std::vector<int>* kept);

}  // namespace theia

#endif  // THEIA_SFM_GLOBAL_POSE_ESTIMATION_ART_POSITION_ESTIMATOR_H_
