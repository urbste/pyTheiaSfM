// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_GLOBAL_POSE_ESTIMATION_GLOMAP_POSITION_ESTIMATOR_H_
#define THEIA_SFM_GLOBAL_POSE_ESTIMATION_GLOMAP_POSITION_ESTIMATOR_H_

#include <Eigen/Core>
#include <memory>
#include <unordered_map>

#include "theia/sfm/global_pose_estimation/position_estimator.h"
#include "theia/sfm/types.h"
#include "theia/util/util.h"

namespace theia {

class RandomNumberGenerator;
class Reconstruction;
class TwoViewInfo;

// Jointly estimates camera centers and 3D points from globally rotated feature
// rays, following the global positioning objective introduced by GLOMAP.
class GlomapPositionEstimator : public PositionEstimator {
 public:
  struct Options {
    std::shared_ptr<RandomNumberGenerator> rng;
    int num_threads = 1;
    int max_num_iterations = 100;
    double robust_loss_width = 0.1;
    int min_track_length = 3;
    int max_num_tracks = -1;
    bool write_points_to_reconstruction = false;
    bool use_pairwise_scale_priors = false;
    double pairwise_scale_prior_weight = 1.0;
  };

  GlomapPositionEstimator(const Options& options,
                          Reconstruction* reconstruction);

  bool EstimatePositions(
      const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
      const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
      std::unordered_map<ViewId, Eigen::Vector3d>* positions) override;

  std::unordered_map<ViewId, Eigen::Vector3d> EstimatePositionsWrapper(
      const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
      const std::unordered_map<ViewId, Eigen::Vector3d>& orientations);

 private:
  const Options options_;
  Reconstruction* reconstruction_;
  std::shared_ptr<RandomNumberGenerator> rng_;

  DISALLOW_COPY_AND_ASSIGN(GlomapPositionEstimator);
};

}  // namespace theia

#endif  // THEIA_SFM_GLOBAL_POSE_ESTIMATION_GLOMAP_POSITION_ESTIMATOR_H_
