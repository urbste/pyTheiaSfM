// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/estimators/estimate_relative_rig_info.h"

#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <limits>
#include <vector>

#include "theia/sfm/pose/five_point_one_point_generalized_relative_pose.h"
#include "theia/sfm/pose/four_point_upright_generalized_relative_pose.h"
#include "theia/util/random.h"

namespace theia {
namespace {

void FillRelativeRigInfo(const RigidTransformation& pose,
                         RelativeRigInfo* info) {
  info->rotation = pose.rotation;
  info->translation = pose.translation;
  info->position = -pose.rotation.transpose() * pose.translation;
}

double SquaredGeneralizedEpipolarError(const GeneralizedRayCorrespondence& corr,
                                       const RelativeRigInfo& info) {
  const double r =
      GeneralizedEpipolarResidual(corr, info.rotation, info.translation);
  return r * r;
}

void CountInliers(const std::vector<GeneralizedRayCorrespondence>& matches,
                  const RelativeRigInfo& info,
                  const double sq_error_thresh,
                  std::vector<int>* inliers) {
  inliers->clear();
  inliers->reserve(matches.size());
  for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
    if (SquaredGeneralizedEpipolarError(matches[i], info) < sq_error_thresh) {
      inliers->push_back(i);
    }
  }
}

int ComputeMaxIterations(const RansacParameters& params,
                         const double inlier_ratio,
                         const int sample_size) {
  if (inlier_ratio < 1e-6) {
    return params.max_iterations;
  }
  const double num =
      std::log(params.failure_probability);
  const double den =
      std::log(std::max(1.0 - std::pow(inlier_ratio, sample_size), 1e-12));
  const int iters = static_cast<int>(std::ceil(num / den));
  return std::max(params.min_iterations,
                  std::min(iters, params.max_iterations));
}

}  // namespace

void RelativeRigInfo::ToTwoViewInfo(TwoViewInfo* info) const {
  CHECK_NOTNULL(info);
  Eigen::Vector3d aa;
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(rotation.data()), aa.data());
  info->rotation_2 = aa;
  const double scale = position.norm();
  if (scale > 1e-12) {
    info->position_2 = position / scale;
    info->scale_estimate = scale;
  } else {
    info->position_2 = Eigen::Vector3d::Zero();
    info->scale_estimate = -1.0;
  }
}

bool EstimateRelativeRigInfo(
    const RansacParameters& ransac_params,
    const std::vector<GeneralizedRayCorrespondence>& central_matches,
    const std::vector<GeneralizedRayCorrespondence>& generalized_matches,
    RelativeRigInfo* relative_rig_info,
    RansacSummary* ransac_summary) {
  CHECK_NOTNULL(relative_rig_info);
  CHECK_NOTNULL(ransac_summary);
  ransac_summary->inliers.clear();
  ransac_summary->num_iterations = 0;

  if (central_matches.size() < 5 || generalized_matches.empty()) {
    return false;
  }
  CHECK_GT(ransac_params.error_thresh, 0.0);

  RandomNumberGenerator local_rng;
  RandomNumberGenerator* rng =
      ransac_params.rng ? ransac_params.rng.get() : &local_rng;

  const double sq_thresh = ransac_params.error_thresh;
  int max_iters = ransac_params.max_iterations;
  if (max_iters > 1000000) {
    max_iters = 10000;  // practical default when unset
  }
  max_iters = std::max(max_iters, ransac_params.min_iterations);

  RelativeRigInfo best_info;
  std::vector<int> best_inliers;
  int best_count = -1;

  std::vector<GeneralizedRayCorrespondence> sample(6);
  for (int iter = 0; iter < max_iters; ++iter) {
    ++ransac_summary->num_iterations;

    // Sample 5 central + 1 generalized.
    for (int i = 0; i < 5; ++i) {
      const int idx = rng->RandInt(0, static_cast<int>(central_matches.size()) - 1);
      sample[i] = central_matches[idx];
    }
    {
      const int idx =
          rng->RandInt(0, static_cast<int>(generalized_matches.size()) - 1);
      sample[5] = generalized_matches[idx];
    }

    std::vector<RigidTransformation> hypotheses;
    if (FivePointOnePointGeneralizedRelativePose(sample, &hypotheses) == 0) {
      continue;
    }

    for (const RigidTransformation& pose : hypotheses) {
      RelativeRigInfo cand;
      FillRelativeRigInfo(pose, &cand);
      std::vector<int> inliers;
      // Score only same-sensor (central) matches. Cross-sensor rays on
      // forward-moving stereo look like the calibrated baseline and will
      // otherwise dominate consensus toward that degenerate pose.
      CountInliers(central_matches, cand, sq_thresh, &inliers);
      if (static_cast<int>(inliers.size()) > best_count) {
        best_count = static_cast<int>(inliers.size());
        best_inliers = inliers;
        best_info = cand;
        const double ratio =
            static_cast<double>(best_count) / central_matches.size();
        max_iters = std::min(
            max_iters, ComputeMaxIterations(ransac_params, ratio, /*sample=*/6));
      }
    }
  }

  if (best_count < 6) {
    return false;
  }

  *relative_rig_info = best_info;
  ransac_summary->inliers = best_inliers;
  return true;
}

bool EstimateRelativeRigInfoUpright(
    const RansacParameters& ransac_params,
    const Eigen::Vector3d& gravity_axis,
    const std::vector<GeneralizedRayCorrespondence>& matches,
    RelativeRigInfo* relative_rig_info,
    RansacSummary* ransac_summary) {
  CHECK_NOTNULL(relative_rig_info);
  CHECK_NOTNULL(ransac_summary);
  ransac_summary->inliers.clear();
  ransac_summary->num_iterations = 0;

  if (matches.size() < 4) {
    return false;
  }
  CHECK_GT(ransac_params.error_thresh, 0.0);

  RandomNumberGenerator local_rng;
  RandomNumberGenerator* rng =
      ransac_params.rng ? ransac_params.rng.get() : &local_rng;

  const double sq_thresh = ransac_params.error_thresh;
  int max_iters = ransac_params.max_iterations;
  if (max_iters > 1000000) {
    max_iters = 10000;
  }
  max_iters = std::max(max_iters, ransac_params.min_iterations);

  RelativeRigInfo best_info;
  std::vector<int> best_inliers;
  int best_count = -1;
  std::vector<GeneralizedRayCorrespondence> sample(4);

  for (int iter = 0; iter < max_iters; ++iter) {
    ++ransac_summary->num_iterations;
    for (int i = 0; i < 4; ++i) {
      sample[i] = matches[rng->RandInt(0, static_cast<int>(matches.size()) - 1)];
    }

    std::vector<RigidTransformation> hypotheses;
    if (FourPointUprightGeneralizedRelativePose(
            gravity_axis, sample, &hypotheses) == 0) {
      continue;
    }

    for (const RigidTransformation& pose : hypotheses) {
      RelativeRigInfo cand;
      FillRelativeRigInfo(pose, &cand);
      std::vector<int> inliers;
      CountInliers(matches, cand, sq_thresh, &inliers);
      if (static_cast<int>(inliers.size()) > best_count) {
        best_count = static_cast<int>(inliers.size());
        best_inliers = inliers;
        best_info = cand;
        const double ratio =
            static_cast<double>(best_count) / matches.size();
        max_iters = std::min(
            max_iters, ComputeMaxIterations(ransac_params, ratio, /*sample=*/4));
      }
    }
  }

  if (best_count < 4) {
    return false;
  }
  *relative_rig_info = best_info;
  ransac_summary->inliers = best_inliers;
  return true;
}

}  // namespace theia
