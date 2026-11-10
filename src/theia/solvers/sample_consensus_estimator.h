// Copyright (C) 2013 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#ifndef THEIA_SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_H_
#define THEIA_SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_H_

#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <limits>
#include <memory>
#include <vector>
#include <iostream>

#include "theia/solvers/estimator.h"
#include "theia/solvers/inlier_support.h"
#include "theia/solvers/mle_quality_measurement.h"
#include "theia/solvers/quality_measurement.h"
#include "theia/solvers/sampler.h"

namespace theia {

// Helper struct to hold parameters to various RANSAC schemes. error_thresh is
// the threshold for consider data points to be inliers to a model. This is the
// only variable that must be explitly set, and the rest can be used with the
// default values unless other values are desired.
struct RansacParameters {
  RansacParameters()
      : error_thresh(-1),
        failure_probability(0.01),
        min_inlier_ratio(0),
        min_iterations(100),
        max_iterations(std::numeric_limits<int>::max()),
        use_mle(false),
        use_Tdd_test(false),
        use_lo(false),
        lo_start_iterations(10),
        lo_max_refinements(5),
        use_sturm_5pt(true) {}

  // The random number generator used to compute random number during
  // RANSAC. This may be controlled by the caller for debugging purposes.
  std::shared_ptr<RandomNumberGenerator> rng;

  // Error threshold to determine inliers for RANSAC. It is compared against the
  // estimator's Error(), which for the pose estimators in theia/sfm/estimators
  // is the SQUARED error in NORMALIZED image coordinates, i.e. after applying
  // inv(K) to the pixel coordinates (x_n = K^-1 [u, v, 1]^T). To use a pixel
  // threshold `t_px` with focal length `f` (pixels), set
  //   error_thresh = (t_px / f)^2.
  // Passing t_px / f (not squared) makes the threshold far too permissive
  // (e.g. 2 px at f = 3400 gives 6e-4 instead of 3.5e-7).
  double error_thresh;

  // The failure probability of RANSAC. Set to 0.01 means that RANSAC has a 1%
  // chance of missing the correct pose.
  double failure_probability;

  // The minimal assumed inlier ratio, i.e., it is assumed that the given set
  // of correspondences has an inlier ratio of at least min_inlier_ratio.
  // This is required to limit the number of RANSAC iteratios.
  double min_inlier_ratio;

  // The minimum number of iterations required before exiting.
  int min_iterations;

  // Another way to specify the maximal number of RANSAC iterations. In effect,
  // the maximal number of iterations is set to min(max_ransac_iterations, T),
  // where T is the number of iterations corresponding to min_inlier_ratio.
  // This variable is useful if RANSAC is to be applied iteratively, i.e.,
  // first applying RANSAC with an min_inlier_ratio of x, then with one
  // of x-y and so on, and we want to avoid repeating RANSAC iterations.
  // However, the preferable way to limit the number of RANSAC iterations is
  // to set min_inlier_ratio and leave max_ransac_iterations to its default
  // value.
  // Per default, this variable is set to std::numeric_limits<int>::max().
  int max_iterations;

  // Instead of the standard inlier count, use the Maximum Likelihood Estimate
  // (MLE) to determine the best solution. Inliers are weighted by their error
  // and outliers count as a constant penalty.
  bool use_mle;

  // If local optimization should be used (LO-RANSAC). This only works if the
  // corresponding estimator has the RefineModel() model function implemented.
  // Otherwise no local optimization is performed
  bool use_lo;

  // Local optimization runs for every new best model found from this RANSAC
  // iteration on. A short delay avoids refining the many short-lived best
  // models of the first iterations; it must stay small, because RANSAC often
  // terminates after a few dozen iterations on clean data. The final model is
  // always refined, independent of this value.
  int lo_start_iterations;

  // Maximum number of refinement rounds per local optimization. Each round
  // refines the model on its inliers, rescores it on all data and re-selects
  // the inliers; the refined model is kept only if it lowers the truncated
  // (MSAC) cost, and the rounds stop once it no longer does (re-selection as
  // in COLMAP's LORANSAC, acceptance as in PoseLib). The inlier ratio of the
  // refined model drives the termination criterion.
  int lo_max_refinements;

  // Whether to use the T_{d,d}, with d=1, test proposed in
  // Chum, O. and Matas, J.: Randomized RANSAC and T(d,d) test, BMVC 2002.
  // After computing the pose, RANSAC selects one match at random and evaluates
  // all poses. If the point is an outlier to one pose, the corresponding pose
  // is rejected. Notice that if the pose solver returns multiple poses, then
  // at most one pose is correct. If the selected match is correct, then only
  // the correct pose will pass the test. Per default, the test is disabled.
  //
  // NOTE: Not currently implemented!
  bool use_Tdd_test;

  // Use the Sturm-sequence-based 5-point solver (ported from PoseLib,
  // theia::FivePointRelativePoseSturm) instead of theia's own Stewenius-style
  // eigendecomposition solver (FivePointRelativePose) inside 5-point RANSAC
  // estimators (RelativePoseEstimator, EssentialMatrixEstimator). The Sturm
  // solver is significantly faster since it never forms a 10x10
  // eigendecomposition, but it is strictly minimal (only used for exactly-5
  // point samples; non-minimal calls always use FivePointRelativePose).
  bool use_sturm_5pt;
};

// A struct to hold useful outputs of Ransac-like methods.
struct RansacSummary {
  // Contains the indices of all inliers.
  std::vector<int> inliers;

  // Number of input data
  int num_input_data_points;

  // The number of iterations performed before stopping RANSAC.
  int num_iterations;

  // The confidence in the solution.
  double confidence;

  // Number of local optimization iterations
  int num_lo_iterations = 0;
};

template <class ModelEstimator>
class SampleConsensusEstimator {
 public:
  typedef typename ModelEstimator::Datum Datum;
  typedef typename ModelEstimator::Model Model;

  SampleConsensusEstimator(const RansacParameters& ransac_params,
                           const ModelEstimator& estimator);

  virtual bool Initialize() { return true; }

  virtual ~SampleConsensusEstimator() {}

  // Computes the best-fitting model using RANSAC. Returns false if RANSAC
  // calculation fails and true (with the best_model output) if successful.
  // Params:
  //   data: the set from which to sample
  //   estimator: The estimator used to estimate the model based on the Datum
  //     and Model type
  //   best_model: The output parameter that will be filled with the best model
  //     estimated from RANSAC
  virtual bool Estimate(const std::vector<Datum>& data,
                        Model* best_model,
                        RansacSummary* summary);

 protected:
  // This method is called from derived classes to set up the sampling scheme
  // and the method for computing inliers. It must be called by derived classes
  // unless they override the Estimate(...) method. The method for computing
  // inliers (standar inlier support or MLE) is determined by the ransac params.
  //
  // sampler: The class that instantiates the sampling strategy for this
  //   particular type of sampling consensus.
  bool Initialize(std::unique_ptr<Sampler> sampler);

  // Installs MLE or inlier-count quality measurement from ransac_params_.
  bool InitializeDefaultQualityMeasurement();

  void SetSampler(std::unique_ptr<Sampler> sampler);

  // Computes the maximum number of iterations required to ensure the inlier
  // ratio is the best with a probability corresponding to log_failure_prob.
  int ComputeMaxIterations(const double min_sample_size,
                           const double inlier_ratio,
                           const double log_failure_prob,
                           const int total_num_samples) const;

  // Local optimization: refines *model on the data indexed by *inlier_indices,
  // rescores the refined model on all data and re-selects its inliers, for up
  // to lo_max_refinements rounds. A refined model is accepted if it lowers the
  // truncated (MSAC) cost sum_i min(r_i, error_thresh); the rounds stop when
  // it no longer does. *model, *cost (the quality measurement's cost) and
  // *inlier_indices are updated with the accepted model.
  void LocallyOptimize(const std::vector<Datum>& data,
                       Model* model,
                       double* cost,
                       std::vector<int>* inlier_indices,
                       RansacSummary* summary);

  // Get inlier datum points (for LO)
  void GetInlierDatum(const std::vector<Datum>& data,
                      std::vector<Datum>& inlier_datum,
                      const std::vector<int>& inlier_indices);

  // The sampling strategy.
  std::unique_ptr<Sampler> sampler_;

  // The quality metric for the estimated model and data.
  std::unique_ptr<QualityMeasurement> quality_measurement_;

  // Ransac parameters (see above struct).
  const RansacParameters& ransac_params_;

  // Estimator to use for generating models.
  const ModelEstimator& estimator_;
};

// --------------------------- Implementation --------------------------------//

template <class ModelEstimator>
SampleConsensusEstimator<ModelEstimator>::SampleConsensusEstimator(
    const RansacParameters& ransac_params, const ModelEstimator& estimator)
    : ransac_params_(ransac_params), estimator_(estimator) {
  CHECK_GT(ransac_params.error_thresh, 0)
      << "Error threshold must be set to greater than zero";
  CHECK_LE(ransac_params.min_inlier_ratio, 1.0);
  CHECK_GE(ransac_params.min_inlier_ratio, 0.0);
  CHECK_LT(ransac_params.failure_probability, 1.0);
  CHECK_GT(ransac_params.failure_probability, 0.0);
  CHECK_GE(ransac_params.max_iterations, ransac_params.min_iterations);
}

template <class ModelEstimator>
void SampleConsensusEstimator<ModelEstimator>::SetSampler(
    std::unique_ptr<Sampler> sampler) {
  CHECK(sampler != nullptr);
  sampler_ = std::move(sampler);
}

template <class ModelEstimator>
bool SampleConsensusEstimator<ModelEstimator>::InitializeDefaultQualityMeasurement() {
  if (ransac_params_.use_mle) {
    quality_measurement_.reset(
        new MLEQualityMeasurement(ransac_params_.error_thresh));
  } else {
    quality_measurement_.reset(new InlierSupport(ransac_params_.error_thresh));
  }
  return quality_measurement_->Initialize();
}

template <class ModelEstimator>
bool SampleConsensusEstimator<ModelEstimator>::Initialize(
    std::unique_ptr<Sampler> sampler) {
  SetSampler(std::move(sampler));
  return InitializeDefaultQualityMeasurement();
}

template <class ModelEstimator>
int SampleConsensusEstimator<ModelEstimator>::ComputeMaxIterations(
    const double min_sample_size,
    const double inlier_ratio,
    const double log_failure_prob,
    const int total_num_samples) const {
  CHECK_GT(inlier_ratio, 0.0);
  if (inlier_ratio == 1.0) {
    return ransac_params_.min_iterations;
  }

  // This includes a fix proposed in "Fixing the RANSAC Stopping Criterion"
  // by J. Schoenberger et al. (2025) 

  // Number of inliers.
  const int ninl = static_cast<int>(inlier_ratio * total_num_samples);

  // If we use the T_{1,1} test, we have to adapt the number of samples
  // that needs to be generated accordingly since we use another
  // match for verification and a correct match is selected with probability
  // inlier_ratio.
  const double num_samples =
      ransac_params_.use_Tdd_test ? min_sample_size + 1 : min_sample_size;

  // Compute the probability of selecting 'samsiz' inliers in a row (without replacement).
  double a = 1.0, b = 1.0;
  for (int i = 0; i < num_samples; ++i) {
    a *= ninl - i;
    b *= total_num_samples - i;
  }
  const double prob_all_inliers = a / b;

  // Handle degenerate cases.
  if (prob_all_inliers < std::numeric_limits<double>::epsilon()) {
    return ransac_params_.max_iterations;
  }
  if (prob_all_inliers >= 1.0 - std::numeric_limits<double>::epsilon()) {
    return ransac_params_.min_iterations;
  }

  // Compute the required number of iterations.
  const double num_iterations = log_failure_prob / log(1.0 - prob_all_inliers);

  return std::max(static_cast<double>(ransac_params_.min_iterations),
                  std::min(num_iterations,
                           static_cast<double>(ransac_params_.max_iterations)));
}

template <class ModelEstimator>
bool SampleConsensusEstimator<ModelEstimator>::Estimate(
    const std::vector<Datum>& data, Model* best_model, RansacSummary* summary) {
  CHECK_GT(data.size(), 0)
      << "Cannot perform estimation with 0 data measurements!";
  CHECK_NOTNULL(sampler_.get());
  CHECK_NOTNULL(quality_measurement_.get());
  CHECK_NOTNULL(summary);
  summary->inliers.clear();
  CHECK_NOTNULL(best_model);

  // Initialize the sampler with the size of the data input.
  if (!sampler_->Initialize(data.size())) {
    return false;
  }

  summary->num_input_data_points = data.size();

  const double log_failure_prob = log(ransac_params_.failure_probability);
  double best_cost = std::numeric_limits<double>::max();
  int max_iterations = ransac_params_.max_iterations;

  // Set the max iterations if the inlier ratio is set.
  if (ransac_params_.min_inlier_ratio > 0) {
    max_iterations =
        std::min(ComputeMaxIterations(estimator_.SampleSize(),
                                      ransac_params_.min_inlier_ratio,
                                      log_failure_prob, data.size()),
                 ransac_params_.max_iterations);
  }

  // Reused across iterations to avoid per-iteration heap allocations; the
  // sampler and estimator only ever append to these after a clear(), so
  // clearing (rather than reallocating) keeps the reserved capacity around.
  std::vector<int> data_subset_indices;
  std::vector<Datum> data_subset;
  std::vector<Model> temp_models;
  std::vector<int> inlier_indices;

  for (summary->num_iterations = 0; summary->num_iterations < max_iterations;
       summary->num_iterations++) {
    // Sample subset. Proceed if successfully sampled.
    data_subset_indices.clear();
    if (!sampler_->Sample(&data_subset_indices)) {
      continue;
    }

    // Get the corresponding data elements for the subset.
    data_subset.resize(data_subset_indices.size());
    for (int i = 0; i < data_subset_indices.size(); i++) {
      data_subset[i] = data[data_subset_indices[i]];
    }

    // Estimate model from subset. Skip to next iteration if the model fails to
    // estimate.
    temp_models.clear();
    if (!estimator_.EstimateModel(data_subset, &temp_models)) {
      continue;
    }

    // Calculate residuals from estimated model.
    for (const Model& temp_model : temp_models) {
      const std::vector<double> residuals =
          estimator_.Residuals(data, temp_model);

      // Count-only cost: avoids building the inlier index vector for models
      // that do not end up beating the current best (the vast majority).
      const double sample_cost = quality_measurement_->ComputeCost(residuals);

      // Update best model if error is the best we have seen.
      if (sample_cost < best_cost) {
        // Only now, for the (rare) winning model, pay for the inlier index
        // vector -- it is needed below for LO-RANSAC and the iteration
        // stopping criterion.
        inlier_indices.clear();
        quality_measurement_->ComputeCost(residuals, &inlier_indices);

        *best_model = temp_model;
        best_cost = sample_cost;
        const double inlier_ratio =
            static_cast<double>(inlier_indices.size()) /
            static_cast<double>(data.size());

        if (inlier_ratio <
            estimator_.SampleSize() / static_cast<double>(data.size())) {
          continue;
        }

        if (ransac_params_.use_lo &&
            summary->num_iterations >= ransac_params_.lo_start_iterations) {
          LocallyOptimize(data, best_model, &best_cost, &inlier_indices,
                          summary);
        }
        const double refined_inlier_ratio =
            static_cast<double>(inlier_indices.size()) /
            static_cast<double>(data.size());

        // A better cost does not guarantee a higher inlier ratio (i.e, the MLE
        // case) so we only update the max iterations if the number decreases.
        max_iterations = std::min(
            ComputeMaxIterations(estimator_.SampleSize(),
                                 refined_inlier_ratio,
                                 log_failure_prob,
                                 data.size()),
            max_iterations);

        VLOG(3) << "Inlier ratio = " << refined_inlier_ratio
                << " and max number of iterations = " << max_iterations;
      }
    }
  }

  // Compute the final inliers for the best model and refine it once more,
  // re-selecting the inliers as long as the cost decreases.
  const std::vector<double> best_residuals =
      estimator_.Residuals(data, *best_model);
  best_cost = quality_measurement_->ComputeCost(best_residuals,
                                                &summary->inliers);

  if (ransac_params_.use_lo &&
      summary->inliers.size() >= estimator_.SampleSize()) {
    LocallyOptimize(data, best_model, &best_cost, &summary->inliers, summary);
  }

  const double inlier_ratio =
      static_cast<double>(summary->inliers.size()) / data.size();
  summary->confidence =
      1.0 - pow(1.0 - pow(inlier_ratio, estimator_.SampleSize()),
                summary->num_iterations);

  return true;
}

template <class ModelEstimator>
void SampleConsensusEstimator<ModelEstimator>::LocallyOptimize(
    const std::vector<Datum>& data,
    Model* model,
    double* cost,
    std::vector<int>* inlier_indices,
    RansacSummary* summary) {
  // Refined models are accepted by their truncated (MSAC) cost, as in
  // PoseLib: the inlier count is too coarse and rejects refined models that
  // are more accurate but lose a single inlier at the threshold.
  const double threshold = ransac_params_.error_thresh;
  const auto msac_cost = [threshold](const std::vector<double>& residuals) {
    double total = 0.0;
    for (const double residual : residuals) {
      total += std::min(residual, threshold);
    }
    return total;
  };
  double current_msac = msac_cost(estimator_.Residuals(data, *model));

  std::vector<Datum> inliers;
  std::vector<int> refined_inlier_indices;
  for (int round = 0; round < ransac_params_.lo_max_refinements; ++round) {
    if (inlier_indices->size() < estimator_.SampleSize()) {
      return;
    }
    GetInlierDatum(data, inliers, *inlier_indices);
    Model refined_model = *model;
    ++summary->num_lo_iterations;
    if (!estimator_.RefineModel(
            inliers, ransac_params_.error_thresh, &refined_model)) {
      return;
    }
    const std::vector<double> residuals =
        estimator_.Residuals(data, refined_model);
    const double refined_msac = msac_cost(residuals);
    if (!(refined_msac < current_msac)) {
      return;
    }
    const bool converged = refined_msac > current_msac * (1.0 - 1e-6);
    refined_inlier_indices.clear();
    *cost =
        quality_measurement_->ComputeCost(residuals, &refined_inlier_indices);
    *model = refined_model;
    current_msac = refined_msac;
    inlier_indices->swap(refined_inlier_indices);
    if (converged) {
      return;
    }
  }
}

template <class ModelEstimator>
void SampleConsensusEstimator<ModelEstimator>::GetInlierDatum(
    const std::vector<Datum>& data,
    std::vector<Datum>& inlier_datum,
    const std::vector<int>& inlier_indices) {
  inlier_datum.resize(inlier_indices.size());
  for (int i = 0; i < inlier_indices.size(); i++) {
    inlier_datum[i] = data[inlier_indices[i]];
  }
}

}  // namespace theia

#endif  // THEIA_SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_H_
