// BSD 3-Clause License

// Copyright (c) 2021, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "theia/sfm/global_pose_estimation/l1_rotation_global_estimator.h"

#include <glog/logging.h>
#include <Eigen/SparseCore>

#include "theia/sfm/global_pose_estimation/rotation_estimator_util.h"
#include "theia/math/l1_solver.h"
#include "theia/util/map_util.h"
#include "theia/util/timer.h"
#include "theia/math/rotation.h"

namespace theia {

L1RotationGlobalEstimator::L1RotationGlobalEstimator(
    const int num_orientations, const int num_edges,
    const L1RotationOptions& options)
    : options_(options) {
  tangent_space_step_.resize((num_orientations - 1) * 3);
  tangent_space_residual_.resize(num_edges * 3);
}

void L1RotationGlobalEstimator::SetViewIdToIndex(
    const std::unordered_map<ViewId, int>& view_id_to_index) {
  view_id_to_index_ = view_id_to_index;
}

void L1RotationGlobalEstimator::SetSparseMatrix(
    const Eigen::SparseMatrix<double>& sparse_matrix) {
  sparse_matrix_ = sparse_matrix;
}

bool L1RotationGlobalEstimator::SolveL1Regression(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& relative_rotations,
    std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations) {
  const int num_edges = relative_rotations.size();

  CHECK_NOTNULL(global_rotations);
  CHECK_GT(global_rotations->size(), 0);
  CHECK_GT(num_edges, 0);

  if (view_id_to_index_.empty()) {
    ViewIdToAscentIndex(*global_rotations, &view_id_to_index_);
  }

  if (sparse_matrix_.rows() == 0) {
    SetupLinearSystem(
        relative_rotations, (*global_rotations).size(),
        view_id_to_index_, &sparse_matrix_);
  }

  L1Solver<Eigen::SparseMatrix<double>>::Options l1_solver_options;
  l1_solver_options.max_num_iterations = 5;
  L1Solver<Eigen::SparseMatrix<double> > l1_solver(
      l1_solver_options, sparse_matrix_);

  tangent_space_step_.setZero();
  ComputeResiduals(relative_rotations, global_rotations);

  Timer timer;
  for (int i = 0; i < options_.max_num_l1_iterations; i++) {
    l1_solver.Solve(tangent_space_residual_, &tangent_space_step_);
    UpdateGlobalRotations(global_rotations);
    ComputeResiduals(relative_rotations, global_rotations);

    double avg_step_size = ComputeAverageStepSize();

    if (avg_step_size <= options_.l1_step_convergence_threshold) {
      break;
    }
    l1_solver_options.max_num_iterations *= 2;
    l1_solver.SetMaxIterations(l1_solver_options.max_num_iterations);
  }

  LOG(INFO) << "Total time [L1Regression]: "
            << timer.ElapsedTimeInSeconds() * 1e3 << " ms.";

  return true;
}

void L1RotationGlobalEstimator::UpdateGlobalRotations(
    std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations) {
  for (auto& rotation : *global_rotations) {
    const int view_index = FindOrDie(view_id_to_index_, rotation.first) - 1;
    if (view_index == kConstantRotationIndex) {
      continue;
    }

    // Apply the rotation change to the global orientation.
    const Eigen::Vector3d& rotation_change =
        tangent_space_step_.segment<3>(3 * view_index);
    rotation.second = theia::MultiplyRotations(rotation.second, rotation_change);
  }
}

void L1RotationGlobalEstimator::ComputeResiduals(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& relative_rotations,
    std::unordered_map<ViewId, Eigen::Vector3d>* global_rotations) {
  int rotation_error_index = 0;

  for (const auto& relative_rotation : relative_rotations) {
    const Eigen::Vector3d& relative_rotation_aa = relative_rotation.second.rotation_2;
    const Eigen::Vector3d& rotation1 =
        FindOrDie(*global_rotations, relative_rotation.first.first);
    const Eigen::Vector3d& rotation2 =
        FindOrDie(*global_rotations, relative_rotation.first.second);

    // Compute the relative rotation error as:
    //   R_err = R2^t * R_12 * R1.
    tangent_space_residual_.segment<3>(3 * rotation_error_index) =
        theia::MultiplyRotations(-rotation2,
            theia::MultiplyRotations(relative_rotation_aa, rotation1));
    ++rotation_error_index;
  }
}

double L1RotationGlobalEstimator::ComputeAverageStepSize() {
  // compute the average step size of the update in tangent_space_step_
  const int num_vertices = tangent_space_step_.size() / 3;
  double delta_V = 0;
  for (int k = 0; k < num_vertices; ++k) {
    delta_V += tangent_space_step_.segment<3>(3 * k).norm();
  }
  return delta_V / num_vertices;
}

}  // namespace gopt
