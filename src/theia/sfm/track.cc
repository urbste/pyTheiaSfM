// Copyright (C) 2015 The Regents of the University of California (Regents).
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

#include "theia/sfm/track.h"

#include <algorithm>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <unordered_set>

#include "theia/util/map_util.h"

namespace theia {

using Eigen::Vector3d;
using Eigen::Vector4d;

Track::Track()
    : is_estimated_(false),
      inverse_depth_(0.0),
      reference_view_id_(theia::kInvalidViewId),
      has_position_prior_(false) {
  point_.setZero();
  color_.setZero();
  reference_bearing_.setZero();
  position_prior_.setZero();
  position_prior_sqrt_information_.setIdentity();
}

int Track::NumViews() const { return view_ids_.size(); }

void Track::SetEstimated(const bool is_estimated) {
  is_estimated_ = is_estimated;
}

bool Track::IsEstimated() const { return is_estimated_; }

const Eigen::Vector4d& Track::Point() const { return point_; }

Vector4d* Track::MutablePoint() { return &point_; }

void Track::SetPoint(const Eigen::Vector4d& point) { point_ = point; }

const Eigen::Matrix<uint8_t, 3, 1>& Track::Color() const { return color_; }

Eigen::Matrix<uint8_t, 3, 1>* Track::MutableColor() { return &color_; }

void Track::SetColor(const Eigen::Matrix<uint8_t, 3, 1>& color) {
  color_ = color;
}

void Track::AddView(const ViewId view_id) {
  view_ids_.insert(view_id);
  if (reference_view_id_ == theia::kInvalidViewId) {
    reference_view_id_ = view_id;
  }
}

bool Track::RemoveView(const ViewId view_id) {
  const bool successfull_removed = view_ids_.erase(view_id) > 0;
  if (!successfull_removed) {
    // Removing a view that does not observe the track must not touch the
    // reference view.
    return false;
  }
  if (view_ids_.empty()) {
    reference_view_id_ = theia::kInvalidViewId;
  } else if (reference_view_id_ == view_id ||
             view_ids_.count(reference_view_id_) == 0) {
    // Only re-select the reference view if the removed view was the reference.
    // Use the smallest id so the choice is deterministic.
    reference_view_id_ = *std::min_element(view_ids_.begin(), view_ids_.end());
  }
  return true;
}

const std::unordered_set<ViewId>& Track::ViewIds() const { return view_ids_; }

ViewId Track::ReferenceViewId() const { return reference_view_id_; }

void Track::SetReferenceViewId(const ViewId view_id) {
  reference_view_id_ = view_id;
}

const double& Track::InverseDepth() const { return inverse_depth_; }

double* Track::MutableInverseDepth() { return &inverse_depth_; }

void Track::SetInverseDepth(const double& inverse_depth) {
  inverse_depth_ = inverse_depth;
}

void Track::SetReferenceBearingVector(const Vector3d& ref_bearing) {
  reference_bearing_ = ref_bearing;
}

const Vector3d& Track::ReferenceBearingVector() const {
  return reference_bearing_;
}

void Track::SetReferenceDescriptor(const Eigen::VectorXf& descriptor) {
  reference_descriptor_ = descriptor;
}

const Eigen::VectorXf& Track::ReferenceDescriptor() const {
  return reference_descriptor_;
}

void Track::SetPositionPrior(const Vector3d& position,
                             const Eigen::Matrix3d& sqrt_information) {
  position_prior_ = position;
  position_prior_sqrt_information_ = sqrt_information;
  has_position_prior_ = true;
}

bool Track::SetPositionPriorCovariance(const Vector3d& position,
                                       const Eigen::Matrix3d& covariance) {
  // Check that the covariance is symmetric positive definite before inverting.
  const Eigen::Matrix3d symmetric_covariance = 0.5 * (covariance + covariance.transpose());
  Eigen::LLT<Eigen::Matrix3d> cov_llt(symmetric_covariance);
  if (cov_llt.info() != Eigen::Success) {
    return false;
  }
  const Eigen::Matrix3d information = symmetric_covariance.inverse();
  // Information = L * L^T, so S = L^T satisfies S^T * S = Information.
  Eigen::LLT<Eigen::Matrix3d> info_llt(0.5 * (information + information.transpose()));
  if (info_llt.info() != Eigen::Success) {
    return false;
  }
  SetPositionPrior(position, info_llt.matrixU());
  return true;
}

bool Track::HasPositionPrior() const { return has_position_prior_; }

const Vector3d& Track::PositionPrior() const { return position_prior_; }

const Eigen::Matrix3d& Track::PositionPriorSqrtInformation() const {
  return position_prior_sqrt_information_;
}

void Track::RemovePositionPrior() {
  has_position_prior_ = false;
  position_prior_.setZero();
  position_prior_sqrt_information_.setIdentity();
}

}  // namespace theia
