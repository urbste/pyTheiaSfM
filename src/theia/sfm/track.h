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

#ifndef THEIA_SFM_TRACK_H_
#define THEIA_SFM_TRACK_H_

#include <Eigen/Core>
#include <cereal/access.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/unordered_set.hpp>
#include <stdint.h>
#include <unordered_set>

#include "theia/io/eigen_serializable.h"
#include "theia/sfm/types.h"

namespace theia {

// A track contains information about a 3D point and the views that observe the
// point. This is based off of LibMV's Structure class:
// https://github.com/libmv/libmv/blob/master/src/libmv/multiview/structure.h
class Track {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Track();
  ~Track() {}

  int NumViews() const;

  void SetEstimated(const bool is_estimated);
  bool IsEstimated() const;

  const Eigen::Vector4d& Point() const;
  Eigen::Vector4d* MutablePoint();
  void SetPoint(const Eigen::Vector4d& point);

  const Eigen::Matrix<uint8_t, 3, 1>& Color() const;
  Eigen::Matrix<uint8_t, 3, 1>* MutableColor();
  void SetColor(const Eigen::Matrix<uint8_t, 3, 1>& color);

  void AddView(const ViewId view_id);
  bool RemoveView(const ViewId view_id);

  const std::unordered_set<ViewId>& ViewIds() const;

  ViewId ReferenceViewId() const;
  // Sets the view that anchors the inverse depth parametrization. The view
  // should observe the track; the reference bearing vector and inverse depth
  // must be re-initialized afterwards (Reconstruction::InitializeTrackInverseDepth).
  void SetReferenceViewId(const ViewId view_id);

  const double& InverseDepth() const;
  double* MutableInverseDepth();
  void SetInverseDepth(const double& inverse_depth);

  void SetReferenceBearingVector(const Eigen::Vector3d& ref_bearing);
  const Eigen::Vector3d& ReferenceBearingVector() const;

  void SetReferenceDescriptor(const Eigen::VectorXf& descriptor);
  const Eigen::VectorXf& ReferenceDescriptor() const;

  // Optional prior on the 3D position of the track (e.g. a ground control point
  // or a LiDAR point). When present and BundleAdjustmentOptions::
  // use_point_priors is set, bundle adjustment adds the residual
  // sqrt_information * (X - position) for optimized tracks. The position is in
  // the same units as the reconstruction (e.g. meters).
  //
  // The sqrt information S satisfies S^T S = Sigma^-1 (Sigma: covariance of the
  // position). The residual is whitened by S, so a larger S means a tighter
  // prior.
  void SetPositionPrior(const Eigen::Vector3d& position,
                        const Eigen::Matrix3d& sqrt_information);
  // Same as SetPositionPrior() but takes the 3x3 covariance of the position
  // and computes the sqrt information from its inverse. Returns false (and
  // leaves the track unchanged) if the covariance is not symmetric positive
  // definite.
  bool SetPositionPriorCovariance(const Eigen::Vector3d& position,
                                  const Eigen::Matrix3d& covariance);
  bool HasPositionPrior() const;
  const Eigen::Vector3d& PositionPrior() const;
  const Eigen::Matrix3d& PositionPriorSqrtInformation() const;
  void RemovePositionPrior();

 private:
  // Templated method for disk I/O with cereal. This method tells cereal which
  // data members should be used when reading/writing to/from disk.
  friend class cereal::access;
  // Version 1 does NOT write view_ids_: a Track only knows its views through
  // the Reconstruction, which rebuilds view_ids_ from the views on load (see
  // Reconstruction::serialize, version >= 2). A version 1 Track therefore only
  // round-trips inside a Reconstruction archive; serialized on its own it loads
  // with an empty view_ids_.
  template <class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {  // NOLINT
    if (version == 0) {
      ar(is_estimated_,
         view_ids_,
         reference_view_id_,
         inverse_depth_,
         point_,
         color_,
         reference_bearing_,
         reference_descriptor_);
      return;
    }
    ar(is_estimated_,
       reference_view_id_,
       inverse_depth_,
       cereal::binary_data(point_.data(), 4 * sizeof(double)),
       cereal::binary_data(color_.data(), 3 * sizeof(uint8_t)),
       cereal::binary_data(reference_bearing_.data(), 3 * sizeof(double)),
       reference_descriptor_);
    // Version 2 appends the optional position prior: a flag, followed by the
    // prior only if it is set (so tracks without a prior stay compact). Version
    // 1 archives (e.g. existing .recon files) do not contain these fields and
    // load without a prior.
    if (version >= 2) {
      ar(has_position_prior_);
      if (has_position_prior_) {
        ar(cereal::binary_data(position_prior_.data(), 3 * sizeof(double)),
           cereal::binary_data(position_prior_sqrt_information_.data(),
                               9 * sizeof(double)));
      }
    }
  }

  bool is_estimated_;
  std::unordered_set<ViewId> view_ids_;
  ViewId reference_view_id_;
  double inverse_depth_;
  Eigen::Vector4d point_;
  Eigen::Matrix<uint8_t, 3, 1> color_;
  Eigen::Vector3d reference_bearing_;
  Eigen::VectorXf reference_descriptor_;
  bool has_position_prior_;
  Eigen::Vector3d position_prior_;
  Eigen::Matrix3d position_prior_sqrt_information_;
};

}  // namespace theia

CEREAL_CLASS_VERSION(theia::Track, 2);

#endif  // THEIA_SFM_TRACK_H_
