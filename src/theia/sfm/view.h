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

#ifndef THEIA_SFM_VIEW_H_
#define THEIA_SFM_VIEW_H_

#include <cereal/access.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>
#include <stdint.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera_intrinsics_prior.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/types.h"

namespace theia {

// A View contains high level information about an image that has been
// captured. This includes the name, EXIF metadata, and track information that
// is found through feature matching.
class View {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  View();
  explicit View(const std::string& name);
  explicit View(const std::string& name, const double timestamp);

  ~View() {}

  const std::string& Name() const;

  void SetEstimated(bool is_estimated);
  bool IsEstimated() const;

  const class Camera& Camera() const;
  class Camera* MutableCamera();

  const struct CameraIntrinsicsPrior& CameraIntrinsicsPrior() const;
  struct CameraIntrinsicsPrior* MutableCameraIntrinsicsPrior();
  void SetCameraIntrinsicsPrior(struct CameraIntrinsicsPrior prior);

  int NumFeatures() const;

  std::vector<TrackId> TrackIds() const;

  const Feature* GetFeature(const TrackId track_id) const;

  const TrackId GetTrack(const Feature& feature) const;

  void AddFeature(const TrackId track_id, const Feature& feature);
  void UpdateFeature(const TrackId track_id, const Feature& feature);
  bool RemoveFeature(const TrackId track_id);

  double GetTimestamp() const;
  void SetTimestamp(const double timestamp);

  // Prior
  void SetOrientationPrior(const Eigen::Vector3d& orientation_prior,
                        const Eigen::Matrix3d& orientation_prior_information);
  Eigen::Vector3d GetOrientationPrior() const;
  Eigen::Matrix3d GetOrientationPriorSqrtInformation() const;
  bool HasOrientationPrior() const;

  void SetPositionPrior(const Eigen::Vector3d& position_prior,
                        const Eigen::Matrix3d& position_prior_information);
  Eigen::Vector3d GetPositionPrior() const;
  Eigen::Matrix3d GetPositionPriorSqrtInformation() const;

  bool HasPositionPrior() const;

  void SetGravityPrior(const Eigen::Vector3d& gravity_prior,
                       const Eigen::Matrix3d& gravity_prior_information);
  Eigen::Vector3d GetGravityPrior() const;
  Eigen::Matrix3d GetGravityPriorSqrtInformation() const;
  bool HasGravityPrior() const;

 private:
  // Templated method for disk I/O with cereal. This method tells cereal which
  // data members should be used when reading/writing to/from disk.
  friend class cereal::access;
  template <class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {  // NOLINT
    ar(name_,
       timestamp_,
       is_estimated_,
       camera_,
       camera_intrinsics_prior_,
       features_,
       features_to_tracks_,
       position_prior_,
       position_prior_sqrt_information_,
       has_position_prior_,
       gravity_prior_,
       gravity_prior_sqrt_information_,
       has_gravity_prior_,
       orientation_prior_,
       orientation_prior_sqrt_information_,
       has_orientation_prior_);
  }

  std::string name_;
  double timestamp_;
  bool is_estimated_;
  class Camera camera_;
  struct CameraIntrinsicsPrior camera_intrinsics_prior_;
  std::unordered_map<TrackId, Feature> features_;
  std::unordered_map<Feature, TrackId> features_to_tracks_;

  // A prior on an absolute position (e.g. GPS)
  Eigen::Vector3d position_prior_;
  Eigen::Matrix3d position_prior_sqrt_information_;
  bool has_position_prior_;

  // A prior on gravity (in the image coordinate system, z-forward, y-down,
  // x-right)
  Eigen::Vector3d gravity_prior_;
  Eigen::Matrix3d gravity_prior_sqrt_information_;
  bool has_gravity_prior_;

  // A prior on orientation (world to camera transformation)
  Eigen::Vector3d orientation_prior_;
  Eigen::Matrix3d orientation_prior_sqrt_information_;
  bool has_orientation_prior_;
};

}  // namespace theia

CEREAL_CLASS_VERSION(theia::View, 0);

#endif  // THEIA_SFM_VIEW_H_
