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

#include "theia/sfm/view.h"

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera_intrinsics_prior.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/types.h"
#include "theia/util/map_util.h"

namespace theia {
using Vector2d = Eigen::Vector2d;

View::View()
    : name_(""),
      is_estimated_(false),
      timestamp_(0.0),
      has_position_prior_(false),
      has_gravity_prior_(false) {
  position_prior_.setZero();
  position_prior_sqrt_information_.setIdentity();
  gravity_prior_.setZero();
  gravity_prior_sqrt_information_.setIdentity();
}

View::View(const std::string& name)
    : name_(name),
      is_estimated_(false),
      timestamp_(0.0),
      has_position_prior_(false),
      has_gravity_prior_(false) {
  position_prior_.setZero();
  position_prior_sqrt_information_.setIdentity();
  gravity_prior_.setZero();
  gravity_prior_sqrt_information_.setIdentity();
}

View::View(const std::string& name, const double timestamp)
    : name_(name),
      is_estimated_(false),
      timestamp_(timestamp),
      has_position_prior_(false),
      has_gravity_prior_(false) {
  position_prior_.setZero();
  position_prior_sqrt_information_.setIdentity();
  gravity_prior_.setZero();
  gravity_prior_sqrt_information_.setIdentity();
}

const std::string& View::Name() const { return name_; }

void View::SetEstimated(bool is_estimated) { is_estimated_ = is_estimated; }

bool View::IsEstimated() const { return is_estimated_; }

const class Camera& View::Camera() const { return camera_; }

class Camera* View::MutableCamera() {
  return &camera_;
}

const struct CameraIntrinsicsPrior& View::CameraIntrinsicsPrior() const {
  return camera_intrinsics_prior_;
}

struct CameraIntrinsicsPrior* View::MutableCameraIntrinsicsPrior() {
  return &camera_intrinsics_prior_;
}

void View::SetCameraIntrinsicsPrior(struct CameraIntrinsicsPrior prior) {
  camera_intrinsics_prior_ = prior;
}

int View::NumFeatures() const { return features_.size(); }

std::vector<TrackId> View::TrackIds() const {
  std::vector<TrackId> track_ids;
  track_ids.reserve(features_.size());
  for (const auto& track : features_) {
    track_ids.emplace_back(track.first);
  }
  return track_ids;
}

const Feature* View::GetFeature(const TrackId track_id) const {
  return FindOrNull(features_, track_id);
}

const TrackId View::GetTrack(const Feature& feature) const {
  const TrackId* t_id = FindOrNull(features_to_tracks_, feature);
  if (t_id) {
    return *t_id;
  }
  return kInvalidTrackId;
}

void View::AddFeature(const TrackId track_id, const Feature& feature) {
  features_[track_id] = feature;
  features_to_tracks_[feature] = track_id;
}

void View::UpdateFeature(const TrackId track_id, const Feature& feature) {
    // Check if the track_id exists in features_
    const auto old_feature = FindOrNull(features_, track_id);
    
    // Only proceed if the old feature exists
    if (old_feature) {
        // If the old feature is different from the new feature, update the mappings
        if (*old_feature != feature) {
            // Remove the old feature from features_to_tracks_
            features_to_tracks_.erase(*old_feature);
            // Update the feature for the track_id
            features_[track_id] = feature;
            // Insert the new feature into features_to_tracks_
            features_to_tracks_[feature] = track_id;
        }
    }
}

bool View::RemoveFeature(const TrackId track_id) {
  const auto feature = FindOrNull(features_, track_id);
  if (feature) {
    return features_.erase(track_id) > 0 &&
           features_to_tracks_.erase(*feature) > 0;
  }
  return false;
}

double View::GetTimestamp() const { return timestamp_; }

void View::SetTimestamp(const double timestamp) { timestamp_ = timestamp; }

void View::SetPositionPrior(
    const Eigen::Vector3d& position_prior,
    const Eigen::Matrix3d& position_prior_sqrt_information) {
  position_prior_ = position_prior;
  position_prior_sqrt_information_ = position_prior_sqrt_information;
  has_position_prior_ = true;
}

Eigen::Vector3d View::GetPositionPrior() const { return position_prior_; }

Eigen::Matrix3d View::GetPositionPriorSqrtInformation() const {
  return position_prior_sqrt_information_;
}

bool View::HasPositionPrior() const { return has_gravity_prior_; }

void View::SetGravityPrior(
    const Eigen::Vector3d& gravity_prior,
    const Eigen::Matrix3d& gravity_prior_sqrt_information) {
  gravity_prior_ = gravity_prior;
  gravity_prior_sqrt_information_ = gravity_prior_sqrt_information;
  has_gravity_prior_ = true;
}

Eigen::Vector3d View::GetGravityPrior() const { return gravity_prior_; }

Eigen::Matrix3d View::GetGravityPriorSqrtInformation() const {
  return gravity_prior_sqrt_information_;
}

bool View::HasGravityPrior() const { return has_gravity_prior_; }

}  // namespace theia
