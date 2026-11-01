// Copyright (C) 2014 The Regents of the University of California (Regents).
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

// edited by Steffen Urban (urbste@gmail.com), April 2021

#ifndef THEIA_SFM_FEATURE_H_
#define THEIA_SFM_FEATURE_H_

#include <Eigen/Core>
#include <cereal/cereal.hpp>
#include <cstdint>
#include <functional>
#include <utility>

#include "theia/io/eigen_serializable.h"
#include "theia/util/hash.h"

namespace theia {

class Feature {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  //! 2D point
  Eigen::Vector2d point_ = Eigen::Vector2d::Zero();
  //! 2D standard deviation
  Eigen::Matrix2d covariance_ = Eigen::Matrix2d::Identity();

  //! depth cue, some features might have a depth cue from an RGB-D image
  double depth_prior_ = 0.0;

  //! depth prior variance
  double depth_prior_variance_ = 1.0;

  Feature() {}
  Feature(const double x, const double y) { point_ << x, y; }
  Feature(const double x, const double y, const double depth_prior) {
    point_ << x, y;
    depth_prior_ = depth_prior;
  }
  Feature(const Eigen::Vector2d& point) : point_(point) {}
  Feature(const Eigen::Vector2d& point, double depth_prior)
      : point_(point), depth_prior_(depth_prior) {}
  Feature(const Eigen::Vector2d& point, const Eigen::Matrix2d& covariance_)

      : point_(point), covariance_(covariance_) {}
  Feature(const Eigen::Vector2d& point,
          const Eigen::Matrix2d& covariance_,
          const double depth_prior,
          const double depth_prior_variance)
      : point_(point),
        covariance_(covariance_),
        depth_prior_(depth_prior),
        depth_prior_variance_(depth_prior_variance) {}

  double x() const { return point_[0]; }
  double y() const { return point_[1]; }

  double depth_prior() const { return depth_prior_; }
  double depth_prior_variance() const { return depth_prior_variance_; }

  // make it hashable
  bool operator==(const Feature& o) const {
    return point_.x() == o.point_.x() && point_.y() == o.point_.y();
  }
  
  // Define the inequality operator
  bool operator!=(const Feature& o) const {
    return !(*this == o); // Use the equality operator
  }

  bool operator<(const Feature& o) const {
    return point_.x() < o.point_.x() ||
           (point_.x() == o.point_.x() && point_.y() < o.point_.y());
  }

 private:
  // Templated method for disk I/O with cereal. This method tells cereal which
  // data members should be used when reading/writing to/from disk.
  friend class cereal::access;
  // Version 1 ("compact") layout: a flags byte followed by only the fields that
  // differ from their defaults. Coordinates are stored as float when that is
  // exact. Version 0 (legacy) is unchanged.
  template <class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {  // NOLINT
    if (version == 0) {
      ar(point_, covariance_, depth_prior_, depth_prior_variance_);
      return;
    }
    SerializeCompact(ar);
  }

  static constexpr std::uint8_t kPointFloat = 1 << 0;   // point stored as float
  static constexpr std::uint8_t kCovariance = 1 << 1;   // covariance_ present
  static constexpr std::uint8_t kDepthPrior = 1 << 2;   // depth prior present

  template <class Archive>
  void SerializeCompact(Archive& ar) {
    std::uint8_t flags = 0;
    if constexpr (Archive::is_saving::value) {
      const double x = point_[0];
      const double y = point_[1];
      const bool point_is_float =
          static_cast<double>(static_cast<float>(x)) == x &&
          static_cast<double>(static_cast<float>(y)) == y;
      const bool has_covariance =
          covariance_(0, 0) != 1.0 || covariance_(0, 1) != 0.0 ||
          covariance_(1, 0) != 0.0 || covariance_(1, 1) != 1.0;
      const bool has_depth_prior =
          depth_prior_ != 0.0 || depth_prior_variance_ != 1.0;
      if (point_is_float) flags |= kPointFloat;
      if (has_covariance) flags |= kCovariance;
      if (has_depth_prior) flags |= kDepthPrior;
    }
    ar(flags);

    if (flags & kPointFloat) {
      float x = 0.0f, y = 0.0f;
      if constexpr (Archive::is_saving::value) {
        x = static_cast<float>(point_[0]);
        y = static_cast<float>(point_[1]);
      }
      ar(x, y);
      if constexpr (Archive::is_loading::value) {
        point_ << static_cast<double>(x), static_cast<double>(y);
      }
    } else {
      double x = point_[0], y = point_[1];
      ar(x, y);
      if constexpr (Archive::is_loading::value) {
        point_ << x, y;
      }
    }

    if (flags & kCovariance) {
      ar(cereal::binary_data(covariance_.data(), 4 * sizeof(double)));
    } else if constexpr (Archive::is_loading::value) {
      covariance_ = Eigen::Matrix2d::Identity();
    }

    if (flags & kDepthPrior) {
      ar(depth_prior_, depth_prior_variance_);
    } else if constexpr (Archive::is_loading::value) {
      depth_prior_ = 0.0;
      depth_prior_variance_ = 1.0;
    }
  }
};

}  // namespace theia

CEREAL_CLASS_VERSION(theia::Feature, 1);

namespace std {
template <>
struct hash<theia::Feature> {
  size_t operator()(const theia::Feature& k) const {
    // Compute individual hash values for two data members and combine them
    // using XOR and bit shifting
    return (
        (hash<double>()(k.point_.x()) ^ (hash<double>()(k.point_.y()) << 1)) >>
        1);
  }
};
}  // namespace std

#endif  // THEIA_SFM_FEATURE_H_
