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

  Feature() {}
  Feature(const double x, const double y) { point_ << x, y; }
  Feature(const Eigen::Vector2d &point) : point_(point) {}
  Feature(const Eigen::Vector2d &point, const Eigen::Matrix2d &covariance_)
      : point_(point), covariance_(covariance_) {}

  double x() const { return point_[0]; }
  double y() const { return point_[1]; }

  // make it hashable
  bool operator==(const Feature &o) const {
      return point_.x() == o.point_.x() && point_.y() == o.point_.y();
  }

  bool operator<(const Feature &o) const {
      return point_.x() < o.point_.x() ||
             (point_.x() == o.point_.x() && point_.y() < o.point_.y());
  }

private:
  // Templated method for disk I/O with cereal. This method tells cereal which
  // data members should be used when reading/writing to/from disk.
  friend class cereal::access;
  template <class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {  // NOLINT
    ar(point_, covariance_);
  }
};

} // namespace theia

CEREAL_CLASS_VERSION(theia::Feature, 0);

namespace std
{
    template <>
    struct hash<theia::Feature>
    {
        size_t operator()(const theia::Feature& k) const
        {
            // Compute individual hash values for two data members and combine them using XOR and bit shifting
            return ((hash<double>()(k.point_.x()) ^ (hash<double>()(k.point_.y()) << 1)) >> 1);
        }
    };
}

#endif // THEIA_SFM_FEATURE_H_
