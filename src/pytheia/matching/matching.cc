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
// Author: Steffen Urban (urbse@googlemail.com), Shengyu Yin

#include "pytheia/matching/matching.h"

#include <nanobind/eigen/dense.h>

#include <Eigen/Core>
#include <iostream>
#include <vector>

#include "theia/matching/create_feature_matcher.h"
#include "theia/matching/feature_correspondence.h"
#include "theia/matching/feature_matcher_options.h"
#include "theia/matching/graph_match.h"
#include "theia/matching/image_pair_match.h"
#include "theia/matching/indexed_feature_match.h"
#include "theia/sfm/feature.h"

namespace nb = nanobind;

namespace pytheia {
namespace matching {

void pytheia_matching_classes(nb::module_& m) {
  m.def("GraphMatch", &theia::GraphMatch);

  nb::class_<theia::ImagePairMatch>(m, "ImagePairMatch")
      .def(nb::init<>())
      .def_rw("image1", &theia::ImagePairMatch::image1)
      .def_rw("image2", &theia::ImagePairMatch::image2)
      .def_rw("twoview_info", &theia::ImagePairMatch::twoview_info)
      .def_rw("correspondences", &theia::ImagePairMatch::correspondences);

  nb::class_<theia::FeatureMatcherOptions>(m, "FeatureMatcherOptions")
      .def(nb::init<>())
      .def_rw("num_threads", &theia::FeatureMatcherOptions::num_threads)
      .def_rw("keep_only_symmetric_matches",
                     &theia::FeatureMatcherOptions::keep_only_symmetric_matches)
      .def_rw("use_lowes_ratio",
                     &theia::FeatureMatcherOptions::use_lowes_ratio)
      .def_rw("lowes_ratio", &theia::FeatureMatcherOptions::lowes_ratio)
      .def_rw(
          "perform_geometric_verification",
          &theia::FeatureMatcherOptions::perform_geometric_verification)
      .def_rw("min_num_feature_matches",
                     &theia::FeatureMatcherOptions::min_num_feature_matches)
      .def_rw(
          "geometric_verification_options",
          &theia::FeatureMatcherOptions::geometric_verification_options);

  nb::class_<theia::IndexedFeatureMatch>(m, "IndexedFeatureMatch")
      .def(nb::init<>())
      .def(nb::init<int, int, float>())
      .def_rw("feature1_ind", &theia::IndexedFeatureMatch::feature1_ind)
      .def_rw("feature2_ind", &theia::IndexedFeatureMatch::feature2_ind)
      .def_rw("distance", &theia::IndexedFeatureMatch::distance);

  nb::class_<theia::FeatureCorrespondence>(m, "FeatureCorrespondence")
      .def(nb::init<>())
      .def(nb::init<theia::Feature, theia::Feature>())
      .def_rw("feature1", &theia::FeatureCorrespondence::feature1)
      .def_rw("feature2", &theia::FeatureCorrespondence::feature2);

  nb::enum_<theia::MatchingStrategy>(m, "MatchingStrategy")
      .value("GLOBAL", theia::MatchingStrategy::BRUTE_FORCE)
      .value("INCREMENTAL", theia::MatchingStrategy::BRUTE_FORCE)
      .export_values();
}

void pytheia_matching(nb::module_& m) {
  nb::module_ m_submodule = m.def_submodule("matching");
  pytheia_matching_classes(m_submodule);
}

}  // namespace matching
}  // namespace pytheia
