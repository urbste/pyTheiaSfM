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

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <iostream>
#include <pybind11/numpy.h>
#include <vector>

#include "theia/matching/create_feature_matcher.h"
#include "theia/matching/feature_correspondence.h"
#include "theia/matching/feature_matcher_options.h"
#include "theia/matching/features_and_matches_database.h"
#include "theia/matching/graph_match.h"
#include "theia/matching/image_pair_match.h"
#include "theia/matching/in_memory_features_and_matches_database.h"
#include "theia/matching/indexed_feature_match.h"
#include "theia/sfm/feature.h"

namespace py = pybind11;

namespace pytheia {
namespace matching {

void pytheia_matching_classes(py::module& m) {
  py::class_<theia::FeaturesAndMatchesDatabase>(m, "FeaturesAndMatchesDatabase");

  m.def("GraphMatch", &theia::GraphMatch);

  py::class_<theia::InMemoryFeaturesAndMatchesDatabase,
             theia::FeaturesAndMatchesDatabase>(
      m, "InMemoryFeaturesAndMatchesDatabase")
      .def(py::init<>())
      .def("GetImagePairMatch",
           &theia::InMemoryFeaturesAndMatchesDatabase::GetImagePairMatch)
      .def("PutImagePairMatch",
           &theia::InMemoryFeaturesAndMatchesDatabase::PutImagePairMatch)
      .def("NumMatches", &theia::InMemoryFeaturesAndMatchesDatabase::NumMatches)
      .def("PutCameraIntrinsicsPrior",
           &theia::InMemoryFeaturesAndMatchesDatabase::PutCameraIntrinsicsPrior)
      .def("GetCameraIntrinsicsPrior",
           &theia::InMemoryFeaturesAndMatchesDatabase::GetCameraIntrinsicsPrior)
      .def("NumCameraIntrinsicsPrior",
           &theia::InMemoryFeaturesAndMatchesDatabase::NumCameraIntrinsicsPrior)
      .def("ImageNamesOfCameraIntrinsicsPriors",
           &theia::InMemoryFeaturesAndMatchesDatabase::
               ImageNamesOfCameraIntrinsicsPriors)
      .def("ImageNamesOfMatches",
           &theia::InMemoryFeaturesAndMatchesDatabase::ImageNamesOfMatches)
      .def("ContainsCameraIntrinsicsPrior",
           &theia::InMemoryFeaturesAndMatchesDatabase::
               ContainsCameraIntrinsicsPrior);

  py::class_<theia::ImagePairMatch>(m, "ImagePairMatch")
      .def(py::init<>())
      .def_readwrite("image1", &theia::ImagePairMatch::image1)
      .def_readwrite("image2", &theia::ImagePairMatch::image2)
      .def_readwrite("twoview_info", &theia::ImagePairMatch::twoview_info)
      .def_readwrite("correspondences", &theia::ImagePairMatch::correspondences);

  py::class_<theia::FeatureMatcherOptions>(m, "FeatureMatcherOptions")
      .def(py::init<>())
      .def_readwrite("num_threads", &theia::FeatureMatcherOptions::num_threads)
      .def_readwrite("keep_only_symmetric_matches",
                     &theia::FeatureMatcherOptions::keep_only_symmetric_matches)
      .def_readwrite("use_lowes_ratio",
                     &theia::FeatureMatcherOptions::use_lowes_ratio)
      .def_readwrite("lowes_ratio", &theia::FeatureMatcherOptions::lowes_ratio)
      .def_readwrite(
          "perform_geometric_verification",
          &theia::FeatureMatcherOptions::perform_geometric_verification)
      .def_readwrite("min_num_feature_matches",
                     &theia::FeatureMatcherOptions::min_num_feature_matches)
      .def_readwrite(
          "geometric_verification_options",
          &theia::FeatureMatcherOptions::geometric_verification_options);

  py::class_<theia::IndexedFeatureMatch>(m, "IndexedFeatureMatch")
      .def(py::init<>())
      .def(py::init<int, int, float>())
      .def_readwrite("feature1_ind", &theia::IndexedFeatureMatch::feature1_ind)
      .def_readwrite("feature2_ind", &theia::IndexedFeatureMatch::feature2_ind)
      .def_readwrite("distance", &theia::IndexedFeatureMatch::distance);

  py::class_<theia::FeatureCorrespondence>(m, "FeatureCorrespondence")
      .def(py::init<>())
      .def(py::init<theia::Feature, theia::Feature>())
      .def_readwrite("feature1", &theia::FeatureCorrespondence::feature1)
      .def_readwrite("feature2", &theia::FeatureCorrespondence::feature2);

  py::enum_<theia::MatchingStrategy>(m, "MatchingStrategy")
      .value("GLOBAL", theia::MatchingStrategy::BRUTE_FORCE)
      .value("INCREMENTAL", theia::MatchingStrategy::BRUTE_FORCE)
      .export_values();
}

void pytheia_matching(py::module& m) {
  py::module m_submodule = m.def_submodule("matching");
  pytheia_matching_classes(m_submodule);
}

}  // namespace matching
}  // namespace pytheia
