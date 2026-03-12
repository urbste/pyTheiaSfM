// Copyright (C) 2015 The Regents of the University of California (Regents).
// Feature matching support removed; only FeatureCorrespondence and ImagePairMatch
// types are exposed for use with pre-computed matches (e.g. from view graph I/O).

#include "pytheia/matching/matching.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "theia/matching/feature_correspondence.h"
#include "theia/matching/image_pair_match.h"
#include "theia/sfm/feature.h"

namespace py = pybind11;

namespace pytheia {
namespace matching {

void pytheia_matching_classes(py::module& m) {
  py::class_<theia::FeatureCorrespondence>(m, "FeatureCorrespondence")
      .def(py::init<>())
      .def(py::init<theia::Feature, theia::Feature>())
      .def_readwrite("feature1", &theia::FeatureCorrespondence::feature1)
      .def_readwrite("feature2", &theia::FeatureCorrespondence::feature2);

  py::class_<theia::ImagePairMatch>(m, "ImagePairMatch")
      .def(py::init<>())
      .def_readwrite("image1", &theia::ImagePairMatch::image1)
      .def_readwrite("image2", &theia::ImagePairMatch::image2)
      .def_readwrite("twoview_info", &theia::ImagePairMatch::twoview_info)
      .def_readwrite("correspondences", &theia::ImagePairMatch::correspondences);
}

void pytheia_matching(py::module& m) {
  py::module m_submodule = m.def_submodule("matching");
  pytheia_matching_classes(m_submodule);
}

}  // namespace matching
}  // namespace pytheia
