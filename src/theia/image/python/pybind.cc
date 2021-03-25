#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>


#include "theia/image/image.h"
#include "theia/image/keypoint_detector/keypoint.h"
#include "theia/image/keypoint_detector/keypoint_detector.h"
#include "theia/image/keypoint_detector/sift_parameters.h"
#include "theia/image/keypoint_detector/sift_detector.h"


#include "theia/image/descriptor/create_descriptor_extractor.h"
#include "theia/image/descriptor/descriptor_extractor.h"
#include "theia/image/descriptor/akaze_descriptor.h"
#include "theia/image/descriptor/sift_descriptor.h"

namespace py = pybind11;
#include <vector>
#include <iostream>
#include <pybind11/numpy.h>


PYBIND11_MODULE(pytheia_image, m) {

    // //image
    // py::class_<theia::FloatImage>(m, "FloatImage")
    //   .def(py::init())
    //   .def(py::init<int, int, int>())
    //   .def(py::init<theia::FloatImage>())
    //   .def(py::init<std::string>())
    //   .def_property_readonly("Rows", &theia::FloatImage::Rows)
    //   .def_property_readonly("Cols", &theia::FloatImage::Cols)
    //   .def_property_readonly("Width", &theia::FloatImage::Width)
    //   .def_property_readonly("Height", &theia::FloatImage::Height)
    //   .def_property_readonly("Channels", &theia::FloatImage::Channels)
    //   //.def("GetRowCol", &theia::FloatImage::GetRowCol)
    //   //.def("SetRowCol", &theia::FloatImage::SetRowCol)
    //   //.def("GetXY", &theia::FloatImage::GetXY)
    //   //.def("SetXY", &theia::FloatImage::SetXY)
    //   .def("Read", &theia::FloatImage::Read)
    //   .def("Write", &theia::FloatImage::Write)
    //   .def("ComputeGradient", &theia::FloatImage::ComputeGradient)
    //   .def("ComputeGradientX", &theia::FloatImage::ComputeGradientX)
    //   .def("ComputeGradientY", &theia::FloatImage::ComputeGradientY)
    //   .def("ApproximateGaussianBlur", &theia::FloatImage::ApproximateGaussianBlur)
    //   .def("MedianFilter", &theia::FloatImage::MedianFilter)

    // ;

    //keypoint
    py::class_<theia::Keypoint>(m, "Keypoint")
      .def(py::init())
      .def(py::init<double, double, theia::Keypoint::KeypointType>())
    ;

    py::enum_<theia::Keypoint::KeypointType>(m, "KeypointType")
      .value("INVALID", theia::Keypoint::KeypointType::INVALID)
      .value("OTHER", theia::Keypoint::KeypointType::OTHER)
      .value("SIFT", theia::Keypoint::KeypointType::SIFT)
      .value("AKAZE", theia::Keypoint::KeypointType::AKAZE)
      .export_values()
    ;

    // //sift parameters
    // py::class_<theia::SiftParameters>(m, "SiftParameters")
    //   .def(py::init())
    //   .def(py::init<int, int, int>())
    //   .def(py::init<int, int, int, float, float>())
    // ;

    // // abstract class
    // py::class_<theia::KeypointDetector>(m, "KeypointDetector")
    //   .def("Initialize", &theia::KeypointDetector::Initialize)
    // ;

    // py::class_<theia::SiftDetector, theia::KeypointDetector>(m, "SiftDetector")
    //   .def(py::init())
    //   .def(py::init<int, int, int>())
    //   .def(py::init<theia::SiftParameters>())
    //   .def("DetectKeypoints", &theia::SiftDetector::DetectKeypointsWrapper)

    // ;


    // // descriptor

    // py::enum_<theia::DescriptorExtractorType>(m, "DescriptorExtractorType")
    //   .value("SIFT", theia::DescriptorExtractorType::SIFT)
    //   .value("AKAZE", theia::DescriptorExtractorType::AKAZE)
    //   .export_values()
    // ;

    // py::enum_<theia::FeatureDensity>(m, "FeatureDensity")
    //   .value("SPARSE", theia::FeatureDensity::SPARSE)
    //   .value("NORMAL", theia::FeatureDensity::NORMAL)
    //   .value("DENSE", theia::FeatureDensity::DENSE)
    //   .export_values()
    // ;

    // // abstract class
    // py::class_<theia::DescriptorExtractor>(m, "DescriptorExtractor")
    //   .def("Initialize", &theia::DescriptorExtractor::Initialize)
    //   .def("ComputeDescriptors", &theia::DescriptorExtractor::ComputeDescriptorsWrapper)
    // ;

    // py::class_<theia::AkazeParameters>(m, "AkazeParameters")
    //   .def(py::init())

    //   .def_readwrite("maximum_octave_levels", &theia::AkazeParameters::maximum_octave_levels)
    //   .def_readwrite("num_sublevels", &theia::AkazeParameters::num_sublevels)
    //   .def_readwrite("hessian_threshold", &theia::AkazeParameters::hessian_threshold)

    // ;

    // py::class_<theia::AkazeDescriptorExtractor, theia::DescriptorExtractor>(m, "AkazeDescriptorExtractor")
    //   .def(py::init<theia::AkazeParameters>())
    //   .def("ComputeDescriptor", &theia::AkazeDescriptorExtractor::ComputeDescriptorWrapper)
    //   .def("DetectAndExtractDescriptors", &theia::AkazeDescriptorExtractor::DetectAndExtractDescriptorsWrapper)

    // ;

    // py::class_<theia::SiftDescriptorExtractor, theia::DescriptorExtractor>(m, "SiftDescriptorExtractor")
    //   .def(py::init<>())
    //   .def(py::init<int, int, int>())
    //   .def(py::init<theia::SiftParameters>())
    //   .def("ComputeDescriptor", &theia::SiftDescriptorExtractor::ComputeDescriptorWrapper)
    //   .def("ComputeDescriptors", &theia::SiftDescriptorExtractor::ComputeDescriptorsWrapper)
    //   .def("DetectAndExtractDescriptors", &theia::SiftDescriptorExtractor::DetectAndExtractDescriptorsWrapper)
    //   //.def_static("ConvertToRootSift", &theia::SiftDescriptorExtractor::ConvertToRootSift)
    // ;
}
