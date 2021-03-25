#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <pybind11/numpy.h>

#include "theia/matching/global_descriptor_extractor.h"
#include "theia/matching/fisher_vector_extractor.h"
#include "theia/sfm/feature.h"
#include "theia/matching/keypoints_and_descriptors.h"
#include "theia/matching/indexed_feature_match.h"
#include "theia/matching/feature_matcher.h"
#include "theia/matching/feature_matcher_options.h"
#include "theia/matching/features_and_matches_database.h"
#include "theia/sfm/two_view_match_geometric_verification.h"
#include "theia/matching/brute_force_feature_matcher.h"
#include "theia/matching/cascade_hashing_feature_matcher.h"
#include "theia/matching/cascade_hasher.h"
#include "theia/matching/rocksdb_features_and_matches_database.h"
//#include "theia/matching/local_features_and_matches_database.h"
#include "theia/matching/in_memory_features_and_matches_database.h"
#include "theia/matching/create_feature_matcher.h"
namespace py = pybind11;


PYBIND11_MODULE(pytheia_matching, m) {


    //FeaturesAndMatchesDatabase
    py::class_<theia::FeaturesAndMatchesDatabase /*, theia::PyFeaturesAndMatchesDatabase */>(m, "FeaturesAndMatchesDatabase")
      //.def(py::init<>())
      //.def("ContainsCameraIntrinsicsPrior", &theia::FeaturesAndMatchesDatabase::ContainsCameraIntrinsicsPrior)
    ;

    //RocksDbFeaturesAndMatchesDatabase

    py::class_<theia::RocksDbFeaturesAndMatchesDatabase, theia::FeaturesAndMatchesDatabase>(m, "RocksDbFeaturesAndMatchesDatabase")
      .def(py::init<std::string>())
      .def("ContainsCameraIntrinsicsPrior", &theia::RocksDbFeaturesAndMatchesDatabase::ContainsCameraIntrinsicsPrior)
      .def("GetCameraIntrinsicsPrior", &theia::RocksDbFeaturesAndMatchesDatabase::GetCameraIntrinsicsPrior)
      .def("PutCameraIntrinsicsPrior", &theia::RocksDbFeaturesAndMatchesDatabase::PutCameraIntrinsicsPrior)
      .def("ImageNamesOfCameraIntrinsicsPriors", &theia::RocksDbFeaturesAndMatchesDatabase::ImageNamesOfCameraIntrinsicsPriors)
      .def("NumCameraIntrinsicsPrior", &theia::RocksDbFeaturesAndMatchesDatabase::NumCameraIntrinsicsPrior)
      .def("ContainsFeatures", &theia::RocksDbFeaturesAndMatchesDatabase::ContainsFeatures)
      .def("GetFeatures", &theia::RocksDbFeaturesAndMatchesDatabase::GetFeatures)
      .def("PutFeatures", &theia::RocksDbFeaturesAndMatchesDatabase::PutFeatures)
      .def("NumImages", &theia::RocksDbFeaturesAndMatchesDatabase::NumImages)
      .def("GetImagePairMatch", &theia::RocksDbFeaturesAndMatchesDatabase::GetImagePairMatch)
      .def("PutImagePairMatch", &theia::RocksDbFeaturesAndMatchesDatabase::PutImagePairMatch)
      .def("ImageNamesOfMatches", &theia::RocksDbFeaturesAndMatchesDatabase::ImageNamesOfMatches)
      .def("NumMatches", &theia::RocksDbFeaturesAndMatchesDatabase::NumMatches)
      .def("RemoveAllMatches", &theia::RocksDbFeaturesAndMatchesDatabase::RemoveAllMatches)
      .def("ImageNamesOfFeatures", &theia::RocksDbFeaturesAndMatchesDatabase::ImageNamesOfFeatures)

    ;

    //InMemoryFeaturesAndMatchesDatabase

    py::class_<theia::InMemoryFeaturesAndMatchesDatabase, theia::FeaturesAndMatchesDatabase>(m, "InMemoryFeaturesAndMatchesDatabase")
      .def(py::init<>())
      .def("ContainsFeatures", &theia::InMemoryFeaturesAndMatchesDatabase::ContainsFeatures)
      .def("GetFeatures", &theia::InMemoryFeaturesAndMatchesDatabase::GetFeatures)
      .def("PutFeatures", &theia::InMemoryFeaturesAndMatchesDatabase::PutFeatures)
      .def("ImageNamesOfFeatures", &theia::InMemoryFeaturesAndMatchesDatabase::ImageNamesOfFeatures)
      .def("NumImages", &theia::InMemoryFeaturesAndMatchesDatabase::NumImages)
      .def("GetImagePairMatch", &theia::InMemoryFeaturesAndMatchesDatabase::GetImagePairMatch)
      .def("PutImagePairMatch", &theia::InMemoryFeaturesAndMatchesDatabase::PutImagePairMatch)
      .def("NumMatches", &theia::InMemoryFeaturesAndMatchesDatabase::NumMatches)
      .def("PutCameraIntrinsicsPrior", &theia::InMemoryFeaturesAndMatchesDatabase::PutCameraIntrinsicsPrior)
      .def("GetCameraIntrinsicsPrior", &theia::InMemoryFeaturesAndMatchesDatabase::GetCameraIntrinsicsPrior)
      .def("NumCameraIntrinsicsPrior", &theia::InMemoryFeaturesAndMatchesDatabase::NumCameraIntrinsicsPrior)
      .def("ImageNamesOfCameraIntrinsicsPriors", &theia::InMemoryFeaturesAndMatchesDatabase::ImageNamesOfCameraIntrinsicsPriors)
      .def("ImageNamesOfMatches", &theia::InMemoryFeaturesAndMatchesDatabase::ImageNamesOfMatches)
      .def("ContainsCameraIntrinsicsPrior", &theia::InMemoryFeaturesAndMatchesDatabase::ContainsCameraIntrinsicsPrior)

    ;
    py::class_<theia::ImagePairMatch>(m, "ImagePairMatch")
      .def(py::init<>())
      .def_readwrite("image1", &theia::ImagePairMatch::image1)
      .def_readwrite("image2", &theia::ImagePairMatch::image2)
      .def_readwrite("twoview_info", &theia::ImagePairMatch::twoview_info)
      .def_readwrite("correspondences", &theia::ImagePairMatch::correspondences)

    ;

    /*
        //LocalFeaturesAndMatchesDatabase

        py::class_<theia::LocalFeaturesAndMatchesDatabase, theia::FeaturesAndMatchesDatabase>(m, "LocalFeaturesAndMatchesDatabase")
          .def(py::init<std::string>())
          .def("ContainsFeatures", &theia::LocalFeaturesAndMatchesDatabase::ContainsFeatures)
          .def("GetFeatures", &theia::LocalFeaturesAndMatchesDatabase::GetFeatures)
          .def("PutFeatures", &theia::LocalFeaturesAndMatchesDatabase::PutFeatures)
          .def("ImageNamesOfFeatures", &theia::LocalFeaturesAndMatchesDatabase::ImageNamesOfFeatures)
          .def("NumImages", &theia::LocalFeaturesAndMatchesDatabase::NumImages)
          .def("GetImagePairMatch", &theia::LocalFeaturesAndMatchesDatabase::GetImagePairMatch)
          .def("PutImagePairMatch", &theia::LocalFeaturesAndMatchesDatabase::PutImagePairMatch)
          .def("NumMatches", &theia::LocalFeaturesAndMatchesDatabase::NumMatches)
          .def("SaveMatchesAndGeometry", &theia::LocalFeaturesAndMatchesDatabase::SaveMatchesAndGeometry)

        ;

    */
    //FeatureMatcherOptions
    py::class_<theia::FeatureMatcherOptions>(m, "FeatureMatcherOptions")
      .def(py::init<>())
      .def_readwrite("num_threads", &theia::FeatureMatcherOptions::num_threads)
      .def_readwrite("keep_only_symmetric_matches", &theia::FeatureMatcherOptions::keep_only_symmetric_matches)
      .def_readwrite("use_lowes_ratio", &theia::FeatureMatcherOptions::use_lowes_ratio)
      .def_readwrite("lowes_ratio", &theia::FeatureMatcherOptions::lowes_ratio)
      .def_readwrite("perform_geometric_verification", &theia::FeatureMatcherOptions::perform_geometric_verification)
      .def_readwrite("min_num_feature_matches", &theia::FeatureMatcherOptions::min_num_feature_matches)
      .def_readwrite("geometric_verification_options", &theia::FeatureMatcherOptions::geometric_verification_options)

    ;

    //KeypointsAndDescriptors
    py::class_<theia::KeypointsAndDescriptors>(m, "KeypointsAndDescriptors")
      .def(py::init<>())
      .def_readwrite("image_name", &theia::KeypointsAndDescriptors::image_name)
      .def_readwrite("keypoints", &theia::KeypointsAndDescriptors::keypoints)
      .def_readwrite("descriptors", &theia::KeypointsAndDescriptors::descriptors)
    ;

    //IndexedFeatureMatch
    py::class_<theia::IndexedFeatureMatch>(m, "IndexedFeatureMatch")
      .def(py::init<>())
      .def(py::init<int, int, float>())
      .def_readwrite("feature1_ind", &theia::IndexedFeatureMatch::feature1_ind)
      .def_readwrite("feature2_ind", &theia::IndexedFeatureMatch::feature2_ind)
      .def_readwrite("distance", &theia::IndexedFeatureMatch::distance)
    ;

    py::class_<theia::FeatureCorrespondence>(m, "FeatureCorrespondence")
      .def(py::init<>())
      .def(py::init<theia::Feature, theia::Feature>())
      .def_readwrite("feature1", &theia::FeatureCorrespondence::feature1)
      .def_readwrite("feature2", &theia::FeatureCorrespondence::feature2)

    ;

    //GlobalDescriptorExtractor
    py::class_<theia::GlobalDescriptorExtractor>(m, "GlobalDescriptorExtractor")
      //abstract class in the constructor

    ;

    //FisherVectorExtractor Options
    py::class_<theia::FisherVectorExtractor::Options>(m, "FisherVectorExtractorOptions")
      .def(py::init<>())
      .def_readwrite("num_threads", &theia::FisherVectorExtractor::Options::num_gmm_clusters)
      .def_readwrite("keep_only_symmetric_matches", &theia::FisherVectorExtractor::Options::max_num_features_for_training)

    ;



    //FisherVectorExtractor
    py::class_<theia::FisherVectorExtractor, theia::GlobalDescriptorExtractor>(m, "FisherVectorExtractor")
      .def(py::init<theia::FisherVectorExtractor::Options>())
      .def("AddFeaturesForTraining", &theia::FisherVectorExtractor::AddFeaturesForTraining)
      .def("Train", &theia::FisherVectorExtractor::Train)
      .def("ExtractGlobalDescriptor", &theia::FisherVectorExtractor::ExtractGlobalDescriptor)

    ;

    //FeatureMatcher
    py::class_<theia::FeatureMatcher>(m, "FeatureMatcher")
      //abstract class in the constructor
      //.def(py::init<theia::FeatureMatcherOptions, &theia::FeaturesAndMatchesDatabase>())
      .def("AddImages", (void (theia::FeatureMatcher::*)(const std::vector<std::string>& )) &theia::FeatureMatcher::AddImages, py::return_value_policy::reference_internal)
      .def("AddImage", (void (theia::FeatureMatcher::*)(const std::string& )) &theia::FeatureMatcher::AddImage, py::return_value_policy::reference_internal)
      .def("MatchImages", &theia::FeatureMatcher::MatchImages)
      .def("SetImagePairsToMatch", &theia::FeatureMatcher::SetImagePairsToMatch)

    ;

    //BruteForceFeatureMatcher
    py::class_<theia::BruteForceFeatureMatcher, theia::FeatureMatcher>(m, "BruteForceFeatureMatcher")
      .def(py::init<theia::FeatureMatcherOptions, theia::FeaturesAndMatchesDatabase*>())
      //abstract class in the constructor
      //.def(py::init<theia::FeatureMatcherOptions, theia::FeaturesAndMatchesDatabase>())


    ;

    //CascadeHashingFeatureMatcher
    py::class_<theia::CascadeHashingFeatureMatcher, theia::FeatureMatcher>(m, "CascadeHashingFeatureMatcher")
      //abstract class in the constructor
      .def(py::init<theia::FeatureMatcherOptions, theia::FeaturesAndMatchesDatabase*>())
      .def("AddImages", (void (theia::CascadeHashingFeatureMatcher::*)(const std::vector<std::string>& )) &theia::CascadeHashingFeatureMatcher::AddImages, py::return_value_policy::reference_internal)
      .def("AddImage", (void (theia::CascadeHashingFeatureMatcher::*)(const std::string& )) &theia::CascadeHashingFeatureMatcher::AddImage, py::return_value_policy::reference_internal)


    ;

    py::enum_<theia::MatchingStrategy>(m, "MatchingStrategy")
      .value("GLOBAL", theia::MatchingStrategy::BRUTE_FORCE)
      .value("INCREMENTAL", theia::MatchingStrategy::CASCADE_HASHING)
      .export_values()
    ;



}
