#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "theia/io/io_wrapper.h"
#include "theia/io/bundler_file_reader.h"
#include "theia/io/import_nvm_file.h"
#include "theia/io/populate_image_sizes.h"
#include "theia/io/read_1dsfm.h"
#include "theia/io/read_bundler_files.h"
#include "theia/io/read_keypoints_and_descriptors.h"
#include "theia/io/read_strecha_dataset.h"
#include "theia/io/reconstruction_reader.h"
#include "theia/io/reconstruction_writer.h"
#include "theia/io/sift_binary_file.h"
#include "theia/io/sift_text_file.h"
#include "theia/io/write_bundler_files.h"
#include "theia/io/write_colmap_files.h"
#include "theia/io/write_keypoints_and_descriptors.h"
#include "theia/io/write_nvm_file.h"
#include "theia/io/write_ply_file.h"



namespace py = pybind11;
#include <vector>
#include <iostream>
#include <pybind11/numpy.h>

PYBIND11_MODULE(pytheia_io, m) {

    //BundleFileReader
    py::class_<theia::BundlerFileReader>(m, "BundlerFileReader")
      .def(py::init<std::string, std::string>())
      //.def_property_readonly("NumCameras", &theia::BundlerFileReader::NumCameras)
      //.def_property_readonly("NumPoints", &theia::BundlerFileReader::NumPoints)
      //.def_property_readonly("NumListEntries", &theia::BundlerFileReader::NumListEntries)

      .def("cameras", &theia::BundlerFileReader::cameras)
      .def("points", &theia::BundlerFileReader::points)
      .def("img_entries", &theia::BundlerFileReader::img_entries)
      .def("ParseBundleFile", &theia::BundlerFileReader::ParseBundleFile)
      .def("ParseListsFile", &theia::BundlerFileReader::ParseListsFile)

    ;

    py::class_<theia::BundlerCamera>(m, "BundlerCamera")
      .def(py::init())
      .def_readwrite("translation", &theia::BundlerCamera::translation)
      .def_readwrite("rotation", &theia::BundlerCamera::rotation)
      .def_readwrite("focal_length", &theia::BundlerCamera::focal_length)
      .def_readwrite("radial_coeff_1", &theia::BundlerCamera::radial_coeff_1)
      .def_readwrite("radial_coeff_2", &theia::BundlerCamera::radial_coeff_2)

    ;

    py::class_<theia::FeatureInfo>(m, "FeatureInfo")
      .def(py::init())
      .def_readwrite("camera_index", &theia::FeatureInfo::camera_index)
      .def_readwrite("sift_index", &theia::FeatureInfo::sift_index)
      .def_readwrite("kpt_x", &theia::FeatureInfo::kpt_x)
      .def_readwrite("kpt_y", &theia::FeatureInfo::kpt_y)

    ;

    py::class_<theia::ListImgEntry>(m, "ListImgEntry")
      .def(py::init())
      .def_readwrite("filename", &theia::ListImgEntry::filename)
      .def_readwrite("second_entry", &theia::ListImgEntry::second_entry)
      .def_readwrite("focal_length", &theia::ListImgEntry::focal_length)

    ;

    m.def("ImportNVMFile", theia::ImportNVMFileWrapper);
    m.def("PopulateImageSizesAndPrincipalPoints", theia::PopulateImageSizesAndPrincipalPointsWrapper);
    m.def("Read1DSFM", theia::Read1DSFMWrapper);
    m.def("ReadBundlerFiles", theia::ReadBundlerFilesWrapper);
    m.def("ReadKeypointsAndDescriptors", theia::ReadKeypointsAndDescriptorsWrapper);

    m.def("ReadStrechaDataset", theia::ReadStrechaDatasetWrapper);
    m.def("ReadReconstruction", theia::ReadReconstructionWrapper);
    m.def("WriteReconstruction", theia::WriteReconstruction);
    m.def("WriteSiftKeyBinaryFile", theia::WriteSiftKeyBinaryFile);
    m.def("ReadSiftKeyBinaryFile", theia::ReadSiftKeyBinaryFileWrapper);
    m.def("ReadSiftKeyTextFile", theia::ReadSiftKeyTextFileWrapper);
    m.def("WriteBundlerFiles", theia::WriteBundlerFiles);
    m.def("WriteColmapFiles", theia::WriteColmapFiles);
    m.def("WriteKeypointsAndDescriptors", theia::WriteKeypointsAndDescriptors);
    m.def("WriteNVMFile", theia::WriteNVMFile);
    m.def("WritePlyFile", theia::WritePlyFile);
}
