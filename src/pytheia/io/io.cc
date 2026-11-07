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

#include "pytheia/io/io.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "theia/io/bundler_file_reader.h"
#include "theia/io/import_nvm_file.h"
#include "theia/io/io_wrapper.h"
#include "theia/io/populate_image_sizes.h"
#include "theia/io/read_1dsfm.h"
#include "theia/io/read_bundler_files.h"
#include "theia/io/read_strecha_dataset.h"
#include "theia/io/reconstruction_reader.h"
#include "theia/io/reconstruction_writer.h"
#include "theia/io/write_bundler_files.h"
#include "theia/io/write_colmap_files.h"
#include "theia/io/write_nerfstudio.h"
#include "theia/io/write_nvm_file.h"
#include "theia/io/write_ply_file.h"
#include "theia/io/write_sdfstudio.h"

namespace py = pybind11;

namespace pytheia {
namespace io {

void pytheia_io_classes(py::module& m) {
  py::class_<theia::BundlerObservation>(m, "BundlerObservation")
      .def(py::init<>())
      .def_readwrite("camera_index", &theia::BundlerObservation::camera_index)
      .def_readwrite("feature_index", &theia::BundlerObservation::feature_index)
      .def_readwrite("x", &theia::BundlerObservation::x)
      .def_readwrite("y", &theia::BundlerObservation::y);

  py::class_<theia::BundlerPoint>(m, "BundlerPoint")
      .def(py::init<>())
      .def_readwrite("position", &theia::BundlerPoint::position)
      .def_readwrite("color", &theia::BundlerPoint::color)
      .def_readwrite("view_list", &theia::BundlerPoint::view_list);

  py::class_<theia::BundlerFileReader>(m, "BundlerFileReader")
      .def(py::init<std::string, std::string>(), py::arg("lists_filepath"),
           py::arg("bundler_filepath"))
      .def("cameras", &theia::BundlerFileReader::cameras)
      .def("points", &theia::BundlerFileReader::points)
      .def("img_entries", &theia::BundlerFileReader::img_entries)
      .def("ParseBundleFile", &theia::BundlerFileReader::ParseBundleFile)
      .def("ParseListsFile", &theia::BundlerFileReader::ParseListsFile);

  py::class_<theia::BundlerCamera>(m, "BundlerCamera")
      .def(py::init())
      .def_readwrite("translation", &theia::BundlerCamera::translation)
      .def_readwrite("rotation", &theia::BundlerCamera::rotation)
      .def_readwrite("focal_length", &theia::BundlerCamera::focal_length)
      .def_readwrite("radial_coeff_1", &theia::BundlerCamera::radial_coeff_1)
      .def_readwrite("radial_coeff_2", &theia::BundlerCamera::radial_coeff_2);

  py::class_<theia::ListImgEntry>(m, "ListImgEntry")
      .def(py::init())
      .def_readwrite("filename", &theia::ListImgEntry::filename)
      .def_readwrite("second_entry", &theia::ListImgEntry::second_entry)
      .def_readwrite("focal_length", &theia::ListImgEntry::focal_length);

  m.def("ImportNVMFile", theia::ImportNVMFileWrapper,
        py::arg("nvm_filepath"));
  m.def("PopulateImageSizesAndPrincipalPoints",
        theia::PopulateImageSizesAndPrincipalPointsWrapper,
        py::arg("image_directory"));
  m.def("Read1DSFM", theia::Read1DSFMWrapper, py::arg("dataset_directory"));
  m.def("ReadBundlerFiles", theia::ReadBundlerFilesWrapper,
        py::arg("lists_file"), py::arg("bundle_file"));

  m.def("ReadStrechaDataset", theia::ReadStrechaDatasetWrapper,
        py::arg("dataset_directory"));
  m.def("ReadReconstruction", theia::ReadReconstructionWrapper,
        py::arg("input_file"));
  m.def(
      "WriteReconstruction",
      [](const theia::Reconstruction& reconstruction,
         const std::string& output_file, bool write_full_reconstruction) {
        return theia::WriteReconstruction(reconstruction, output_file,
                                        write_full_reconstruction);
      },
      py::arg("reconstruction"), py::arg("output_file"),
      py::arg("write_full_reconstruction") = false,
      "Write reconstruction to binary file. By default writes only estimated "
      "views and multi-view tracks (smaller file). Pass "
      "write_full_reconstruction=True to save the complete model.");
  m.def("WriteReconstructionJson", theia::WriteReconstructionJson,
        py::arg("reconstruction"), py::arg("output_json_file"));
  m.def("WriteBundlerFiles", theia::WriteBundlerFiles,
        py::arg("reconstruction"), py::arg("lists_file"),
        py::arg("bundle_file"));
  m.def("WriteColmapFiles", theia::WriteColmapFiles,
        py::arg("reconstruction"), py::arg("output_directory"));
  m.def("WriteNVMFile", theia::WriteNVMFile, py::arg("nvm_filepath"),
        py::arg("reconstruction"));
  m.def("WritePlyFile", theia::WritePlyFile, py::arg("ply_file"),
        py::arg("reconstruction"), py::arg("camera_color"),
        py::arg("min_num_observations_per_point"));
  m.def(
      "WriteRigPlyFile",
      [](const std::string& ply_file,
         const theia::Reconstruction& reconstruction,
         const Eigen::Vector3i& sensor_color,
         int min_num_observations_per_point,
         const Eigen::Vector3i& capture_color,
         const Eigen::Vector3i& trajectory_color,
         const Eigen::Vector3i& baseline_color,
         int edge_samples) {
        return theia::WriteRigPlyFile(ply_file,
                                      reconstruction,
                                      sensor_color,
                                      min_num_observations_per_point,
                                      capture_color,
                                      trajectory_color,
                                      baseline_color,
                                      edge_samples);
      },
      py::arg("ply_file"),
      py::arg("reconstruction"),
      py::arg("sensor_color"),
      py::arg("min_num_observations_per_point") = 2,
      py::arg("capture_color") = Eigen::Vector3i(255, 220, 0),
      py::arg("trajectory_color") = Eigen::Vector3i(0, 220, 255),
      py::arg("baseline_color") = Eigen::Vector3i(255, 0, 180),
      py::arg("edge_samples") = 24,
      "Write a PLY with tracks, capture centers, sensor centers, and sampled "
      "polylines for the rig trajectory and intra-capture baselines.");
  m.def("WriteNerfStudio", theia::WriteNerfStudio,
        py::arg("path_to_images"), py::arg("reconstruction"),
        py::arg("aabb_scale"), py::arg("out_json_nerfstudio_file"));
  m.def("WriteSdfStudio", theia::WriteSdfStudio,
        py::arg("path_to_images"), py::arg("reconstruction"),
        py::arg("nearfar"), py::arg("radius"));
}

void pytheia_io(py::module& m) {
  py::module m_submodule = m.def_submodule("io");
  pytheia_io_classes(m_submodule);
}

}  // namespace io
}  // namespace pytheia
