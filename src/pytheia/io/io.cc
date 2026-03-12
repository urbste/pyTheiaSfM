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

#include <iostream>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
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
  py::class_<theia::BundlerFileReader>(m, "BundlerFileReader",
                                       "Reader for Bundler format (lists + bundle file).")
      .def(py::init<std::string, std::string>(),
           py::arg("lists_file"), py::arg("bundle_file"))
      .def("cameras", &theia::BundlerFileReader::cameras,
           "Return list of Bundler cameras.")
      .def("points", &theia::BundlerFileReader::points,
           "Return 3D points.")
      .def("img_entries", &theia::BundlerFileReader::img_entries,
           "Return image list entries.")
      .def("ParseBundleFile", &theia::BundlerFileReader::ParseBundleFile,
           "Parse the bundle file. Returns True on success.")
      .def("ParseListsFile", &theia::BundlerFileReader::ParseListsFile,
           "Parse the image lists file. Returns True on success.");

  py::class_<theia::BundlerCamera>(m, "BundlerCamera",
                                   "Bundler camera: translation, rotation, focal length, radial coeffs.")
      .def(py::init())
      .def_readwrite("translation", &theia::BundlerCamera::translation)
      .def_readwrite("rotation", &theia::BundlerCamera::rotation)
      .def_readwrite("focal_length", &theia::BundlerCamera::focal_length)
      .def_readwrite("radial_coeff_1", &theia::BundlerCamera::radial_coeff_1)
      .def_readwrite("radial_coeff_2", &theia::BundlerCamera::radial_coeff_2);

  py::class_<theia::FeatureInfo>(m, "FeatureInfo",
                                "Feature index and 2D position for one camera.")
      .def(py::init())
      .def_readwrite("camera_index", &theia::FeatureInfo::camera_index)
      .def_readwrite("feature_index", &theia::FeatureInfo::feature_index)
      .def_readwrite("kpt_x", &theia::FeatureInfo::kpt_x)
      .def_readwrite("kpt_y", &theia::FeatureInfo::kpt_y);

  py::class_<theia::ListImgEntry>(m, "ListImgEntry",
                                 "Single entry in a Bundler image list (filename, focal length).")
      .def(py::init())
      .def_readwrite("filename", &theia::ListImgEntry::filename)
      .def_readwrite("second_entry", &theia::ListImgEntry::second_entry)
      .def_readwrite("focal_length", &theia::ListImgEntry::focal_length);

  m.def("ImportNVMFile", theia::ImportNVMFileWrapper,
        py::arg("nvm_filepath"),
        "Load reconstruction and view graph from an NVM file. Returns (success, reconstruction).");
  m.def("PopulateImageSizesAndPrincipalPoints",
        theia::PopulateImageSizesAndPrincipalPointsWrapper,
        py::arg("image_directory"),
        "Populate image sizes and principal points from images in directory. Returns (success, reconstruction).");
  m.def("Read1DSFM", theia::Read1DSFMWrapper,
        py::arg("dataset_directory"),
        "Read 1DSFM dataset. Returns (success, reconstruction, view_graph).");
  m.def("ReadBundlerFiles", theia::ReadBundlerFilesWrapper,
        py::arg("lists_file"), py::arg("bundle_file"),
        "Read Bundler reconstruction from lists and bundle file. Returns (success, reconstruction).");
  m.def("ReadStrechaDataset", theia::ReadStrechaDatasetWrapper,
        py::arg("dataset_directory"),
        "Read Strecha dataset. Returns (success, reconstruction).");
  m.def("ReadReconstruction", theia::ReadReconstructionWrapper,
        py::arg("input_file"),
        "Read reconstruction from binary file. Returns (success, reconstruction).");
  m.def("WriteReconstruction", theia::WriteReconstruction,
        py::arg("reconstruction"), py::arg("output_file"),
        "Write reconstruction to binary file. Returns True on success.");
  m.def("WriteReconstructionJson", theia::WriteReconstructionJson,
        py::arg("reconstruction"), py::arg("output_json_file"),
        "Write reconstruction to JSON file. Returns True on success.");
  m.def("WriteBundlerFiles", theia::WriteBundlerFiles,
        py::arg("reconstruction"), py::arg("lists_file"), py::arg("bundle_file"),
        "Write reconstruction in Bundler format. Returns True on success.");
  m.def("WriteColmapFiles", theia::WriteColmapFiles,
        py::arg("reconstruction"), py::arg("output_directory"),
        "Write reconstruction in COLMAP format (cameras, images, points). Returns True on success.");
  m.def("WriteNVMFile", theia::WriteNVMFile,
        py::arg("nvm_filepath"), py::arg("reconstruction"),
        "Write reconstruction to NVM file. Returns True on success.");
  m.def("WritePlyFile", theia::WritePlyFile,
        py::arg("ply_file"), py::arg("reconstruction"),
        py::arg("camera_color"), py::arg("min_num_observations_per_point"),
        "Write reconstruction to PLY file (points and cameras). Returns True on success.");
  m.def("WriteNerfStudio", theia::WriteNerfStudio,
        py::arg("path_to_images"), py::arg("reconstruction"),
        py::arg("aabb_scale"), py::arg("out_json_nerfstudio_file"),
        "Write reconstruction for NeRF Studio. Returns True on success.");
  m.def("WriteSdfStudio", theia::WriteSdfStudio,
        py::arg("path_to_images"), py::arg("reconstruction"),
        py::arg("nearfar"), py::arg("radius"),
        "Write reconstruction for SDF Studio. Returns True on success.");
}

void pytheia_io(py::module& m) {
  py::module m_submodule =
      m.def_submodule("io", "I/O for reconstructions: Bundler, NVM, COLMAP, PLY, NeRF Studio, SDF Studio.");
  pytheia_io_classes(m_submodule);
}

}  // namespace io
}  // namespace pytheia