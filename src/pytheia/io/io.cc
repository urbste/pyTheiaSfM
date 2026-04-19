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
      .def(py::init<std::string, std::string>())
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

  m.def("ImportNVMFile", theia::ImportNVMFileWrapper);
  m.def("PopulateImageSizesAndPrincipalPoints",
        theia::PopulateImageSizesAndPrincipalPointsWrapper);
  m.def("Read1DSFM", theia::Read1DSFMWrapper);
  m.def("ReadBundlerFiles", theia::ReadBundlerFilesWrapper);

  m.def("ReadStrechaDataset", theia::ReadStrechaDatasetWrapper);
  m.def("ReadReconstruction", theia::ReadReconstructionWrapper);
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
  m.def("WriteReconstructionJson", theia::WriteReconstructionJson);
  m.def("WriteBundlerFiles", theia::WriteBundlerFiles);
  m.def("WriteColmapFiles", theia::WriteColmapFiles);
  m.def("WriteNVMFile", theia::WriteNVMFile);
  m.def("WritePlyFile", theia::WritePlyFile);
  m.def("WriteNerfStudio", theia::WriteNerfStudio);
  m.def("WriteSdfStudio", theia::WriteSdfStudio);
}

void pytheia_io(py::module& m) {
  py::module m_submodule = m.def_submodule("io");
  pytheia_io_classes(m_submodule);
}

}  // namespace io
}  // namespace pytheia
