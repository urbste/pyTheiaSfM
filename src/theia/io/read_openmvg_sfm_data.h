// Copyright (C) 2026, Steffen Urban
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

#ifndef THEIA_IO_READ_OPENMVG_SFM_DATA_H_
#define THEIA_IO_READ_OPENMVG_SFM_DATA_H_

#include <string>

#include "theia/io/imported_reconstruction.h"

namespace theia {

class Reconstruction;

// Reads an OpenMVG sfm_data.json (views, intrinsics, extrinsics and structure)
// into a Reconstruction. Only the JSON format is read, not the cereal binary
// sfm_data.bin.
//
// Input params are as follows:
//   json_path: Path to sfm_data.json.
//   reconstruction: The output. It should be empty.
//   messages: Optional. Receives the error on failure and the warnings.
//
// The cereal pointer encoding is resolved: a ptr_wrapper is defined once with
// its data and may be referenced later by id. View names are the image paths
// relative to root_path (local_path/filename). Views are added in view-key
// order and become estimated when they have an extrinsic (pose) entry, which
// holds the world-to-camera rotation and the camera center. The OpenMVG
// intrinsic key becomes the camera intrinsics group id. Structure entries
// become tracks with their observations, read in pixel coordinates without an
// offset.
//
// Pinhole intrinsics are read with the distortion keys disto_k1, disto_k3
// ([k1, k2, k3]) and disto_t2 ([k1, k2, k3, d1, d2, t1, t2]; d1, d2 are
// dropped with a warning). They map to PINHOLE, or to
// PINHOLE_RADIAL_TANGENTIAL when k3 or a tangential term is nonzero.
//
// Returns false if the file is missing (messages->file_not_found is then set)
// or malformed, or if it uses an unsupported camera model (fisheye,
// spherical, unknown distortion keys).
bool ReadOpenMVGSfMData(const std::string& json_path,
                        Reconstruction* reconstruction,
                        ImportMessages* messages = nullptr);

}  // namespace theia

#endif  // THEIA_IO_READ_OPENMVG_SFM_DATA_H_
