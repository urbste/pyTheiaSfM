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

#ifndef THEIA_IO_READ_COLMAP_FILES_H_
#define THEIA_IO_READ_COLMAP_FILES_H_

#include <string>

#include "theia/io/imported_reconstruction.h"

namespace theia {

class Reconstruction;

// Reads a COLMAP sparse model (cameras, images, points3D as .txt or .bin) into
// a Reconstruction. GLOMAP writes the same format. This is the counterpart of
// WriteColmapFiles.
//
// Input params are as follows:
//   path: The sparse model folder, or a folder above it. The folder is found
//       by checking path, path/sparse/0, path/sparse, path/0 and then the
//       subfolders of path (depth first in sorted order, up to four levels).
//       A folder qualifies when it holds cameras, images and points3D in one
//       format. Binary files are preferred when both formats are present.
//   reconstruction: The output. It should be empty.
//   pixel_offset: Subtracted from every observation and from the principal
//       point. The default 0 keeps COLMAP's pixel coordinates (centre of the
//       top-left pixel at 0.5). Pass 0.5 for the OpenCV convention.
//   messages: Optional. Receives the error on failure and the warnings.
//
// Every image becomes an estimated view, in image-id order, and the COLMAP
// camera id becomes the camera intrinsics group id. Every 3D point becomes an
// estimated track with its color, and every 2D point that refers to a 3D point
// becomes an observation. COLMAP stores a world-to-camera rotation as a
// quaternion (w, x, y, z) and tvec = -R C, so the camera center is -R^T tvec.
//
// Camera models are mapped to the pyTheia model of the same name where one
// exists, with all of their parameters: SIMPLE_PINHOLE, PINHOLE,
// SIMPLE_RADIAL and RADIAL to PINHOLE (k1, k2); OPENCV to
// PINHOLE_RADIAL_TANGENTIAL; OPENCV_FISHEYE to FISHEYE; SIMPLE_RADIAL_FISHEYE
// to RADIAL_FISHEYE with k2 = 0; FOV, FULL_OPENCV, RADIAL_FISHEYE,
// THIN_PRISM_FISHEYE and RAD_TAN_THIN_PRISM_FISHEYE to the models of the same
// name. Other COLMAP models are rejected.
//
// Returns false if no model is found (messages->file_not_found is then set),
// if the files are malformed, or if a camera model is not supported.
bool ReadColmapFiles(const std::string& path,
                     Reconstruction* reconstruction,
                     double pixel_offset = 0.0,
                     ImportMessages* messages = nullptr);

}  // namespace theia

#endif  // THEIA_IO_READ_COLMAP_FILES_H_
