// Copyright (C) 2025, Steffen Urban
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

#ifndef THEIA_IO_WRITE_TRANSFORMS_JSON_H_
#define THEIA_IO_WRITE_TRANSFORMS_JSON_H_

#include <string>

namespace theia {

class Reconstruction;

// Writes a transforms.json file in the convention used by instant-ngp, gsplat,
// and Nerfstudio (the "NeRF synthetic" style file).
//
// Convention:
//   * transform_matrix is a 4x4 camera-to-world matrix [R_c2w | C; 0 0 0 1] in
//     the world frame of the reconstruction, where C is the camera center
//     (Camera::GetPosition()) and R_c2w = R^T with R the world-to-camera
//     rotation (Camera::GetOrientationAsRotationMatrix()), i.e.
//     x_cam = R (X - C) for a world point X.
//   * Camera axes use the OpenGL / NeRF convention: x points right, y points
//     up and z points backwards, so the camera looks along -z. This is the
//     OpenCV camera-to-world matrix with its 2nd and 3rd columns negated
//     (y and z of the OpenCV camera axes are flipped).
//   * No additional world-axis permutation is applied. This differs from
//     WriteNerfStudio, which emits Nerfstudio's COLMAP-converted world frame.
//   * Intrinsics are in pixels: fl_x, fl_y (focal lengths), cx, cy (principal
//     point), w, h (image size). Radial distortion is written as k1, k2 and
//     tangential distortion as p1, p2 (0). camera_model is "PINHOLE" when the
//     camera has no distortion and "OPENCV" otherwise.
//   * Each frame has file_path = path_to_images + "/" + view name. Relative
//     paths are resolved relative to the location of the json file.
//   * When all views share one camera intrinsics group the intrinsics are
//     also written at the top level.
//
// Only estimated views are written. Only PINHOLE cameras are supported; other
// camera models produce an error and return false.
//
// Returns false if the reconstruction has no estimated views, the camera model
// is unsupported, or the file cannot be written.
bool WriteTransformsJson(const std::string& path_to_images,
                         const Reconstruction& reconstruction,
                         const std::string& out_json_file);

}  // namespace theia

#endif  // THEIA_IO_WRITE_TRANSFORMS_JSON_H_
