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
// Author: Aleksander Holynski (holynski@cs.washington.edu)

#ifndef THEIA_IO_WRITE_COLMAP_FILES_H_
#define THEIA_IO_WRITE_COLMAP_FILES_H_

#include <string>

namespace theia {

class Reconstruction;

// Writes all information from a Reconstruction into the COLMAP format.
//
// Input params are as follows:
//   reconstruction: A Theia Reconstruction containing the camera, track, and
//       point cloud information. See theia/sfm/reconstruction.h for more
//       information.
//   output_directory: The directory to which the COLMAP files will be written.
//       With binary = false this is cameras.txt, images.txt, points3D.txt.
//       With binary = true this is cameras.bin, images.bin, points3D.bin in
//       the COLMAP binary model format (little-endian).
//   binary: Write the binary format instead of the text format. Defaults to
//       false, which keeps the original behavior.
//
// Only estimated views and tracks are written. Cameras are written with the
// PINHOLE family of COLMAP models: SIMPLE_PINHOLE or PINHOLE without
// distortion, OPENCV (fx, fy, cx, cy, k1, k2, 0, 0) with distortion (binary
// only). Non-pinhole cameras (fisheye, FOV, ...) are rejected.
bool WriteColmapFiles(const Reconstruction& reconstruction,
                      const std::string& output_directory,
                      bool binary = false);

}  // namespace theia

#endif  // THEIA_IO_WRITE_COLMAP_FILES_H_
