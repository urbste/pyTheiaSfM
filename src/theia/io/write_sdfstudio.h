
// Copyright (C) 2023, Steffen Urban
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

#ifndef THEIA_IO_WRITE_SDFSTUDIO_H_
#define THEIA_IO_WRITE_SDFSTUDIO_H_

#include <string>

namespace theia {

class Reconstruction;

// Converts a theia reconstruction to a sdfstudio json file
// It is assumed, that the Views in the reconstruction have the same
// name than the image filename, e.g. image001.png with extension
// The filenames in the nerfstudio json are then concatenated as:
// path_to_images + / + view_image_name
// Currently only 1 type of camera models is supported: PINHOLE
// All images need to be undistorted! No distortion in the settings.
bool WriteSdfStudio(const std::string& path_to_images,
                  const Reconstruction& reconstruction,
                  const std::pair<double, double>& nearfar,
                  const double radius);

}  // namespace theia

#endif  // THEIA_IO_WRITE_SDFSTUDIO_H_
