// Copyright (C) 2014 The Regents of the University of California (Regents).
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
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#ifndef THEIA_SFM_EXIF_READER_H_
#define THEIA_SFM_EXIF_READER_H_

#include <string>

#include "theia/util/util.h"

namespace theia {

struct CameraIntrinsicsPrior;

// EXIF metadata extraction is not implemented in pyTheia. Use Python (e.g.
// Pillow, exifread, or OpenCV) to populate CameraIntrinsicsPrior and pass it
// into reconstruction APIs. This class remains for API compatibility.
class ExifReader {
 public:
  ExifReader() = default;

  // If the file cannot be opened, returns false. Otherwise resets `prior` to a
  // default state (no fields set) and returns true. EXIF is not read in C++;
  // use Python to populate intrinsics priors when needed.
  bool ExtractEXIFMetadata(const std::string& image_file,
                           CameraIntrinsicsPrior* camera_intrinsics_prior) const;

 private:
  DISALLOW_COPY_AND_ASSIGN(ExifReader);
};

}  // namespace theia

#endif  // THEIA_SFM_EXIF_READER_H_
