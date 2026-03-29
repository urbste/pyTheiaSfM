// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.

#include "theia/sfm/exif_reader.h"

#include <fstream>

#include <glog/logging.h>

#include "theia/sfm/camera_intrinsics_prior.h"

namespace theia {

bool ExifReader::ExtractEXIFMetadata(
    const std::string& image_file,
    CameraIntrinsicsPrior* camera_intrinsics_prior) const {
  CHECK_NOTNULL(camera_intrinsics_prior);
  std::ifstream f(image_file.c_str());
  if (!f.good()) {
    return false;
  }
  *camera_intrinsics_prior = CameraIntrinsicsPrior{};
  return true;
}

}  // namespace theia
