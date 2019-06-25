// Copyright (C) 2019 The Regents of the University of California (Regents).
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

// File created by Steffen Urban, June 2019


#ifndef THEIA_IMAGE_DESCRIPTOR_SOSNET_DESCRIPTOR_H_
#define THEIA_IMAGE_DESCRIPTOR_SOSNET_DESCRIPTOR_H_

#include <vector>

#include "theia/image/descriptor/descriptor_extractor.h"
#include "theia/util/util.h"
#include <torch/torch.h>

const int SOSNET_PATCH_SIZE = 32;
constexpr float SOSNET_PATCH_SIZE_2 = SOSNET_PATCH_SIZE / 2.0f;
const float SOSNET_RHO = M_PI / 180.f;

namespace theia {

class FloatImage;
class Keypoint;

// Parameters for the akaze feature extractor.
struct SOSNetParameters {
  std::string model_path;
  bool use_gpu;
  // AKAZE detector params
  int maximum_octave_levels = 4;
  int num_sublevels = 4;
  // Lowering this threshold will increase the number of features.
  float hessian_threshold = 0.001f;
};

class SOSNetDescriptorExtractor : public DescriptorExtractor {
 public:
  explicit SOSNetDescriptorExtractor(const SOSNetParameters& detector_params);
  ~SOSNetDescriptorExtractor() {}

  // NOTE: This method will gracefully fail with a fatal logging method. AKAZE
  // must use its own keypoints so only DetectAndExtract can be used.
  bool ComputeDescriptor(const FloatImage& image,
                         const Keypoint& keypoint,
                         Eigen::VectorXf* descriptor);

  // Detect keypoints using the Akaze keypoint detector and extracts them at the
  // same time.
  bool DetectAndExtractDescriptors(const FloatImage& image,
                                   std::vector<Keypoint>* keypoints,
                                   std::vector<Eigen::VectorXf>* descriptors);

 private:
  const SOSNetParameters sosnet_params_;
  std::shared_ptr<torch::jit::script::Module> sosnet;

  DISALLOW_COPY_AND_ASSIGN(SOSNetDescriptorExtractor);
};

}  // namespace theia

#endif  // THEIA_IMAGE_DESCRIPTOR_SOSNET_DESCRIPTOR_H_
