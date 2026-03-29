// Copyright (C) 2018 The Regents of the University of California (Regents).
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

#include "theia/matching/fisher_vector_extractor.h"

#include <glog/logging.h>

namespace theia {

FisherVectorExtractor::FisherVectorExtractor(const Options& options)
    : options_(options) {
  (void)options_;
}

FisherVectorExtractor::~FisherVectorExtractor() = default;

void FisherVectorExtractor::AddFeaturesForTraining(
    const std::vector<Eigen::VectorXf>& features) {
  (void)features;
}

bool FisherVectorExtractor::Train() {
  LOG(INFO) << "Global image descriptors: mean pooling of local features "
               "(VLFeat Fisher / GMM removed).";
  return true;
}

Eigen::VectorXf FisherVectorExtractor::ExtractGlobalDescriptor(
    const std::vector<Eigen::VectorXf>& features) {
  CHECK_GT(features.size(), 0);
  const int dim = features[0].size();
  CHECK_GT(dim, 0);
  Eigen::VectorXf pooled = Eigen::VectorXf::Zero(dim);
  for (const Eigen::VectorXf& f : features) {
    CHECK_EQ(f.size(), dim);
    CHECK(!f.hasNaN());
    pooled += f;
  }
  pooled /= static_cast<float>(features.size());
  return pooled;
}

}  // namespace theia
