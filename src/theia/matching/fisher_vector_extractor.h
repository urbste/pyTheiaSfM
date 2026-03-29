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
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (sweeney.chris.m@gmail.com)

#ifndef THEIA_MATCHING_FISHER_VECTOR_EXTRACTOR_H_
#define THEIA_MATCHING_FISHER_VECTOR_EXTRACTOR_H_

#include <Eigen/Core>
#include <vector>

#include "theia/matching/global_descriptor_extractor.h"

namespace theia {

// Per-image global descriptor used for kNN image-pair preselection in matching.
// Historically this used a VLFeat Fisher vector over a GMM; without VLFeat this
// is implemented as the element-wise mean of local descriptors (same length as
// one local descriptor).
class FisherVectorExtractor : public GlobalDescriptorExtractor {
 public:
  struct Options {
    // Legacy names kept for existing call sites; both are ignored.
    int num_gmm_clusters = 16;
    int max_num_features_for_training = 100000;
  };

  explicit FisherVectorExtractor(const Options& options);
  ~FisherVectorExtractor() override;

  void AddFeaturesForTraining(
      const std::vector<Eigen::VectorXf>& features) override;

  bool Train() override;

  Eigen::VectorXf ExtractGlobalDescriptor(
      const std::vector<Eigen::VectorXf>& features) override;

 private:
  Options options_;
};

}  // namespace theia
#endif  // THEIA_MATCHING_FISHER_VECTOR_EXTRACTOR_H_
