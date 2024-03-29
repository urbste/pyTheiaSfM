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
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "theia/sfm/bundle_adjustment/create_loss_function.h"
#include "theia/sfm/bundle_adjustment/loss_functions.h"
#include <ceres/ceres.h>
#include <memory>

namespace theia {

std::unique_ptr<ceres::LossFunction> CreateLossFunction(
    const LossFunctionType& loss_function_type,
    const double robust_loss_width) {
  std::unique_ptr<ceres::LossFunction> loss_function;
  switch (loss_function_type) {
    case LossFunctionType::TRIVIAL:
      loss_function.reset(new ceres::TrivialLoss());
      break;
    case LossFunctionType::HUBER:
      loss_function.reset(new ceres::HuberLoss(robust_loss_width));
      break;
    case LossFunctionType::SOFTLONE:
      loss_function.reset(new ceres::SoftLOneLoss(robust_loss_width));
      break;
    case LossFunctionType::CAUCHY:
      loss_function.reset(new ceres::CauchyLoss(robust_loss_width));
      break;
    case LossFunctionType::ARCTAN:
      loss_function.reset(new ceres::ArctanLoss(robust_loss_width));
      break;
    case LossFunctionType::TUKEY:
      loss_function.reset(new ceres::TukeyLoss(robust_loss_width));
      break;
    case LossFunctionType::TRUNCATED:
      loss_function.reset(new TruncatedLoss(robust_loss_width));
      break;
    default:
      LOG(FATAL) << "Invalid Loss Function chosen.";
      break;
  }

  return loss_function;
}

}  // namespace theia
