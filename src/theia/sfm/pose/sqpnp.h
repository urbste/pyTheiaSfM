//
// sqpnp.h
//
// George Terzakis (terzakig-at-hotmail-dot-com), September 2020
// Nearest orthogonal approximation code (C) 2019 Manolis Lourakis
//
// Implementation of SQPnP as described in the paper:
//
// "A Consistently Fast and Globally Optimal Solution to the Perspective-n-Point Problem" by G. Terzakis and M. Lourakis
//  	 a) Paper: 	   http://www.ecva.net/papers/eccv_2020/papers_ECCV/papers/123460460.pdf 
//       b) Supplementary: https://www.ecva.net/papers/eccv_2020/papers_ECCV/papers/123460460.pdf
  
// Copyright (C) 2022 Steffen Urban.
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
// Author: Steffen Urban (urbste@googlemail.com)

#ifndef THEIA_SFM_POSE_SQPNP_H_
#define THEIA_SFM_POSE_SQPNP_H_

#include "types.h"
#include <vector>
#include <assert.h>
#include <iostream>

namespace theia
{
// Computes the camera pose using the Perspective N-point method from "A Consistently Fast and Globally Optimal Solution 
// to the Perspective-n-Point Problem" by G. Terzakis and M. Lourakis
//
// Params:
//   feature_position: Feature positions corresponding to model points. Must
//     contain at least 3 points.
//   points_3d: 3D location of features. Must correspond to the image_ray
//     of the same index. Must contain the same number of points as image_ray,
//     and at least 3.
//   solution_rotation: the rotation quaternion of the candidate solutions
//   solution_translation: the translation of the candidate solutions
  bool SQPnP(const std::vector<Eigen::Vector2d>& feature_positions,
             const std::vector<Eigen::Vector3d>& world_point,
             std::vector<Eigen::Quaterniond>* solution_rotation,
             std::vector<Eigen::Vector3d>* solution_translation);

} 

#endif // THEIA_SFM_POSE_SQPNP_H_