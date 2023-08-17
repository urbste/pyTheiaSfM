// Copyright (C) 2023 Steffen Urban
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
// Author: Steffen Urban (urbste@gmail.com)

#include <Eigen/Core>

using namespace std;

namespace theia {

// see Förstner et al, PCV, page 521
// Js(x)
template <typename T>
void jacobian_3_vec(const Eigen::Matrix<T, 3, 1>& vec,
                    Eigen::Matrix<T, 3, 3>& jac) {
  jac = (Eigen::Matrix<T, 3, 3>::Identity() -
         (vec * vec.transpose()) / (vec.transpose() * vec)) /
        vec.norm();
}

// have a look at
// W. Foerstner, PCV, Page 370
// and S.Urban, MLPnP paper
template <typename T>
void get_information_for_bearing(const T& variance,
                                 const Eigen::Matrix<T, 3, 3>& inv_cam_mat,
                                 const Eigen::Matrix<T, 3, 3>& bearing_jac,
                                 const Eigen::Matrix<T, 3, 2>& bearing_ns,
                                 Eigen::Matrix<T, 2, 2>& information) {
  const Eigen::Matrix<T, 3, 3> Exx =
      Eigen::Matrix<T, 3, 1>(variance, variance, 0).asDiagonal();
  const Eigen::Matrix<T, 3, 3> proj_Exx =
      inv_cam_mat * Exx * inv_cam_mat.transpose();
  const Eigen::Matrix<T, 3, 3> Evv =
      bearing_jac * proj_Exx * bearing_jac.transpose();
  const Eigen::Matrix<T, 2, 2> Ers = bearing_ns.transpose() * Evv * bearing_ns;
  information = Ers.inverse();
}

/**
 * compute the nullspace of a 3-vector efficiently
 * without QR see W.Foerstner PCV, Page 778, eq. A.120)
 *
 * @param vector  Eigen::Matrix<T, 3, 1>
 *
 * @return      nullspace 3x2
 */
template <typename T>
void nullS_3x2_templated(const Eigen::Matrix<T, 3, 1>& vector,
                         Eigen::Matrix<T, 3, 2>& nullspace) {
  const T x_n = vector(2);
  const Eigen::Matrix<T, 2, 1> x_0(vector(0), vector(1));
  const Eigen::Matrix<T, 2, 2> I_2 = Eigen::Matrix<T, 2, 2>::Identity();

  const Eigen::Matrix<T, 2, 2> outer_prod = x_0 * x_0.transpose();
  if (x_n > T(0)) {
    const Eigen::Matrix<T, 2, 2> tmp = I_2 - outer_prod / (T(1) + x_n);
    nullspace.row(0) = tmp.row(0);
    nullspace.row(1) = tmp.row(1);
    nullspace.row(2) = -x_0.transpose();
  } else {
    const Eigen::Matrix<T, 2, 2> tmp = I_2 - outer_prod / (T(1) - x_n);
    nullspace.row(0) = tmp.row(0);
    nullspace.row(1) = tmp.row(1);
    nullspace.row(2) = x_0.transpose();
  }
}

/**
 * compute the nullspace of a 4-vector efficiently
 * without QR, see W.Foerstner PCV, Page 778, eq. A.120)
 *
 * @param vector  Eigen::Matrix<T, 4, 1>
 *
 * @return      nullspace 4x3
 */
template <typename T>
void nullS_3x4_templated(const Eigen::Matrix<T, 4, 1>& vector,
                         Eigen::Matrix<T, 4, 3>& nullspace) {
  const T x_n = vector(3);
  const Eigen::Matrix<T, 3, 1> x_0(vector(0), vector(1), vector(2));
  const Eigen::Matrix<T, 3, 3> I_3 = Eigen::Matrix<T, 3, 3>::Identity();

    const Eigen::Matrix<T, 3, 3> outer_prod = (x_0  * x_0.transpose();
    if (x_n > T(0))
    {
    const Eigen::Matrix<T, 3, 3> tmp = I_3 - outer_prod / (T(1) + x_n);
    nullspace.row(0) = tmp.row(0);
    nullspace.row(1) = tmp.row(1);
    nullspace.row(2) = tmp.row(2);
    nullspace.row(3) = -x_0.transpose();
    }
    else
    {
    const Eigen::Matrix<T, 3, 3> tmp = I_3 - outer_prod / (T(1) - x_n);
    nullspace.row(0) = tmp.row(0);
    nullspace.row(1) = tmp.row(1);
    nullspace.row(2) = tmp.row(2);
    nullspace.row(3) = x_0.transpose();
    }
}

}  // namespace theia
