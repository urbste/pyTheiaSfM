// Copyright (C) 2021 Steffen Urban
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
//     * Neither the name of The Regents or University of California, Google,
//       nor the names of its contributors may be used to endorse or promote
//       products derived from this software without specific prior written
//       permission.
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

// Code taken from Cenek Albl: https://github.com/CenekAlbl/RnP

#include "seven_point_rs_focal_length_radial_distortion.h"
#include "rolling_shutter_projection_utils.h"

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <chrono>
#include <iostream>

namespace theia {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;

static void compute_coeffs(const double data[35], double coeffs[34]) {
  coeffs[0] = -data[15];
  coeffs[1] = -data[16];
  coeffs[2] = -data[14];
  coeffs[3] = data[1] - data[17];
  coeffs[4] = -data[18];
  coeffs[5] = data[2];
  coeffs[6] = -data[19];
  coeffs[7] = data[0];
  coeffs[8] = data[3];
  coeffs[9] = data[4] - data[20];
  coeffs[10] = data[5];
  coeffs[11] = data[6];
  coeffs[12] = data[8];
  coeffs[13] = data[9] - data[17];
  coeffs[14] = data[7];
  coeffs[15] = data[10];
  coeffs[16] = data[11];
  coeffs[17] = data[12] - data[20];
  coeffs[18] = data[13];
  coeffs[19] = 1.0;
  coeffs[20] = data[22];
  coeffs[21] = data[23];
  coeffs[22] = data[21];
  coeffs[23] = data[24];
  coeffs[24] = data[25];
  coeffs[25] = data[26];
  coeffs[26] = data[27];
  coeffs[27] = data[29];
  coeffs[28] = data[30];
  coeffs[29] = data[28];
  coeffs[30] = data[31];
  coeffs[31] = data[32];
  coeffs[32] = data[33];
  coeffs[33] = data[34];
}

static void setup_elimination_template(Eigen::Matrix<double, 35, 1> const input,
                                       Eigen::Matrix<double, 26, 26> &C0,
                                       Eigen::Matrix<double, 26, 10> &C1) {
  double coeffs[34];
  int i0;
  static const short iv0[151] = {
      0,   5,   27,  35,  54,  57,  60,  78,  79,  80,  81,  83,  85,  87,
      102, 105, 108, 113, 116, 130, 133, 153, 157, 162, 163, 179, 184, 187,
      195, 208, 211, 233, 234, 235, 236, 237, 240, 241, 247, 256, 257, 259,
      260, 262, 263, 264, 265, 270, 272, 280, 286, 289, 297, 313, 318, 319,
      322, 323, 324, 326, 335, 340, 341, 351, 363, 364, 366, 367, 374, 375,
      376, 377, 378, 380, 389, 394, 400, 401, 402, 404, 405, 408, 410, 426,
      427, 428, 430, 452, 453, 454, 456, 458, 459, 460, 482, 484, 485, 486,
      496, 497, 499, 502, 503, 507, 513, 515, 518, 519, 522, 525, 535, 554,
      555, 565, 570, 573, 578, 579, 580, 581, 591, 595, 596, 602, 606, 607,
      613, 616, 617, 618, 622, 625, 628, 630, 631, 632, 633, 634, 635, 636,
      638, 639, 642, 643, 644, 647, 648, 654, 665, 668, 670};

  static const signed char iv1[151] = {
      0,  0,  0,  0,  0,  12, 19, 3,  2,  1,  0,  13, 19, 2,  19, 4,  0,
      4,  0,  5,  1,  19, 5,  1,  21, 28, 2,  14, 19, 7,  2,  19, 8,  7,
      5,  13, 2,  22, 21, 19, 29, 28, 9,  6,  4,  2,  17, 19, 2,  19, 10,
      6,  19, 10, 6,  25, 21, 28, 5,  1,  32, 7,  14, 22, 29, 11, 10, 17,
      22, 29, 7,  25, 2,  19, 32, 17, 24, 31, 9,  4,  25, 6,  32, 25, 32,
      10, 6,  26, 33, 11, 17, 32, 25, 10, 18, 33, 26, 11, 3,  12, 15, 22,
      14, 20, 2,  19, 29, 27, 4,  16, 19, 20, 12, 0,  27, 3,  0,  20, 21,
      13, 1,  27, 28, 12, 24, 16, 20, 0,  4,  27, 31, 9,  13, 4,  24, 25,
      17, 20, 27, 3,  0,  21, 1,  6,  28, 31, 32, 16, 24, 4,  31};

  static const unsigned char uv0[87] = {
      2U,   3U,   6U,   13U,  19U,  21U,  22U,  25U,  28U,  29U,  30U,
      31U,  39U,  41U,  43U,  44U,  46U,  51U,  54U,  55U,  65U,  66U,
      68U,  69U,  70U,  77U,  84U,  86U,  87U,  97U,  99U,  100U, 102U,
      105U, 110U, 111U, 123U, 125U, 126U, 127U, 136U, 149U, 151U, 152U,
      160U, 162U, 164U, 165U, 170U, 171U, 172U, 173U, 174U, 175U, 176U,
      177U, 178U, 180U, 183U, 188U, 189U, 192U, 193U, 194U, 196U, 198U,
      199U, 200U, 201U, 203U, 204U, 205U, 214U, 222U, 224U, 225U, 226U,
      227U, 229U, 230U, 238U, 248U, 249U, 250U, 251U, 252U, 254U};

  static const signed char iv2[87] = {
      8,  15, 14, 23, 7,  22, 29, 30, 9,  16, 14, 18, 24, 22, 19, 2,  29, 31,
      11, 18, 26, 14, 29, 22, 7,  33, 12, 23, 15, 3,  20, 27, 30, 8,  13, 23,
      5,  21, 28, 30, 15, 8,  23, 30, 15, 16, 26, 18, 12, 23, 27, 20, 3,  9,
      30, 24, 31, 33, 11, 17, 26, 23, 30, 8,  13, 28, 21, 5,  10, 25, 32, 33,
      18, 15, 30, 23, 8,  11, 26, 33, 18, 16, 26, 31, 24, 9,  33};

  compute_coeffs(input.data(), coeffs);
  memset(&C0.data()[0], 0, 676U * sizeof(double));
  memset(&C1.data()[0], 0, 260U * sizeof(double));
  for (i0 = 0; i0 < 151; i0++) {
    C0.data()[iv0[i0]] = coeffs[iv1[i0]];
  }

  for (i0 = 0; i0 < 87; i0++) {
    C1.data()[uv0[i0]] = coeffs[iv2[i0]];
  }
}

int solver_R7Pfr(Eigen::Matrix<double, 35, 1> const data,
                 Eigen::Matrix<std::complex<double>, 4, 10> &sols) {

  Eigen::Matrix<double, 26, 26> C0;
  Eigen::Matrix<double, 26, 10> C1;

  setup_elimination_template(data, C0, C1);

  C1 = C0.lu().solve(C1);

  Eigen::Matrix<double, 17, 10> RR;

  RR.topRows(7) = -C1.bottomRows(7);
  RR.bottomRows(10) = Eigen::MatrixXd::Identity(10, 10);

  int ind[10] = {0, 1, 8, 2, 3, 10, 4, 5, 13, 6};

  Eigen::Matrix<double, 10, 10> AM;

  for (int i = 0; i < 10; i++) {
    AM.row(i) = RR.row(ind[i]);
  }

  Eigen::EigenSolver<Eigen::MatrixXd> eig(AM);
  Eigen::Matrix<std::complex<double>, 10, 1> D = eig.eigenvalues();
  Eigen::Matrix<std::complex<double>, 10, 10> V = eig.eigenvectors();

  Eigen::Matrix<std::complex<double>, 10, 1> scale =
      D.array() / V.row(9).array().transpose();

  V = V.array() *
      (Eigen::Matrix<double, 10, 1>::Ones() * scale.transpose()).array();

  sols.row(0) << V.row(2);
  sols.row(1) << V.row(8);
  sols.row(2) << D.transpose();
  sols.row(3) << V.row(4).array() / sols.row(1).array().pow(2);
  return 0;
}

bool RSPoseFocalLengthRadialDistFromSevenPointsImpl(
    const std::vector<Eigen::Vector2d> &image_points,
    const std::vector<Eigen::Vector3d> &world_points, const double start_row,
    const Eigen::Vector3d &initial_rotational_velocity,
    std::vector<RSLinearizedCameraPose> *results) {

  Eigen::Matrix<double, 7, 3> X;
  Eigen::Matrix<double, 7, 2> u;
  for (int i = 0; i < 7; ++i) {
    X.row(i) = world_points[i];
    u.row(i) = image_points[i];
  }

  results->reserve(10);

  Eigen::Matrix<double, 7, 11> A;
  A.setZero();

  A.col(0) = -X.col(2).array() * u.col(0).array();
  A.col(1) = -X.col(2).array() * u.col(1).array();
  A.col(2) =
      X.col(0).array() * u.col(0).array() + X.col(1).array() * u.col(1).array();
  A.col(3) =
      u.col(0).array() * (X.col(2).array() * (start_row - u.col(0).array()) -
                          X.col(0).array() * initial_rotational_velocity(1) *
                              (start_row - u.col(0).array()) +
                          X.col(1).array() * initial_rotational_velocity(0) *
                              (start_row - u.col(0).array()));
  A.col(4) =
      u.col(1).array() * (X.col(2).array() * (start_row - u.col(0).array()) -
                          X.col(0).array() * initial_rotational_velocity(1) *
                              (start_row - u.col(0).array()) +
                          X.col(1).array() * initial_rotational_velocity(0) *
                              (start_row - u.col(0).array()));
  A.col(5) =
      -u.col(0).array() * (X.col(0).array() * (start_row - u.col(0).array()) -
                           X.col(1).array() * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array()) +
                           X.col(2).array() * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())) -
      u.col(1).array() * (X.col(1).array() * (start_row - u.col(0).array()) +
                          X.col(0).array() * initial_rotational_velocity(2) *
                              (start_row - u.col(0).array()) -
                          X.col(2).array() * initial_rotational_velocity(0) *
                              (start_row - u.col(0).array()));
  A.col(6) = -u.col(1);
  A.col(7) = u.col(0);
  A.col(8) = u.col(1).array() * (start_row - u.col(0).array());
  A.col(9) = -u.col(0).array() * (start_row - u.col(0).array());
  A.col(10) =
      X.col(1).array() * u.col(0).array() - X.col(0).array() * u.col(1).array();

  Eigen::MatrixXd nn = A.fullPivLu().kernel();

  Eigen::MatrixXd n(10, 4);

  for (int i = 0; i < 10; i++) {
    n(i, 0) = nn(i, 0) - (nn(i, 3) * nn(10, 0) / nn(10, 3));
    n(i, 1) = nn(i, 1) - (nn(i, 3) * nn(10, 1) / nn(10, 3));
    n(i, 2) = nn(i, 2) - (nn(i, 3) * nn(10, 2) / nn(10, 3));
    n(i, 3) = nn(i, 3) / nn(10, 3);
  }

  Eigen::VectorXd r2 = u.col(0).array().pow(2) + u.col(1).array().pow(2);

  Eigen::MatrixXd AA(7, 14);

  AA.col(0) =
      -u.col(1).array() *
      (X.col(1).array() * (n(3, 0) * (start_row - u.col(0).array()) - n(0, 0) +
                           n(4, 0) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())) +
       X.col(0).array() * (n(1, 0) - n(4, 0) * (start_row - u.col(0).array()) +
                           n(3, 0) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())) -
       X.col(2).array() * (n(3, 0) * initial_rotational_velocity(0) *
                               (start_row - u.col(0).array()) +
                           n(4, 0) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())));
  AA.col(1) =
      r2.array() *
      (X.col(0).array() * (n(5, 0) * (start_row - u.col(0).array()) - n(2, 0) +
                           n(3, 0) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())) -
       n(7, 0) + n(9, 0) * (start_row - u.col(0).array()) +
       X.col(2).array() * (n(0, 0) - n(3, 0) * (start_row - u.col(0).array()) +
                           n(5, 0) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())) -
       X.col(1).array() * (n(3, 0) * initial_rotational_velocity(0) *
                               (start_row - u.col(0).array()) +
                           n(5, 0) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())));
  AA.col(2) =
      X.col(0).array() * (n(5, 0) * (start_row - u.col(0).array()) - n(2, 0) +
                          n(3, 0) * initial_rotational_velocity(1) *
                              (start_row - u.col(0).array())) -
      n(7, 0) + n(9, 0) * (start_row - u.col(0).array()) +
      X.col(2).array() * (n(0, 0) - n(3, 0) * (start_row - u.col(0).array()) +
                          n(5, 0) * initial_rotational_velocity(1) *
                              (start_row - u.col(0).array())) -
      X.col(1).array() * (n(3, 0) * initial_rotational_velocity(0) *
                              (start_row - u.col(0).array()) +
                          n(5, 0) * initial_rotational_velocity(2) *
                              (start_row - u.col(0).array()));
  AA.col(3) =
      -u.col(1).array() *
      (X.col(1).array() * (n(3, 1) * (start_row - u.col(0).array()) - n(0, 1) +
                           n(4, 1) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())) +
       X.col(0).array() * (n(1, 1) - n(4, 1) * (start_row - u.col(0).array()) +
                           n(3, 1) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())) -
       X.col(2).array() * (n(3, 1) * initial_rotational_velocity(0) *
                               (start_row - u.col(0).array()) +
                           n(4, 1) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())));
  AA.col(4) =
      r2.array() *
      (X.col(0).array() * (n(5, 1) * (start_row - u.col(0).array()) - n(2, 1) +
                           n(3, 1) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())) -
       n(7, 1) + n(9, 1) * (start_row - u.col(0).array()) +
       X.col(2).array() * (n(0, 1) - n(3, 1) * (start_row - u.col(0).array()) +
                           n(5, 1) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())) -
       X.col(1).array() * (n(3, 1) * initial_rotational_velocity(0) *
                               (start_row - u.col(0).array()) +
                           n(5, 1) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())));
  AA.col(7) =
      X.col(0).array() * (n(5, 1) * (start_row - u.col(0).array()) - n(2, 1) +
                          n(3, 1) * initial_rotational_velocity(1) *
                              (start_row - u.col(0).array())) -
      n(7, 1) + n(9, 1) * (start_row - u.col(0).array()) +
      X.col(2).array() * (n(0, 1) - n(3, 1) * (start_row - u.col(0).array()) +
                          n(5, 1) * initial_rotational_velocity(1) *
                              (start_row - u.col(0).array())) -
      X.col(1).array() * (n(3, 1) * initial_rotational_velocity(0) *
                              (start_row - u.col(0).array()) +
                          n(5, 1) * initial_rotational_velocity(2) *
                              (start_row - u.col(0).array()));
  AA.col(8) =
      -u.col(1).array() *
      (X.col(1).array() * (n(3, 2) * (start_row - u.col(0).array()) - n(0, 2) +
                           n(4, 2) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())) +
       X.col(0).array() * (n(1, 2) - n(4, 2) * (start_row - u.col(0).array()) +
                           n(3, 2) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())) -
       X.col(2).array() * (n(3, 2) * initial_rotational_velocity(0) *
                               (start_row - u.col(0).array()) +
                           n(4, 2) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())));
  AA.col(9) =
      r2.array() *
      (X.col(0).array() * (n(5, 2) * (start_row - u.col(0).array()) - n(2, 2) +
                           n(3, 2) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())) -
       n(7, 2) + n(9, 2) * (start_row - u.col(0).array()) +
       X.col(2).array() * (n(0, 2) - n(3, 2) * (start_row - u.col(0).array()) +
                           n(5, 2) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())) -
       X.col(1).array() * (n(3, 2) * initial_rotational_velocity(0) *
                               (start_row - u.col(0).array()) +
                           n(5, 2) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())));
  AA.col(10) =
      X.col(0).array() * (n(5, 2) * (start_row - u.col(0).array()) - n(2, 2) +
                          n(3, 2) * initial_rotational_velocity(1) *
                              (start_row - u.col(0).array())) -
      n(7, 2) + n(9, 2) * (start_row - u.col(0).array()) +
      X.col(2).array() * (n(0, 2) - n(3, 2) * (start_row - u.col(0).array()) +
                          n(5, 2) * initial_rotational_velocity(1) *
                              (start_row - u.col(0).array())) -
      X.col(1).array() * (n(3, 2) * initial_rotational_velocity(0) *
                              (start_row - u.col(0).array()) +
                          n(5, 2) * initial_rotational_velocity(2) *
                              (start_row - u.col(0).array()));
  AA.col(5) = u.col(1).array();
  AA.col(6) = -u.col(1).array() * (start_row - u.col(0).array());
  AA.col(11) =
      -u.col(1).array() *
      (X.col(1).array() * (n(3, 3) * (start_row - u.col(0).array()) - n(0, 3) +
                           n(4, 3) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())) +
       X.col(0).array() * (n(1, 3) - n(4, 3) * (start_row - u.col(0).array()) +
                           n(3, 3) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array())) -
       X.col(2).array() * (n(3, 3) * initial_rotational_velocity(0) *
                               (start_row - u.col(0).array()) +
                           n(4, 3) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array()) +
                           1));
  AA.col(12) =
      r2.array() *
      (X.col(0).array() * (n(5, 3) * (start_row - u.col(0).array()) - n(2, 3) +
                           n(3, 3) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())) -
       n(7, 3) + n(9, 3) * (start_row - u.col(0).array()) +
       X.col(2).array() * (n(0, 3) - n(3, 3) * (start_row - u.col(0).array()) +
                           n(5, 3) * initial_rotational_velocity(1) *
                               (start_row - u.col(0).array())) -
       X.col(1).array() * (n(3, 3) * initial_rotational_velocity(0) *
                               (start_row - u.col(0).array()) +
                           n(5, 3) * initial_rotational_velocity(2) *
                               (start_row - u.col(0).array()) +
                           1));
  AA.col(13) =
      X.col(0).array() * (n(5, 3) * (start_row - u.col(0).array()) - n(2, 3) +
                          n(3, 3) * initial_rotational_velocity(1) *
                              (start_row - u.col(0).array())) -
      n(7, 3) + n(9, 3) * (start_row - u.col(0).array()) +
      X.col(2).array() * (n(0, 3) - n(3, 3) * (start_row - u.col(0).array()) +
                          n(5, 3) * initial_rotational_velocity(1) *
                              (start_row - u.col(0).array())) -
      X.col(1).array() * (n(3, 3) * initial_rotational_velocity(0) *
                              (start_row - u.col(0).array()) +
                          n(5, 3) * initial_rotational_velocity(2) *
                              (start_row - u.col(0).array()) +
                          1);

  Eigen::MatrixXd AR = AA.leftCols(7).lu().solve(AA);

  Eigen::Matrix<double, 35, 1> data;
  data << AR.row(0).tail(7).transpose(), AR.row(1).tail(7).transpose(),
      AR.row(2).tail(7).transpose(), AR.row(3).tail(7).transpose(),
      AR.row(4).tail(7).transpose();

  Eigen::Matrix<std::complex<double>, 4, 10> sols;

  solver_R7Pfr(data, sols);

  for (int i = 0; i < 10; i++) {
    if (!std::isnan(sols(2, i).real()) & std::abs(sols(2, i).real()) > 1e-6 &
        std::abs(sols(2, i).imag()) < 1e-10) {
      RSLinearizedCameraPose res;
      double f = sols(2, i).real();
      if (f <= 0.0) {
          continue;
      }
      double b = sols(0, i).real();
      double c = sols(1, i).real();
      double rd = sols(3, i).real();

      Eigen::Matrix<double, 7, 1> mon;
      mon << b, c * f, c * rd, c, f, rd, 1;
      double a = -AR.row(2).tail(7).dot(mon);
      res.C(2) = (-AR.row(5).tail(7).dot(mon)) / f;
      res.t(2) = (-AR.row(6).tail(7).dot(mon)) / f;
      res.v(0) = a * n(0, 0) + b * n(0, 1) + c * n(0, 2) + n(0, 3);
      res.v(1) = a * n(1, 0) + b * n(1, 1) + c * n(1, 2) + n(1, 3);
      res.v(2) = a * n(2, 0) + b * n(2, 1) + c * n(2, 2) + n(2, 3);
      res.w(0) = a * n(3, 0) + b * n(3, 1) + c * n(3, 2) + n(3, 3);
      res.w(1) = a * n(4, 0) + b * n(4, 1) + c * n(4, 2) + n(4, 3);
      res.w(2) = a * n(5, 0) + b * n(5, 1) + c * n(5, 2) + n(5, 3);
      res.C(0) = a * n(6, 0) + b * n(6, 1) + c * n(6, 2) + n(6, 3);
      res.C(1) = a * n(7, 0) + b * n(7, 1) + c * n(7, 2) + n(7, 3);
      res.t(0) = a * n(8, 0) + b * n(8, 1) + c * n(8, 2) + n(8, 3);
      res.t(1) = a * n(9, 0) + b * n(9, 1) + c * n(9, 2) + n(9, 3);
      res.f = 1 / f;
      res.rd = rd;

      results->push_back(res);
    }
  }

  return results->size() > 0;
}

bool RSPoseFocalLengthRadialDistFromSevenPoints(
    const std::vector<Vector2d> &image_points,
    const std::vector<Vector3d> &world_points, const int row_col_0,
    const RSDirection rs_direction, const int max_iter,
    RSLinearizedCameraPose &result) {

  std::vector<Vector3d> world_points_ = world_points;
  std::vector<Vector2d> image_points_ = image_points;

  // flip x and y if column wise RS
  if (rs_direction == RSDirection::ColWise) {
    SwitchInputXY(world_points_, image_points_);
  }

  result.v = Vector3d::Zero();
  result.C = Vector3d::Zero();
  result.w = Vector3d::Zero();
  result.t = Vector3d::Zero();
  result.f = 1.0;
  result.rd = 0.0;

  int k = 0;
  bool found = false;
  // iterate solver to find a solution
  while (!found && k < max_iter) {
    std::vector<RSLinearizedCameraPose> results;
    double err_prev = std::numeric_limits<double>::max();
    RSPoseFocalLengthRadialDistFromSevenPointsImpl(
        image_points_, world_points_, row_col_0, result.v, &results);
    // if the inner solver returned no solution
    if (!results.size()) {
      std::cout << "RS7pfr solver returned no solution\n";
      return false;
    }

    for (auto const &res : results) {
      double error = RSLinearizedProjectionError(
          image_points_, world_points_, res, RSProjectionType::DoubleLinearized,
          rs_direction, row_col_0);
      if (error < err_prev) {
        result = res;
        err_prev = error;
      }
      if (error < 1e-10) {
        found = true;
        break;
      }
    }
    ++k;
  }

  if (rs_direction == RSDirection::ColWise) {
    result = SwitchOutputXY(result);
  }

  return true;
}
} // namespace theia
