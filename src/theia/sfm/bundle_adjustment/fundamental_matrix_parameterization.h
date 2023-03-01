// Please contact the author of this library if you have any questions.
// Author: Steffen Urban

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_FUNDAMENTAL_MATRIX_PARAMETERIZATION_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_FUNDAMENTAL_MATRIX_PARAMETERIZATION_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace theia {

struct FundamentalMatrixParametrization {
  template <typename T>
  bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Mat3T = Eigen::Matrix<T, 3, 3>;

    // get current fundamental matrix
    const Eigen::Map<Mat3T const> F(x);
    Eigen::Map<Mat3T> F_plus_delta(x_plus_delta);
    const Eigen::Map<Eigen::Matrix<T, 7, 1> const> delta_(delta);

    Mat3T R1, R2;
    ceres::AngleAxisToRotationMatrix(delta_.segment(0,3).data(), R1.data());
    ceres::AngleAxisToRotationMatrix(delta_.segment(3,3).data(), R2.data());

    Eigen::JacobiSVD<Mat3T> svd(F,
        Eigen::ComputeFullV | Eigen::ComputeFullU);
    Mat3T U = svd.matrixU();
    Mat3T V = svd.matrixV();
    Vec3T s = svd.singularValues();
    // parameter update step (equation 3)
    Mat3T U_new = U * R1;
    Mat3T V_new = V * R2;
    const T sigma = s(1) / s(0) + delta_(6);

    // get back full F
    // fundamental matrix update step (equation 2)
    F_plus_delta = U_new.col(0) * V_new.col(0).transpose() + sigma * U_new.col(1) * V_new.col(1).transpose();
    return true;
  }
};

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_FUNDAMENTAL_MATRIX_PARAMETERIZATION_H_
