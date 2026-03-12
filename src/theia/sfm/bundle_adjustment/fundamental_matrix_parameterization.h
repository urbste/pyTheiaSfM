// Please contact the author of this library if you have any questions.
// Author: Steffen Urban

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_FUNDAMENTAL_MATRIX_PARAMETERIZATION_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_FUNDAMENTAL_MATRIX_PARAMETERIZATION_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace theia {

// Functor for ceres::AutoDiffManifold: ambient 9, tangent 7.
struct FundamentalMatrixParametrization {
  template <typename T>
  bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Mat3T = Eigen::Matrix<T, 3, 3>;

    const Eigen::Map<const Mat3T> F(x);
    Eigen::Map<Mat3T> F_plus_delta(x_plus_delta);
    const Eigen::Map<const Eigen::Matrix<T, 7, 1>> delta_(delta);

    Mat3T R1, R2;
    ceres::AngleAxisToRotationMatrix(delta_.segment(0, 3).data(), R1.data());
    ceres::AngleAxisToRotationMatrix(delta_.segment(3, 3).data(), R2.data());

    Eigen::JacobiSVD<Mat3T> svd(F,
        Eigen::ComputeFullV | Eigen::ComputeFullU);
    Mat3T U = svd.matrixU();
    Mat3T V = svd.matrixV();
    Vec3T s = svd.singularValues();
    const T sigma = s(1) / s(0) + delta_(6);
    Mat3T U_new = U * R1;
    Mat3T V_new = V * R2;
    F_plus_delta = U_new.col(0) * V_new.col(0).transpose() +
                  sigma * U_new.col(1) * V_new.col(1).transpose();
    return true;
  }

  template <typename T>
  bool Minus(const T* y, const T* x, T* y_minus_x) const {
    using Mat3T = Eigen::Matrix<T, 3, 3>;
    Eigen::Map<Eigen::Matrix<T, 7, 1>> result(y_minus_x);

    const Eigen::Map<const Mat3T> F_x(x);
    const Eigen::Map<const Mat3T> F_y(y);

    Eigen::JacobiSVD<Mat3T> svd_x(F_x,
        Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Mat3T> svd_y(F_y,
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    Mat3T U_x = svd_x.matrixU();
    Mat3T V_x = svd_x.matrixV();
    Mat3T U_y = svd_y.matrixU();
    Mat3T V_y = svd_y.matrixV();
    Eigen::Matrix<T, 3, 1> s_x = svd_x.singularValues();
    Eigen::Matrix<T, 3, 1> s_y = svd_y.singularValues();

    Mat3T R1 = U_x.transpose() * U_y;
    Mat3T R2 = V_x.transpose() * V_y;
    T angle_axis_1[3], angle_axis_2[3];
    ceres::RotationMatrixToAngleAxis(R1.data(), angle_axis_1);
    ceres::RotationMatrixToAngleAxis(R2.data(), angle_axis_2);
    result(0) = angle_axis_1[0];
    result(1) = angle_axis_1[1];
    result(2) = angle_axis_1[2];
    result(3) = angle_axis_2[0];
    result(4) = angle_axis_2[1];
    result(5) = angle_axis_2[2];
    result(6) = (s_y(1) / s_y(0)) - (s_x(1) / s_x(0));
    return true;
  }
};

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_FUNDAMENTAL_MATRIX_PARAMETERIZATION_H_
