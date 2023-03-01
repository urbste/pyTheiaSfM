
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#ifndef SAMPSON_ERROR_H
#define SAMPSON_ERROR_H

#include <ceres/ceres.h>

#include <Eigen/Core>

namespace theia {

struct SampsonError {
 public:
  explicit SampsonError(const Eigen::Vector2d& feature_1,
                        const Eigen::Vector2d& feature_2)
      : feature_1_(feature_1),
        feature_2_(feature_2) {}

  template <typename T>
  bool operator()(const T* fundamental_matrix, T* residual) const {
    Eigen::Map<Eigen::Matrix<T, 1, 1>> res(residual);
    const Eigen::Map<Eigen::Matrix<T, 3, 3> const> F(fundamental_matrix);

    const Eigen::Matrix<T, 3, 1> epiline_x = F * feature_1_.homogeneous();
    const T numerator_sqrt = feature_2_.homogeneous().dot(epiline_x);
    const Eigen::Matrix<T, 4, 1> denominator(feature_2_.homogeneous().dot(F.col(0)),
                                             feature_2_.homogeneous().dot(F.col(1)),
                                             epiline_x[0], epiline_x[1]);
    res[0] = numerator_sqrt * numerator_sqrt / denominator.squaredNorm();
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector2d& feature1,
                                     const Eigen::Vector2d& feature2) {
    static const int kParameterSize = 9;
    return new ceres::AutoDiffCostFunction<SampsonError,
                                           1,
                                           kParameterSize>(
        new SampsonError(feature1, feature2));
  }

 private:
  const Eigen::Vector2d feature_1_;
  const Eigen::Vector2d feature_2_;
};

}  // namespace theia

#endif  // SAMPSON_ERROR_H
