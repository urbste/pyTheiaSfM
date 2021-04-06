#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <tuple>
#include "theia/sfm/camera/camera.h"
typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;

namespace theia {

Eigen::Matrix3d IntrinsicsToCalibrationMatrixWrapper(const double focal_length,
                                   const double skew,
                                   const double aspect_ratio,
                                   const double principal_point_x,
                                   const double principal_point_y);

std::tuple<double,double,double,double,double> CalibrationMatrixToIntrinsicsWrapper(const Eigen::Matrix3d& calibration_matrix);

std::tuple<bool,Eigen::Matrix3d,Eigen::Vector3d,Eigen::Vector3d> DecomposeProjectionMatrixWrapper(const Matrix3x4d pmatrix);

std::tuple<bool, Matrix3x4d> ComposeProjectionMatrixWrapper(const Eigen::Matrix3d& calibration_matrix,
                             const Eigen::Vector3d& rotation,
                             const Eigen::Vector3d& position);


}  // namespace theia
