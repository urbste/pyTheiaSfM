#include <theia/sfm/camera/camera_wrapper.h>
#include <theia/sfm/camera/projection_matrix_utils.h>

namespace theia {

Eigen::Matrix3d IntrinsicsToCalibrationMatrixWrapper(const double focal_length,
                                   const double skew,
                                   const double aspect_ratio,
                                   const double principal_point_x,
                                   const double principal_point_y){
    Eigen::Matrix3d calibration_matrix;
    IntrinsicsToCalibrationMatrix(focal_length, skew, aspect_ratio, principal_point_x, principal_point_y, &calibration_matrix);
    return calibration_matrix;
}

std::tuple<double,double,double,double,double> CalibrationMatrixToIntrinsicsWrapper(const Eigen::Matrix3d& calibration_matrix){
    double focal_length;
    double skew;
    double aspect_ratio;
    double principal_point_x;
    double principal_point_y;
    CalibrationMatrixToIntrinsics(calibration_matrix,&focal_length,&skew,&aspect_ratio,&principal_point_x,&principal_point_y);
    return std::make_tuple(focal_length,skew,aspect_ratio,principal_point_x,principal_point_y);
}

std::tuple<bool,Eigen::Matrix3d,Eigen::Vector3d,Eigen::Vector3d> DecomposeProjectionMatrixWrapper(const Matrix3x4d pmatrix){

     Eigen::Matrix3d calibration_matrix;
     Eigen::Vector3d rotation;
     Eigen::Vector3d position;
     const bool success = DecomposeProjectionMatrix(pmatrix, &calibration_matrix, &rotation, &position);
     return std::make_tuple(success,calibration_matrix,rotation,position);
}

std::tuple<bool, Matrix3x4d> ComposeProjectionMatrixWrapper(const Eigen::Matrix3d& calibration_matrix,
                             const Eigen::Vector3d& rotation,
                             const Eigen::Vector3d& position){
    Matrix3x4d pmatrix;
    const bool success = ComposeProjectionMatrix(calibration_matrix,rotation,position,&pmatrix);
    return std::make_tuple(success,pmatrix);
}


}  // namespace theia
