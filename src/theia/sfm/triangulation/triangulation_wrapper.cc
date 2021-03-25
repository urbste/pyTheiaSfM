#include "theia/sfm/triangulation/triangulation_wrapper.h"
#include "theia/sfm/triangulation/triangulation.h"
#include "theia/sfm/types.h"

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace theia {

std::tuple<bool, Vector4d> TriangulateNViewSVDWrapper(
    const std::vector<Matrix3x4d>& poses,
        const std::vector<Vector2d>& points){
    Vector4d triangulated_point;
    const bool success = TriangulateNViewSVD(poses, points, &triangulated_point);
    return std::make_tuple(success, triangulated_point);

};

std::tuple<bool, Vector4d> TriangulateNViewWrapper(const std::vector<Matrix3x4d>& poses,
                                                   const std::vector<Vector2d>& points){
    Vector4d triangulated_point;
    const bool success = TriangulateNView(poses, points, &triangulated_point);
    return std::make_tuple(success, triangulated_point);
}

std::tuple<bool, Vector4d> TriangulateWrapper(const Matrix3x4d& pose1,
                 const Matrix3x4d& pose2,
                 const Vector2d& point1,
                 const Vector2d& point2){
    Vector4d triangulated_point;
    bool success = Triangulate(pose1, pose2, point1, point2, &triangulated_point);
    return std::make_tuple(success, triangulated_point);

}

std::tuple<bool, Vector4d> TriangulateMidpointWrapper(const std::vector<Vector3d>& origins,
                                                             const std::vector<Vector3d>& ray_directions){
    Vector4d triangulated_point;
    const bool success = TriangulateMidpoint(origins, ray_directions, &triangulated_point);
    return std::make_tuple(success, triangulated_point);

}

std::tuple<bool, Vector4d> TriangulateDLTWrapper(const Matrix3x4d& pose1,
                    const Matrix3x4d& pose2,
                    const Vector2d& point1,
                    const Vector2d& point2){
    Vector4d triangulated_point;
    const bool success = TriangulateDLT(pose1, pose2, point1, point2, &triangulated_point);
    return std::make_tuple(success, triangulated_point);

}

}  // namespace theia
