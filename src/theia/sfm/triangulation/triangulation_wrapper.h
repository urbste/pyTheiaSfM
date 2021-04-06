#include "theia/sfm/types.h"
#include <vector>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace theia {

std::tuple<bool, Eigen::Vector4d> TriangulateWrapper(const Matrix3x4d& pose1,
                 const Matrix3x4d& pose2,
                 const Eigen::Vector2d& point1,
                 const Eigen::Vector2d& point2);


std::tuple<bool, Eigen::Vector4d> TriangulateMidpointWrapper(const std::vector<Eigen::Vector3d>& origins,
                         const std::vector<Eigen::Vector3d>& ray_directions);

std::tuple<bool, Eigen::Vector4d>  TriangulateDLTWrapper(const Matrix3x4d& pose1,
                    const Matrix3x4d& pose2,
                    const Eigen::Vector2d& point1,
                    const Eigen::Vector2d& point2);


std::tuple<bool, Eigen::Vector4d> TriangulateNViewSVDWrapper(
    const std::vector<Matrix3x4d>& poses,
    const std::vector<Eigen::Vector2d>& points);

std::tuple<bool, Eigen::Vector4d> TriangulateNViewWrapper(const std::vector<Matrix3x4d>& poses,
                      const std::vector<Eigen::Vector2d>& points);


}  // namespace theia
