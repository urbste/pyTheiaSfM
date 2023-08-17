#include "theia/math/math_wrapper.h"

namespace theia {

std::unordered_map<ViewId, Eigen::Vector3d> AlignOrientationsWrapper(
    const std::unordered_map<ViewId, Eigen::Vector3d>& gt_rotations,
    const std::unordered_map<ViewId, Eigen::Vector3d>& rotations) {
  std::unordered_map<ViewId, Eigen::Vector3d> rot_aligned = rotations;
  theia::AlignOrientations(gt_rotations, &rot_aligned);

  return rot_aligned;
}

}  // namespace theia