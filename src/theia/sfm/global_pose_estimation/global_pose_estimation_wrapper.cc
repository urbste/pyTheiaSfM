
#include "theia/sfm/global_pose_estimation/global_pose_estimation_wrapper.h"
#include "theia/sfm/global_pose_estimation/compute_triplet_baseline_ratios.h"

namespace theia {

std::tuple<bool, Eigen::Vector3d> ComputeTripletBaselineRatiosWrapper(const ViewTriplet& triplet,
                                  const std::vector<Feature>& feature1,
                                  const std::vector<Feature>& feature2,
                                  const std::vector<Feature>& feature3){
    Eigen::Vector3d baseline;
    const bool success = ComputeTripletBaselineRatios(triplet, feature1, feature2, feature3, &baseline);
    return std::make_tuple(success, baseline);
}

}  // namespace theia
