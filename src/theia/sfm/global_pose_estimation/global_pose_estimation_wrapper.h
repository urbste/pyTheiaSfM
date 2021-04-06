
#include <Eigen/Core>
#include <vector>

#include "theia/sfm/view_triplet.h"
#include "theia/sfm/feature.h"

namespace theia {

std::tuple<bool, Eigen::Vector3d> ComputeTripletBaselineRatiosWrapper(const ViewTriplet& triplet,
                                  const std::vector<Feature>& feature1,
                                  const std::vector<Feature>& feature2,
                                  const std::vector<Feature>& feature3);

}  // namespace theia
