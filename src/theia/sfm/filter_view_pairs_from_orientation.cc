// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.
// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// Steffen Urban


#include "theia/sfm/filter_view_pairs_from_orientation.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <unordered_map>
#include <unordered_set>

#include "theia/math/rotation.h"
#include "theia/math/util.h"
#include "theia/util/hash.h"
#include "theia/util/map_util.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view_graph/view_graph.h"

namespace theia {

namespace {

bool AngularDifferenceIsAcceptable(
    const Eigen::Vector3d& orientation1,
    const Eigen::Vector3d& orientation2,
    const Eigen::Vector3d& relative_orientation,
    const double sq_max_relative_rotation_difference_radians) {
  const Eigen::Vector3d composed_relative_rotation =
      MultiplyRotations(orientation2, -orientation1);
  const Eigen::Vector3d loop_rotation =
      MultiplyRotations(-relative_orientation, composed_relative_rotation);
  const double sq_rotation_angular_difference_radians =
      loop_rotation.squaredNorm();
  return sq_rotation_angular_difference_radians <=
         sq_max_relative_rotation_difference_radians;
}

}  // namespace

void FilterViewPairsFromOrientation(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    const double max_relative_rotation_difference_degrees,
    ViewGraph* view_graph) {
  CHECK_NOTNULL(view_graph);
  CHECK_GE(max_relative_rotation_difference_degrees, 0.0);

  // Precompute the squared threshold in radians.
  const double max_relative_rotation_difference_radians =
      DegToRad(max_relative_rotation_difference_degrees);
  const double sq_max_relative_rotation_difference_radians =
      max_relative_rotation_difference_radians *
      max_relative_rotation_difference_radians;

  std::unordered_set<ViewIdPair> view_pairs_to_remove;
  const auto& view_pairs = view_graph->GetAllEdges();
  for (const auto& view_pair : view_pairs) {
    const Eigen::Vector3d* orientation1 =
        FindOrNull(orientations, view_pair.first.first);
    const Eigen::Vector3d* orientation2 =
        FindOrNull(orientations, view_pair.first.second);

    // If the view pair contains a view that does not have an orientation then
    // remove it.
    if (orientation1 == nullptr || orientation2 == nullptr) {
      LOG(WARNING)
          << "View pair (" << view_pair.first.first << ", "
          << view_pair.first.second
          << ") contains a view that does not exist! Removing the view pair.";
      view_pairs_to_remove.insert(view_pair.first);
      continue;
    }

    // Remove the view pair if the relative rotation estimate is not within the
    // tolerance.
    if (!AngularDifferenceIsAcceptable(
            *orientation1,
            *orientation2,
            view_pair.second.rotation_2,
            sq_max_relative_rotation_difference_radians)) {
      view_pairs_to_remove.insert(view_pair.first);
    }
  }

  // Remove all the "bad" relative poses.
  for (const ViewIdPair view_id_pair : view_pairs_to_remove) {
    view_graph->RemoveEdge(view_id_pair.first, view_id_pair.second);
  }
  VLOG(1) << "Removed " << view_pairs_to_remove.size()
          << " view pairs by rotation filtering.";
}

}  // namespace theia
