/**
 * @file surface_frontier_classifier.cpp
 * @brief Implements the shared surface-frontier neighbor scan.
 */

#include "open_apero_core/plugin/map/surface_frontier_classifier.h"

#include "open_apero_core/common/typedef.h"

namespace open_apero {
namespace map {

SurfaceFrontierClassifier::SurfaceFrontierClassifier(OccupancyMap* map, bool accurate_frontiers, bool surface_frontiers,
                                                     double checking_distance)
    : map_(map), surface_frontiers_(surface_frontiers), num_neighbors_(accurate_frontiers ? 26 : 6) {
  const double vs = map_->getVoxelSize() * checking_distance;
  if (!accurate_frontiers) {
    neighbor_offsets_[0] = Eigen::Vector3d(vs, 0, 0);
    neighbor_offsets_[1] = Eigen::Vector3d(-vs, 0, 0);
    neighbor_offsets_[2] = Eigen::Vector3d(0, vs, 0);
    neighbor_offsets_[3] = Eigen::Vector3d(0, -vs, 0);
    neighbor_offsets_[4] = Eigen::Vector3d(0, 0, vs);
    neighbor_offsets_[5] = Eigen::Vector3d(0, 0, -vs);
  } else {
    neighbor_offsets_[0] = Eigen::Vector3d(vs, 0, 0);
    neighbor_offsets_[1] = Eigen::Vector3d(vs, vs, 0);
    neighbor_offsets_[2] = Eigen::Vector3d(vs, -vs, 0);
    neighbor_offsets_[3] = Eigen::Vector3d(vs, 0, vs);
    neighbor_offsets_[4] = Eigen::Vector3d(vs, vs, vs);
    neighbor_offsets_[5] = Eigen::Vector3d(vs, -vs, vs);
    neighbor_offsets_[6] = Eigen::Vector3d(vs, 0, -vs);
    neighbor_offsets_[7] = Eigen::Vector3d(vs, vs, -vs);
    neighbor_offsets_[8] = Eigen::Vector3d(vs, -vs, -vs);
    neighbor_offsets_[9] = Eigen::Vector3d(0, vs, 0);
    neighbor_offsets_[10] = Eigen::Vector3d(0, -vs, 0);
    neighbor_offsets_[11] = Eigen::Vector3d(0, 0, vs);
    neighbor_offsets_[12] = Eigen::Vector3d(0, vs, vs);
    neighbor_offsets_[13] = Eigen::Vector3d(0, -vs, vs);
    neighbor_offsets_[14] = Eigen::Vector3d(0, 0, -vs);
    neighbor_offsets_[15] = Eigen::Vector3d(0, vs, -vs);
    neighbor_offsets_[16] = Eigen::Vector3d(0, -vs, -vs);
    neighbor_offsets_[17] = Eigen::Vector3d(-vs, 0, 0);
    neighbor_offsets_[18] = Eigen::Vector3d(-vs, vs, 0);
    neighbor_offsets_[19] = Eigen::Vector3d(-vs, -vs, 0);
    neighbor_offsets_[20] = Eigen::Vector3d(-vs, 0, vs);
    neighbor_offsets_[21] = Eigen::Vector3d(-vs, vs, vs);
    neighbor_offsets_[22] = Eigen::Vector3d(-vs, -vs, vs);
    neighbor_offsets_[23] = Eigen::Vector3d(-vs, 0, -vs);
    neighbor_offsets_[24] = Eigen::Vector3d(-vs, vs, -vs);
    neighbor_offsets_[25] = Eigen::Vector3d(-vs, -vs, -vs);
  }
}

bool SurfaceFrontierClassifier::isSurfaceFrontier(const Eigen::Vector3d& voxel) const {
  for (int i = 0; i < num_neighbors_; ++i) {
    const Occupancy voxel_state = map_->getVoxelOccupancy(voxel + neighbor_offsets_[i]);
    if (voxel_state == Occupancy::kUnknown) {
      continue;
    }
    if (surface_frontiers_) {
      return voxel_state == Occupancy::kOccupied;
    }
    return true;
  }
  return false;
}

}  // namespace map
}  // namespace open_apero
