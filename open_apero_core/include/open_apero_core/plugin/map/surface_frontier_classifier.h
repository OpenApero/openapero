#pragma once

/**
 * @file surface_frontier_classifier.h
 * @brief Standalone surface-frontier predicate shared by evaluators and planners.
 */

#include <Eigen/Core>

#include "open_apero_core/plugin/map/occupancy_map.h"

namespace open_apero {
namespace map {

/**
 * @brief Classifies unknown voxels by their first observed neighbor: occupied for a surface
 *        frontier, any observed state otherwise.
 *
 * Pair it with a visibility source: such voxels lie behind walls as readily as in front. The
 * scan stops at the first observed neighbor, so the verdict depends on the offset order
 * (kept so the published results reproduce).
 */
class SurfaceFrontierClassifier {
 public:
  /**
   * @brief Builds the neighbor offsets for one occupancy map.
   * @param map Occupancy source, which must outlive this object.
   * @param accurate_frontiers Scan all 26 neighbors instead of the 6 face neighbors.
   * @param surface_frontiers Require an occupied neighbor instead of any observed neighbor.
   * @param checking_distance Neighbor offset in voxel widths.
   */
  SurfaceFrontierClassifier(OccupancyMap* map, bool accurate_frontiers, bool surface_frontiers, double checking_distance);

  /**
   * @brief Returns whether the voxel at @p voxel borders the mapped surface.
   * @param voxel Voxel center position in world frame.
   * @return true when the first observed neighbor satisfies the configured mode.
   */
  bool isSurfaceFrontier(const Eigen::Vector3d& voxel) const;

 private:
  OccupancyMap* map_;
  bool surface_frontiers_;
  int num_neighbors_;
  Eigen::Vector3d neighbor_offsets_[26];
};

}  // namespace map
}  // namespace open_apero
