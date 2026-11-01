#pragma once

/**
 * @file occupancy_map.h
 * @brief Abstract interface for voxel-based occupancy grid maps.
 *
 * Extends the Map interface with voxel-level queries for occupancy state,
 * voxel size, center computation, and optional distance field lookups.
 */

#include <Eigen/Core>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace map {

/**
 * @brief Abstract base class for occupancy grid map implementations.
 *
 * Provides voxel-level occupancy queries on top of the Map interface.
 */
class OccupancyMap : public Map {
 public:
  /** @brief Constructs an OccupancyMap and binds it to the agent. */
  explicit OccupancyMap(AgentInterface& agent) : Map(agent) {}
  virtual ~OccupancyMap() = default;

  /**
   * @brief Returns the occupancy state of the voxel containing the given point.
   * @param point Query point in world frame.
   * @return Occupancy state of the voxel.
   */
  virtual Occupancy getVoxelOccupancy(const Eigen::Vector3d& point) = 0;

  /**
   * @brief Returns the side length of a single voxel.
   * @return Voxel size in meters.
   */
  virtual double getVoxelSize() = 0;

  /**
   * @brief Returns the center position of the voxel containing the given point.
   * @param center Output voxel center in world frame.
   * @param point Query point in world frame.
   * @return true if the voxel center was successfully computed.
   */
  virtual bool getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) = 0;

  /**
   * @brief Returns the center of the voxel holding the surface a ray sample reads kOccupied from.
   *
   * The default is the sample's own voxel.
   * @param center Output voxel center in world frame.
   * @param sample Ray sample that reads kOccupied.
   * @param direction Unit direction of the ray.
   * @return true if the voxel center was successfully computed.
   */
  virtual bool getSurfaceVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& sample, const Eigen::Vector3d& direction);
};

inline bool OccupancyMap::getSurfaceVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& sample,
                                                const Eigen::Vector3d& /*direction*/) {
  return getVoxelCenter(center, sample);
}

}  // namespace map
}  // namespace open_apero
