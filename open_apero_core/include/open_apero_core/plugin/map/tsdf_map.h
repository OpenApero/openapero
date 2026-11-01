#pragma once

/**
 * @file tsdf_map.h
 * @brief Abstract interface for TSDF (Truncated Signed Distance Field) maps.
 *
 * Extends OccupancyMap with distance and weight queries specific to TSDF
 * representations. Used mainly by SimpleRobot's traversability check (distances), the sensor
 * models' hit voxel (distances) and VoxelWeightEvaluator (weights).
 */

#include <cmath>
#include <limits>

#include <Eigen/Core>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/plugin/map/occupancy_map.h"

namespace open_apero {
namespace map {

/**
 * @brief Abstract base class for TSDF voxel grid map implementations.
 *
 * Adds signed distance and observation weight queries to the occupancy map interface.
 */
class TsdfMap : public OccupancyMap {
 public:
  /** @brief Constructs a TsdfMap and binds it to the agent. */
  explicit TsdfMap(AgentInterface& agent)
      : OccupancyMap(agent) {}  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  virtual ~TsdfMap() = default;

  /**
   * @brief Returns the ESDF/distance-field value at the given position.
   * @param position Query position in world frame.
   * @param distance Output distance to the nearest obstacle in meters.
   * @return true if a distance value is available.
   */
  virtual bool getDistanceAtPosition(const Eigen::Vector3d& position, double* distance) = 0;

  /**
   * @brief Checks whether a world-frame sphere collides with the distance field.
   * @param position Sphere center in world coordinates.
   * @param radius Sphere radius in meters.
   * @return kColliding, kNotColliding, or kUnknown if the map has no distance value.
   */
  virtual CollisionStatus getSphereCollisionStatus(const Eigen::Vector3d& position, double radius) {
    double distance = 0.0;
    if (!getDistanceAtPosition(position, &distance)) {
      return CollisionStatus::kUnknown;
    }
    return distance > radius ? CollisionStatus::kNotColliding : CollisionStatus::kColliding;
  }

  /**
   * @brief Returns the center of the voxel holding the surface a ray sample reads kOccupied from.
   *
   * Of the voxels one voxel before, at and one voxel past the sample along the ray, the one whose
   * center is closest to the surface. The sample's own voxel when none has a distance value.
   * @param center Output voxel center in world frame.
   * @param sample Ray sample that reads kOccupied.
   * @param direction Unit direction of the ray.
   * @return true if the voxel center was successfully computed.
   */
  bool getSurfaceVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& sample, const Eigen::Vector3d& direction) override {
    if (!getVoxelCenter(center, sample)) {
      return false;
    }
    const Eigen::Vector3d step = getVoxelSize() * direction;
    double closest = std::numeric_limits<double>::infinity();
    Eigen::Vector3d candidate;
    double distance = 0.0;
    for (int i = -1; i <= 1; ++i) {
      if (getVoxelCenter(&candidate, sample + i * step) && getDistanceAtPosition(candidate, &distance) && std::abs(distance) < closest) {
        closest = std::abs(distance);
        *center = candidate;
      }
    }
    return true;
  }

  /**
   * @brief Returns the observation weight stored at the voxel containing the given point.
   * @param point Query point in world frame.
   * @return Observation weight (higher means more confident).
   */
  virtual double getVoxelWeight(const Eigen::Vector3d& point) = 0;

  /**
   * @brief Returns the maximum allowed observation weight, or 0 if weights are uncapped
   *        (VoxbloxMap returns the integrator's max_weight).
   * @return Maximum weight value.
   */
  virtual double getMaximumWeight() = 0;
};

}  // namespace map
}  // namespace open_apero
