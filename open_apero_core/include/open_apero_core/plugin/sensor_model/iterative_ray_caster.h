#pragma once

/**
 * @file iterative_ray_caster.h
 * @brief Camera ray caster that iteratively subdivides rays to reduce redundancy.
 */

#include <string>
#include <vector>

#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/sensor_model/camera_model.h"

namespace open_apero {
namespace sensor_model {

/**
 * @brief Starts coarse and doubles the ray density in each farther distance section.
 */
class IterativeRayCaster : public CameraModel {
 public:
  /** @brief Constructs the IterativeRayCaster and binds it to the agent. */
  explicit IterativeRayCaster(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  virtual ~IterativeRayCaster() = default;

  /**
   * @brief Returns visible voxels using adaptive ray splitting.
   * @param result Output map of voxel positions to occupancy states.
   * @param sensor_transform Sensor transform in world frame.
   * @return true on success.
   */
  bool getVisibleVoxels(VoxelOccupancyMap* result, const Transform3d& sensor_transform) override;

  /** @brief Loads ray step, downsampling, and split distances. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Validates the derived ray grid, which needs at least two rays per axis.
   * @param error_message Populated with a description of the first validation failure.
   * @return true if all parameters are valid.
   */
  bool checkParamsValid(std::string* error_message) override;

 protected:
  static PluginRegistry::Registration<IterativeRayCaster> registration;

  // Parameters.
  double p_ray_step_ = 0.0;             ///< Defaults to the map voxel size at setup.
  double p_downsampling_factor_ = 1.0;  ///< Reduces the minimum resolution for speed.

  // Derived constants.
  int c_res_x_ = 0;  ///< Effective ray-casting resolution.
  int c_res_y_ = 0;
  int c_n_sections_ = 0;                   ///< Number of ray splits.
  std::vector<double> c_split_distances_;  ///< Distances at which rays split; camera depths with a planar boundary.
  std::vector<int> c_split_widths_;        ///< Rays at the observation boundary covered by each split, per axis.

  /**
   * @brief Marks all rays in the neighborhood of (x, y) at the given segment as covered.
   * @param x         Ray column index.
   * @param y         Ray row index.
   * @param segment   Split segment index at which to mark neighbors.
   * @param value     Value to write into the ray table (e.g., ray id).
   * @param ray_table 2D array tracking which rays have already been cast.
   */
  void markNeighboringRays(int x, int y, int segment, int value, Eigen::ArrayXXi& ray_table);
};

}  // namespace sensor_model
}  // namespace open_apero
