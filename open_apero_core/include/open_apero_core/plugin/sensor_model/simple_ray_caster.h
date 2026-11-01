#pragma once

/**
 * @file simple_ray_caster.h
 * @brief Simple camera ray caster that casts one ray per voxel at maximum distance resolution.
 */

#include <string>
#include <vector>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/sensor_model/camera_model.h"

namespace open_apero {
namespace sensor_model {

/**
 * @brief Casts one ray per voxel at the observation boundary.
 */
class SimpleRayCaster : public CameraModel {
 public:
  /** @brief Constructs the SimpleRayCaster and binds it to the agent. */
  explicit SimpleRayCaster(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  virtual ~SimpleRayCaster() = default;

  /**
   * @brief Casts one ray per voxel column at max-distance resolution and collects visible voxels.
   * @param result Output map of voxel positions to occupancy states.
   * @param sensor_transform Sensor transform in world frame.
   * @return true on success.
   */
  bool getVisibleVoxels(VoxelOccupancyMap* result, const Transform3d& sensor_transform) override;

  /** @brief Loads SimpleRayCaster parameters (ray step, downsampling factor) from the config map. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Validates the camera, ray_step > 0, downsampling_factor > 0 and at least 2 rays per axis. */
  bool checkParamsValid(std::string* error_message) override;

 protected:
  static PluginRegistry::Registration<SimpleRayCaster> registration;

  // params
  double p_ray_step_ = 0.0;             // defaults to the map voxel size at setup
  double p_downsampling_factor_ = 1.0;  // Artificially reduce the minimum resolution to increase performance

  // constants
  int c_res_x_ = 0;  // factual resolution that is used for ray casting
  int c_res_y_ = 0;
};

}  // namespace sensor_model
}  // namespace open_apero
