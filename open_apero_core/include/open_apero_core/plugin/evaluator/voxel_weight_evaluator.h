#pragma once

/**
 * @file voxel_weight_evaluator.h
 * @brief Evaluator that uses TSDF voxel weights to estimate potential observation impact.
 */

#include <vector>

#include <Eigen/Core>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/evaluator/surface_frontier_evaluator.h"
#include "open_apero_core/plugin/map/tsdf_map.h"

namespace open_apero {
namespace evaluator {

/**
 * @brief Estimates observation impact from TSDF weights and frontier gain.
 */
class VoxelWeightEvaluator : public SurfaceFrontierEvaluator {
 public:
  /** @brief Constructs a VoxelWeightEvaluator and binds it to the agent. */
  explicit VoxelWeightEvaluator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /** @brief Visualizes per-voxel weight-based gain contribution at the node. */
  void getVisualization(VisualizationMarkers* markers, const Node* node) override;

  /**
   * @brief Loads the impact factor and voxel weights.
   * @param param_map Pointer to the plugin parameter map.
   * @throws std::runtime_error if the agent's map is not a TsdfMap.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

 protected:
  static PluginRegistry::Registration<VoxelWeightEvaluator> registration;

  /**
   * @brief Computes gain from TSDF voxel weights combined with frontier counts.
   * @param node The node to evaluate.
   * @return true on success.
   */
  bool evaluateFromVisibleVoxels(Node* node) override;

  /** @brief The agent's map as a TsdfMap (the inherited map_, typed for weight queries). */
  map::TsdfMap* tsdf_map_ = nullptr;

  double p_min_impact_factor_ = 0.0;  ///< Contributions below this factor are zero.
  double p_ray_angle_x_ = 0.0025;     ///< Horizontal angle per pixel [rad].
  double p_ray_angle_y_ = 0.0025;     ///< Vertical angle per pixel [rad].

  double c_voxel_size_ = 0.0;

  /**
   * @brief Computes a voxel's gain from TSDF weight and occupancy.
   * @param voxel Voxel center position.
   * @param origin Sensor origin used for distance-based weighting.
   * @param occupancy Occupancy state of the voxel.
   * @return Scalar gain contribution for this voxel.
   */
  double getVoxelValue(const Eigen::Vector3d& voxel, const Eigen::Vector3d& origin, Occupancy occupancy);
};

}  // namespace evaluator
}  // namespace open_apero
