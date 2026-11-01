#pragma once

/**
 * @file surface_frontier_evaluator.h
 * @brief Evaluator that weights visible unknown voxels, frontier voxels the most.
 */

#include <optional>

#include <Eigen/Core>

#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/evaluator/simulated_sensor_evaluator.h"
#include "open_apero_core/plugin/map/occupancy_map.h"
#include "open_apero_core/plugin/map/surface_frontier_classifier.h"

namespace open_apero {
namespace evaluator {

/**
 * @brief Scores nodes by their visible surface frontiers.
 */
class SurfaceFrontierEvaluator : public SimulatedSensorEvaluator {
 public:
  /** @brief Constructs a SurfaceFrontierEvaluator and binds it to the agent. */
  explicit SurfaceFrontierEvaluator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /** @brief Visualizes per-voxel frontier status at the node. */
  void getVisualization(VisualizationMarkers* markers, const Node* node) override;

  /**
   * @brief Returns the map's mapped surface area.
   * @return Mapped surface area [m^2].
   */
  double getRealAccumulatedGain() const override;

  /**
   * @brief Loads frontier evaluator parameters (weights, checking distance) from the config map.
   * @param param_map Pointer to the plugin parameter map.
   * @throws std::runtime_error if the agent's map is not an OccupancyMap.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

 protected:
  static PluginRegistry::Registration<SurfaceFrontierEvaluator> registration;

  /**
   * @brief Computes gain by counting frontier voxels among visible voxels at the node.
   * @param node The node to evaluate.
   * @return true on success.
   */
  bool evaluateFromVisibleVoxels(Node* node) override;

  bool p_accurate_frontiers_ = false;  ///< Use all 26 neighbors instead of 6 face neighbors.
  bool p_surface_frontiers_ = true;    ///< Require an occupied neighbor instead of any observed neighbor.
  double p_checking_distance_ = 1.0;   ///< Neighbor offset in voxel widths.

  double p_new_voxel_weight_ = 0.01;      ///< Weight assigned to unobserved voxels.
  double p_frontier_voxel_weight_ = 1.0;  ///< Weight assigned to frontier voxels.

  std::optional<map::SurfaceFrontierClassifier> frontier_classifier_;  ///< Built once the parameters are known.

  /**
   * @brief Returns whether the given voxel is a frontier: unknown and next to an occupied voxel
   *        (surface_frontiers) or to any observed voxel.
   * @param voxel The voxel center position to check.
   * @return true if the voxel is a surface frontier.
   */
  bool isSurfaceFrontier(const Eigen::Vector3d& voxel) const;
};

}  // namespace evaluator
}  // namespace open_apero
