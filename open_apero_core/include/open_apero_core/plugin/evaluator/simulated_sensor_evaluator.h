#pragma once

/**
 * @file simulated_sensor_evaluator.h
 * @brief Base evaluator that simulates a sensor to discover visible voxels and compute gain.
 */

#include <memory>
#include <vector>

#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/map/occupancy_map.h"
#include "open_apero_core/plugin/sensor_model/sensor_model.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace evaluator {

/**
 * @brief Base evaluator for simulated sensor observations.
 *
 * evaluate() runs, for every unobserved node of the cluster: the sensor model's getVisibleVoxelsFromNode(); a filter
 * keeping the voxels inside the bounding volume (when one is configured); with clear_from_parents, removal of voxels a
 * tree ancestor already sees; storeVisibleVoxels(), which replaces node->info with a SimulatedSensorInfo holding them;
 * evaluateFromVisibleVoxels(), which a subclass implements to set the node's gain from that info; and
 * computeFrontierStatus(). updateEvaluation() re-runs evaluate() on the unobserved nodes that are traversable,
 * worth updating (gain above update_gain or at a frontier) and pass nodeSensorViewOverlapsUpdatedMap().
 */
class SimulatedSensorEvaluator : public Evaluator {
 public:
  /** @brief Constructs a SimulatedSensorEvaluator and binds it to the agent. */
  explicit SimulatedSensorEvaluator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /** @brief Visualizes the sensor view frustum and per-voxel gain at the node. */
  void getVisualization(VisualizationMarkers* markers, const Node* node) override;

  /** @brief Loads the sensor model and frontier thresholds. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Resets evaluator state for a new episode. */
  void resetEpisode() override {
    max_num_visible_voxels_ = 0;
    if (sensor_model_) {
      sensor_model_->resetEpisode();
    }
  }

  /**
   * @brief Tests whether the state's sensor view overlaps the map update.
   * @param state The robot state to query.
   * @return true if there is overlap with the updated map region.
   */
  bool sensorViewOverlapsUpdatedMap(const RobotState& state);

  /**
   * @brief Tests whether a sensor pose overlaps the map update.
   * @param sensor_transform Sensor transform in world frame.
   * @return true if there is overlap with the updated map region.
   */
  bool sensorViewOverlapsUpdatedMap(const Transform3d& sensor_transform);

 protected:
  /** @brief Evaluates gain from simulated visible voxels. */
  bool evaluateUnobserved(const NodeClusterPtrs& cluster) override;

  /** @brief Re-evaluates eligible nodes after map changes. */
  bool reevaluateUnobserved(const NodeClusterPtrs& cluster, const Node* current_node) override;

  std::unique_ptr<SensorModel> sensor_model_;

  /** @brief The agent's map as an OccupancyMap; nullptr when the map is of another type. */
  map::OccupancyMap* map_ = nullptr;

  int max_num_visible_voxels_ = 0;

  // Parameters.
  bool p_clear_from_parents_ = false;
  bool p_visualize_sensor_view_ = false;

  double p_frontier_min_unknown_ratio_ = 0.0;
  double p_frontier_min_visible_ratio_ = 0.0;

  double p_update_gain_ = 0.0;  ///< Updates only nodes above this gain.

  bool p_only_update_for_updated_map_ = true;  ///< If false, updates every node.

  /**
   * @brief Tests whether the map update can affect what a node observes.
   *
   * The default tests the node's own sensor pose; evaluators that raycast other poses (e.g.
   * gain anchors with their own orientations) must override it.
   *
   * @param node The node about to be re-evaluated.
   * @return true if the node still has to be re-evaluated.
   */
  virtual bool nodeSensorViewOverlapsUpdatedMap(const Node* node);

  /**
   * @brief Stores the newly visible voxels into the node's data structure.
   * @param node The node to update.
   * @param new_voxels The voxel occupancy map from ray casting.
   * @return true on success.
   */
  virtual bool storeVisibleVoxels(Node* node, const VoxelOccupancyMap& new_voxels);

  /**
   * @brief Computes the frontier status of a node from its visible voxel occupancy.
   * @param node The node whose frontier status will be updated.
   */
  void computeFrontierStatus(Node* node);

  /**
   * @brief Computes the scalar gain from the node's stored visible voxel data.
   * @param node The node to evaluate.
   * @return true on success.
   */
  virtual bool evaluateFromVisibleVoxels(Node* node) = 0;
};

}  // namespace evaluator
}  // namespace open_apero
