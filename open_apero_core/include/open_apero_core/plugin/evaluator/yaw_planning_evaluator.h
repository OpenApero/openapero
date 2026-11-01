#pragma once

/**
 * @file yaw_planning_evaluator.h
 * @brief Base class for evaluators that plan optimal yaw orientations along trajectories.
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "open_apero_core/common/seeding.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/evaluator/simulated_sensor_evaluator.h"

namespace open_apero {
namespace evaluator {

/**
 * @brief Base evaluator for testing and selecting yaw orientations.
 */
class YawPlanningEvaluator : public Evaluator {
 public:
  /** @brief Constructs a YawPlanningEvaluator and binds it to the agent. */
  explicit YawPlanningEvaluator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  void resetEpisode() override {
    if (following_evaluator_) {
      following_evaluator_->resetEpisode();
    }
    rng_.reseed(agent_.getRandomSeed());
  }

  /** @brief Visualizes the gain at the best yaw orientation found for this node. */
  void getVisualization(VisualizationMarkers* markers, const Node* node) override;

  /**
   * @brief Marks the active orientation as visited and forwards to the following evaluator.
   * @param reached_node   The node the robot just navigated to.
   * @param traversed_edge Edge used to reach the node, or nullptr.
   */
  void onNodeReached(Node* reached_node, const Edge* traversed_edge) override;

  /**
   * @brief Loads yaw settings and creates the delegated evaluator.
   * @throws std::runtime_error if the following evaluator is not a SimulatedSensorEvaluator.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

 protected:
  /**
   * @brief Tests multiple yaw orientations and delegates gain computation to the following evaluator.
   * @param cluster The candidate viewpoint nodes.
   * @return true on success.
   */
  bool evaluateUnobserved(const NodeClusterPtrs& cluster) override;

  /** @brief Re-evaluates gains across all sampled yaw orientations when the map changes.
   * If current_node matches the cluster, gains are refreshed without changing yaw.
   */
  bool reevaluateUnobserved(const NodeClusterPtrs& cluster, const Node* current_node) override;

  // members
  std::unique_ptr<Evaluator> following_evaluator_;

  /// following_evaluator_ as the SimulatedSensorEvaluator setup requires it to be.
  SimulatedSensorEvaluator* sensor_evaluator_ = nullptr;

  double p_update_gain_ = 0.0;  // Update only orientations whose gain is at least this

  /**
   * @brief Sets the sensor orientation of a node and updates its trajectory.
   * @param node The node to reorient.
   * @param orientation The new sensor orientation quaternion.
   * @return true on success.
   */
  virtual bool setNodeOrientation(Node* node, const Eigen::Quaterniond& orientation);

  /**
   * @brief Samples yaw orientations and sets the one with highest gain on the node.
   * @param node The node to optimize.
   * @return true on success.
   */
  virtual bool setBestYaw(Node* node);

  // Reproducible RNG for shuffling tied/random yaw candidates.
  SeededRng rng_;
};

}  // namespace evaluator
}  // namespace open_apero
