#pragma once

/**
 * @file backtracking_layer.h
 * @brief Layer that owns a Backtracker plugin and produces recovery steps.
 *
 * Its baseline priority is low, and it is promoted while the backtracker's initial
 * exploration (e.g. a one-time rotation) is running or the current state is untraversable.
 */

#include <memory>
#include <string>
#include <vector>

#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/backtracker/backtracker.h"
#include "open_apero_core/plugin/layer/backtracking_layer_metrics.h"
#include "open_apero_core/plugin/layer/layer.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace layer {

/**
 * @brief Layer wrapping a Backtracker plugin. Owns its own two-node-one-edge
 *        recovery state; never reaches into another layer's storage.
 */
class BacktrackingLayer : public Layer {
 public:
  explicit BacktrackingLayer(AgentInterface& agent);
  ~BacktrackingLayer() override = default;

  BacktrackingLayer(const BacktrackingLayer&) = delete;
  BacktrackingLayer& operator=(const BacktrackingLayer&) = delete;

  /// @brief Construct the owned Backtracker from YAML and hand it to the metrics, so its columns join the
  /// layer's. Does NOT call Layer::setupFromParamMap: the backtracker is the only child (edge costs come
  /// from the agent's cost computer).
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /// @brief Forwards to the owned Backtracker's resetEpisode.
  void resetEpisode() override;

  /// @brief Caches the latest robot state and lets the backtracker record
  /// non-recovery executed edges (so e.g. Reverse can queue forward segments).
  void onStateReached(const RobotState& state, const Edge* executed_edge, Node* active_node) override;

  /// @brief No graph to extend — no-op.
  void expand() override {}

  /// @brief No state to re-evaluate — no-op.
  void update() override {}

  /// @brief Ask the backtracker for a recovery trajectory and wrap it as a
  /// step. Returns false if the strategy declines.
  bool planPath(const RobotState& current_state, Map* map, RobotModel* robot, std::vector<Node*>* planned_path,
                std::vector<Edge*>* planned_edges) override;

  /// @brief Truncate the active recovery edge to its executed prefix [0, waypoint_idx] and return it
  /// as the finished edge, with a new recovery_node_ at its end as the intermediate. Fails only on
  /// invalid arguments.
  bool splitActiveEdge(Node* active_node, Edge* edge, int waypoint_idx, Node** intermediate_out, Edge** finished_edge_out) override;

  /// @brief Returns promoted_priority while the backtracker's initial sweep runs or @p state
  /// is untraversable, the configured baseline otherwise.
  int getPriority(const RobotState& state) const override;

  /// @brief Configured baseline plus the promoted priority (initial sweep running or untraversable state).
  std::vector<int> possiblePriorities() const override { return {configuredPriority(), p_promoted_priority_}; }

  /// @brief Forward debug markers from the owned backtracker.
  void getVisualization(Node* selected_node, VisualizationMarkers* markers) override;

  /// @brief Accessor for the owned backtracker (used by tests / debug).
  Backtracker& getBacktracker() { return *backtracker_; }

  /// @brief This layer's metrics, already downcast. The constructor installs a BacktrackingLayerMetrics, so
  /// the cast always holds; Layer::getLayerMetrics() returns the same object as LayerMetrics&.
  BacktrackingLayerMetrics& getMetrics() { return static_cast<BacktrackingLayerMetrics&>(*metrics_); }
  const BacktrackingLayerMetrics& getMetrics() const { return static_cast<const BacktrackingLayerMetrics&>(*metrics_); }

 protected:
  std::string layerName() const override { return "backtracking"; }
  int defaultPriority() const override { return -1000; }

  std::unique_ptr<Backtracker> backtracker_;

  /// @brief Stub node carrying the layer's view of the robot's current state.
  /// Used only as planned_path[0]; the agent consumes [1].
  std::shared_ptr<Node> current_node_;

  /// @brief Destination node of the most recent recovery proposal.
  std::shared_ptr<Node> recovery_node_;

  /// @brief Edge wrapping the recovery trajectory; owned by the layer.
  std::shared_ptr<Edge> recovery_edge_;

  /// @brief Priority while the initial sweep runs or the robot's state is untraversable.
  int p_promoted_priority_ = 1000;

 private:
  static PluginRegistry::Registration<BacktrackingLayer> registration;
};

}  // namespace layer
}  // namespace open_apero
