#pragma once

/**
 * @file planning_layer.h
 * @brief Concrete graph-based planning layer.
 */

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/layer/layer.h"
#include "open_apero_core/plugin/layer/planning_layer_metrics.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"

namespace open_apero {
namespace layer {

/**
 * @brief Graph-based planning layer: a GraphGenerator samples and connects viewpoints, an
 *        Evaluator scores them, and a PathPlanner picks the next-best path.
 */
class PlanningLayer : public Layer {
 public:
  explicit PlanningLayer(AgentInterface& agent);
  ~PlanningLayer() override = default;

  PlanningLayer(const PlanningLayer&) = delete;
  PlanningLayer& operator=(const PlanningLayer&) = delete;

  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  void resetEpisode() override;

  /**
   * @brief Initialize the planning graph at @p init_state.
   * Wraps the generator's generateInitNode and makes its node the layer's current_node_.
   */
  void onEpisodeStart(const RobotState& init_state) override;

  /**
   * @brief Advance current_node_ to @p active_node when it is a vertex of this layer's graph,
   *        else insert an intermediate vertex at the end of @p executed_edge; with neither it stays
   *        where it is. Also records the evaluator's accumulated gain and moves the generator's
   *        start node (setNewStartNode).
   */
  void onStateReached(const RobotState& state, const Edge* executed_edge, Node* active_node) override;

  /**
   * @brief Run max_expansions rounds, each sampling one viewpoint cluster, connecting it into the
   *        graph, evaluating its gain and computing its edge costs; then connect current_node_ if
   *        it has no outgoing edge.
   */
  void expand() override;

  /**
   * @brief Prune untraversable nodes/edges, regenerate edges near current_node_,
   *        and re-evaluate gains across all clusters.
   */
  void update() override;

  /**
   * @brief Run the configured PathPlanner over the graph and finalize the
   *        winning edge for execution (refinement, intermediate-node insertion
   *        for replan_num_steps, cost recompute). Returns false when the
   *        planning graph has no traversable outgoing step.
   */
  bool planPath(const RobotState& current_state, Map* map, RobotModel* robot, std::vector<Node*>* planned_path,
                std::vector<Edge*>* planned_edges) override;

  /**
   * @brief Split the active edge at @p waypoint_idx via the layer's
   *        GraphGenerator, using current_node_ as the source vertex.
   */
  bool splitActiveEdge(Node* active_node, Edge* edge, int waypoint_idx, Node** intermediate_out, Edge** finished_edge_out) override;

  /** @brief Number of nodes in the layer's graph or tree; used for planner metrics. */
  int getNodeCount() const;

  /**
   * @brief Force one pass through every owned plugin to trigger any lazy init.
   *        Called by Agent::setupFromParamMap when build_plugins_on_init is true. The path planner
   *        searches the graph only when current_node_ has an outgoing edge, as in planPath().
   *        Precondition: onEpisodeStart() has set current_node_.
   */
  void warmUp() override;

  /**
   * @brief Append graph + per-node gain visualization markers. With @p selected_node, also the
   *        evaluator's markers for the selected path's first evaluated node past current_node_, else
   *        for @p selected_node.
   */
  void getVisualization(Node* selected_node, VisualizationMarkers* markers) override;

  /** @brief Hands the evaluator the path just selected, for its own visualization topic. */
  void getPathVisualization(Node* selected_node, VisualizationMarkers* markers) override;

  /**
   * @brief One-line summary of the graph-size evolution during the last cycle: the node count
   *        after the start-node update, the net change from update and expansion, and the count at
   *        selection.
   */
  std::string cycleSummary() const override;

  /** @brief The evaluator's real accumulated gain; std::nullopt without an evaluator. */
  std::optional<double> realAccumulatedGain() const override;

  /** @brief Current vertex in the planning graph representing the robot's state: the generator's start node. */
  Node* getCurrentNode() const { return current_node_; }

  /** @brief Typed accessor for the layer's own metrics. The base
   *  Layer::getLayerMetrics() returns the same object as LayerMetrics&. */
  PlanningLayerMetrics& getMetrics() { return static_cast<PlanningLayerMetrics&>(*metrics_); }
  const PlanningLayerMetrics& getMetrics() const { return static_cast<const PlanningLayerMetrics&>(*metrics_); }

  std::string layerName() const override { return "planning"; }

 protected:
  /// @brief Raw pointer to the graph owned by generator_; refreshed by onEpisodeStart. nullptr with a tree generator.
  GraphType* graph_ = nullptr;

  /// @brief Current vertex in graph_ representing the robot's state; the generator calls it the start node.
  Node* current_node_ = nullptr;

  /// @brief The path planPath() last offered, for visualization only.
  ///
  /// Valid only while this layer's step was just taken (a non-null selected node); read nowhere else.
  std::vector<Node*> selected_path_;

  /// @brief How many trajectory points before inserting an intermediate node into a
  /// selected edge so we can replan early. INT_MAX disables splitting.
  int p_replan_num_steps_ = std::numeric_limits<int>::max();

  /// @brief Number of expansion rounds run per replan cycle (each round samples, connects,
  /// evaluates and prices one node cluster).
  int p_max_expansions_ = 50;

 private:
  /// @brief Whether current_node_ has a step to offer. True for a tree, which has no graph
  /// to read; throws when current_node_ is not a vertex of graph_.
  bool currentNodeHasOutgoingEdges() const;

  static PluginRegistry::Registration<PlanningLayer> registration;
};

}  // namespace layer
}  // namespace open_apero
