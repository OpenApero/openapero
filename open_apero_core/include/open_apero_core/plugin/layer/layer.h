#pragma once

/**
 * @file layer.h
 * @brief Abstract base class for planner layers.
 *
 * Agent walks its layers by descending getPriority() every tick; the first whose planPath()
 * returns true wins. Priorities must be distinct. The core provides PlanningLayer (graph-based
 * planning) and BacktrackingLayer (recovery); applications may register more.
 */

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/cost_computer/cost_computer.h"
#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/generator/generator.h"
#include "open_apero_core/plugin/layer/layer_metrics.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/**
 * @brief Abstract planner layer that keeps its own view of the current state and offers steps.
 *
 * Owned by Agent. A Layer is a Plugin; the default setup composes a generator, an evaluator and a
 * path planner created through the agent's factory, which a layer may replace with its own
 * (BacktrackingLayer). RobotState is the cross-layer currency.
 */
class Layer : public Plugin {
 public:
  explicit Layer(AgentInterface& agent) : Plugin(agent), metrics_(std::make_unique<LayerMetrics>()) {}
  ~Layer() override = default;

  /**
   * @brief The layer's semantic kind, fixed per concrete class (e.g. "planning", "backtracking").
   *
   * Not the class name or the registered type string. Runtime identity (CSV column prefix,
   * visualization topic, logs) comes from instanceName().
   */
  virtual std::string layerName() const = 0;

  /**
   * @brief Runtime instance identity: the "layers" dictionary key this layer was configured under.
   *
   * Falls back to layerName() for layers created outside that path (tests).
   */
  std::string instanceName() const { return instance_name_.empty() ? layerName() : instance_name_; }

  /// @brief Set the runtime instance name (see instanceName()).
  void setInstanceName(const std::string& instance_name) { instance_name_ = instance_name; }

  /** @brief This layer's metrics; subclasses install a derived LayerMetrics in their constructor. */
  LayerMetrics& getLayerMetrics() { return *metrics_; }
  const LayerMetrics& getLayerMetrics() const { return *metrics_; }

  /**
   * @brief Loads the layer's params and creates the generator, evaluator and path_planner children.
   *
   * Edge costs come from the agent's cost computer (see computeCost), not a child. Layers with
   * extra params call this from their override; layers without the three-plugin composition
   * (e.g. BacktrackingLayer) replace it, and must then read "priority" themselves (default
   * defaultPriority()).
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Update the layer's private current-state view.
   *
   * Called once per tick before update/expand/planPath. @p active_node is planned_path[1] of the
   * step last executed, from whichever layer produced it; a layer that does not own that node
   * adopts @p state its own way. Layers that record executed edges consume @p executed_edge here.
   *
   * @param state         Latest robot state.
   * @param executed_edge The edge the agent just finished executing
   *                      (nullptr on the first tick before any edge has run).
   * @param active_node   Destination of the executed edge (nullptr if none).
   */
  virtual void onStateReached(const RobotState& state, const Edge* executed_edge, Node* active_node) = 0;

  /**
   * @brief Initialize per-episode state from the robot's starting pose.
   *
   * Called once per episode before the first onStateReached. Default is a
   * no-op; layers that need to seed an internal structure (e.g. PlanningLayer
   * rooting its graph at @p init_state) override this.
   */
  virtual void onEpisodeStart(const RobotState& /*init_state*/) {}

  /**
   * @brief Optional one-shot lazy-init pass triggered when the agent is
   *        configured with build_plugins_on_init. Default is a no-op.
   */
  virtual void warmUp() {}

  /**
   * @brief Split the active edge at @p waypoint_idx into a new intermediate node and the leading half.
   *
   * Hook for agent subclasses that recover a partially executed edge (e.g. after a collision);
   * nothing in the core calls it. The default returns false: splitting unsupported, which a caller
   * with no other recovery treats as fatal.
   *
   * @param active_node          Destination node of the active edge.
   * @param edge                 The active edge being split.
   * @param waypoint_idx         Trajectory index at which to split.
   * @param intermediate_out     Out: new intermediate node inserted at the split.
   * @param finished_edge_out    Out: leading half of the split edge.
   * @return True if the edge was split and both outputs are set.
   */
  virtual bool splitActiveEdge(Node* active_node, Edge* edge, int waypoint_idx, Node** intermediate_out, Edge** finished_edge_out);

  /**
   * @brief Per-tick priority; higher values are consulted first.
   *
   * The default returns the configured priority. State-dependent overrides must list every value
   * they can return in possiblePriorities().
   */
  virtual int getPriority(const RobotState& /*state*/) const { return p_priority_; }

  /**
   * @brief Baseline priority used when YAML does not specify "priority".
   *        Subclasses override to give themselves a sensible default (e.g.
   *        BacktrackingLayer returns a very low number so it is the fallback).
   */
  virtual int defaultPriority() const { return 0; }

  /// @brief The static priority loaded from YAML (before any state-dependent
  /// promotion in getPriority()). Used for diagnostics such as the agent's
  /// startup stack summary.
  int configuredPriority() const { return p_priority_; }

  /**
   * @brief Every priority this layer can possibly report from getPriority().
   *
   * The agent rejects overlapping sets at startup, so a layer with state-dependent priority must
   * override this.
   */
  virtual std::vector<int> possiblePriorities() const { return {configuredPriority()}; }

  /**
   * @brief Grow the layer's candidate set: sample new candidates, connect and evaluate them.
   *
   * Called every iteration after update(), from the layer's own current-node view. How many
   * candidates one call adds is the layer's choice (PlanningLayer runs max_expansions rounds).
   */
  virtual void expand() = 0;

  /**
   * @brief Re-evaluate existing candidate nodes after map updates.
   *
   * Called every iteration after onStateReached and before expand(). Default is a no-op.
   */
  virtual void update() {}

  /**
   * @brief Try to produce a planned path/edge handoff for execution.
   *
   * Returns false when the layer has nothing to offer; Agent then consults the next layer. On
   * success planned_path[0] is the layer's current node, planned_path[1] the destination and
   * planned_edges[0] the edge to execute; every edge has >= 2 states and starts at time 0.
   *
   * @param current_state Latest robot state (used by free-space planners).
   * @param map           The shared agent map.
   * @param robot         The shared robot model.
   * @param planned_path  Out: planned nodes, starting at the layer's current node.
   * @param planned_edges Out: edges between consecutive @p planned_path nodes.
   * @return True if the layer offers a step.
   */
  virtual bool planPath(const RobotState& current_state, Map* map, RobotModel* robot, std::vector<Node*>* planned_path,
                        std::vector<Edge*>* planned_edges) = 0;

  /**
   * @brief Called by Agent after this layer's planPath proposal was accepted.
   *
   * Overrides must call Layer::onSelectedAsWinner() to keep the common metrics accurate.
   */
  virtual void onSelectedAsWinner() { getLayerMetrics().times_selected_as_winner += 1; }

  /**
   * @brief Append per-layer visualization markers for the current planning state.
   *
   * Default no-op. Layers with persistent geometry (graph, KD-tree, synthetic
   * targets) override to render it.
   *
   * @param selected_node The next node selected by the agent this tick (may
   *                      be nullptr if no step was selected).
   * @param markers       Output marker container.
   */
  virtual void getVisualization(Node* selected_node, VisualizationMarkers* markers);

  /**
   * @brief Append markers describing the path this layer just offered, for a topic of its own.
   *
   * Default no-op.
   *
   * @param selected_node The next node selected by the agent this tick, or nullptr when this
   *                      layer offered no step; a layer that lost has no path to describe.
   * @param markers       Output marker container.
   */
  virtual void getPathVisualization(Node* selected_node, VisualizationMarkers* markers);

  /**
   * @brief The gain this layer has really collected this episode, for episode_summary.yaml.
   *
   * The default returns std::nullopt: the layer reports no gain.
   */
  virtual std::optional<double> realAccumulatedGain() const { return std::nullopt; }

  /**
   * @brief Human-readable one-line summary of this layer's last planning cycle.
   *
   * Default is empty; the agent prints any non-empty summary in verbose mode.
   */
  virtual std::string cycleSummary() const { return {}; }

  /**
   * @brief Reset per-episode state.
   *
   * Default resets this layer's own children; the agent's cost computer is
   * reset by Agent::clearEpisodeState. Subclasses that override must call
   * Layer::resetEpisode().
   */
  void resetEpisode() override {
    if (generator_) {
      generator_->resetEpisode();
    }
    if (evaluator_) {
      evaluator_->resetEpisode();
    }
    if (path_planner_) {
      path_planner_->resetEpisode();
    }
    if (metrics_) {
      metrics_->resetEpisode();
    }
  }

  /**
   * @brief The generator child created by Layer::setupFromParamMap().
   *
   * Precondition: hasGenerator(). A layer that replaces that setup (e.g. BacktrackingLayer) has no children, and
   * this and the two accessors below then dereference a null pointer.
   */
  Generator& getGenerator() { return *generator_; }
  /// @brief The evaluator child; precondition: hasEvaluator() (see getGenerator()).
  Evaluator& getEvaluator() { return *evaluator_; }
  /// @brief The path-planner child; precondition: hasPathPlanner() (see getGenerator()).
  PathPlanner& getPathPlanner() { return *path_planner_; }

  /// @brief Whether the layer owns a generator child.
  bool hasGenerator() const { return static_cast<bool>(generator_); }
  /// @brief Whether the layer owns an evaluator child.
  bool hasEvaluator() const { return static_cast<bool>(evaluator_); }
  /// @brief Whether the layer owns a path-planner child.
  bool hasPathPlanner() const { return static_cast<bool>(path_planner_); }

  /// @brief The agent's cost computer, shared by every layer.
  CostComputer* getCostComputer() const { return agent_.getCostComputer(); }
  /// @brief Whether the agent has a cost computer; Agent always configures one.
  bool hasCostComputer() const { return getCostComputer() != nullptr; }

  /**
   * @brief Compute the cost of @p edge using the agent's cost computer.
   * @param edge The edge whose cost is set.
   * @return The cost computer's result: true if the cost was computed.
   * @throws std::runtime_error if the agent has no cost computer.
   */
  bool computeCost(Edge* edge) {
    CostComputer* cost_computer = getCostComputer();
    if (cost_computer == nullptr) {
      throw std::runtime_error("Layer " + instanceName() + ": the agent has no cost computer to compute edge costs!");
    }
    return cost_computer->computeCost(edge);
  }

 protected:
  std::unique_ptr<Generator> generator_;
  std::unique_ptr<Evaluator> evaluator_;
  std::unique_ptr<PathPlanner> path_planner_;

  /// @brief Per-layer metrics; a plain LayerMetrics unless a subclass installs a derived type.
  std::unique_ptr<LayerMetrics> metrics_;

  /// @brief Baseline priority loaded from YAML in setupFromParamMap.
  /// Layers may override getPriority() to make it state-dependent.
  int p_priority_ = 0;

 private:
  /// @brief Runtime instance name; see instanceName(). Empty until the agent
  /// sets it when building the layer stack.
  std::string instance_name_;
};

inline bool Layer::splitActiveEdge(Node* /*active_node*/, Edge* /*edge*/, int /*waypoint_idx*/, Node** /*intermediate_out*/,
                                   Edge** /*finished_edge_out*/) {
  return false;
}

inline void Layer::getVisualization(Node* /*selected_node*/, VisualizationMarkers* /*markers*/) {}

inline void Layer::getPathVisualization(Node* /*selected_node*/, VisualizationMarkers* /*markers*/) {}

}  // namespace open_apero
