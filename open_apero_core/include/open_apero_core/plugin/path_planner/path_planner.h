#pragma once

/**
 * @file path_planner.h
 * @brief Path-planning plugin interface (graph search) and objectives.
 */

#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "open_apero_core/gain/gain_cost_table.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/path.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"

namespace open_apero {

// Layer includes this plugin interface, so including layer.h here would create
// an include cycle.
class Layer;

/** @brief Optimization objective for next-best-path selection; C is the agent's remaining cost budget, as in the paper. */
enum class Objective {
  kPathGain,      ///< Score: gain.
  kPathRatio,     ///< Score: gain / cost.
  kExpectedGain,  ///< Score: ratio * C when the path ends at a frontier node, gain otherwise.
  kTableGain      ///< Score: gain + GainCostTable::valueAt(C - cost) at a frontier end, gain otherwise.
};

/**
 * @brief Convert an Objective enum value to its string representation.
 * @param type The objective to convert.
 * @return String name of the objective.
 * @throws std::invalid_argument if @p type is none of the four enumerators.
 */
inline std::string toString(Objective type) {
  switch (type) {
    case Objective::kPathGain:
      return "path_gain";
    case Objective::kPathRatio:
      return "path_ratio";
    case Objective::kExpectedGain:
      return "expected_gain";
    case Objective::kTableGain:
      return "table_gain";
  }
  throw std::invalid_argument("PathPlanner: unknown Objective value");
}

/**
 * @brief Parse the "objective" key into an Objective enum value.
 * @param str String name of the objective.
 * @return The corresponding Objective value.
 * @throws std::runtime_error if @p str names none of the four objectives.
 */
inline Objective objectiveFromString(const std::string& str) {
  if (str == "path_gain") return Objective::kPathGain;
  if (str == "path_ratio") return Objective::kPathRatio;
  if (str == "expected_gain") return Objective::kExpectedGain;
  if (str == "table_gain") return Objective::kTableGain;
  throw std::runtime_error("PathPlanner: unsupported objective \"" + str +
                           "\" (expected path_gain, path_ratio, expected_gain or table_gain)");
}

/**
 * @brief Abstract base class that plans the next best path over the planning graph.
 *
 * Selects among candidate paths from the start node by the configured Objective. The cost
 * budget is the agent's (AgentInterface::getCurrentCostBudget).
 */
class PathPlanner : public Plugin {
 public:
  /**
   * @brief Construct a PathPlanner associated with the given agent.
   * @param agent Reference to the agent interface.
   */
  explicit PathPlanner(AgentInterface& agent) : Plugin(agent), gain_cost_table_(p_concave_envelope_) {}

  /** @brief Sets the owning layer; called by the owning layer after setup. getLayer() is nullptr during setupFromParamMap. */
  void setLayer(Layer* layer) { layer_ = layer; }

  /** @brief Owning layer, or nullptr if not yet wired. Used to reach sibling plugins. */
  Layer* getLayer() const { return layer_; }

  void resetEpisode() override { gain_cost_table_.reset(); }

  /**
   * @brief Set up parameters from the parameter map.
   *
   * "objective" is required unless objective_required_ is false, in which case it defaults to
   * the current p_objective_.
   *
   * @param param_map Pointer to the plugin parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Validates gain_suppression_radius: finite and >= 0. */
  bool checkParamsValid(std::string* error_message) override;

  /**
   * @brief Select the next best path from the given start node.
   *
   * Answers "where should I go next": any prefix the objective likes is a legal answer.
   * @param start_node The node to start path selection from.
   * @param graph Pointer to the planning graph; nullptr when the layer's generator is a tree (TreeGenerator).
   * @return A pair containing the ordered sequence of nodes forming the best path and its value.
   */
  std::pair<std::vector<Node*>, double> planPath(Node* start_node, GraphType* graph) { return planPath(start_node, graph, nullptr); }

  /**
   * @brief Select the best path from @p start_node that ends at @p required_terminal_node.
   *
   * Must return an EMPTY path, never a fallback step, when the destination is unreachable;
   * finishing through concludeSearch() guarantees this. The path traverses at least one edge,
   * so @p start_node is never a valid destination.
   *
   * @param start_node The node to start path selection from.
   * @param graph Pointer to the planning graph; nullptr when the layer's generator is a tree (TreeGenerator).
   * @param required_terminal_node Node the selected path must end at, or nullptr to search freely.
   * @return The best path and its value, or an empty path when the destination is unreachable.
   */
  virtual std::pair<std::vector<Node*>, double> planPath(Node* start_node, GraphType* graph, Node* required_terminal_node) = 0;

 protected:
  /**
   * @brief Fold one candidate into the running best by its Objective score.
   *
   * Exact ties prefer a frontier end under kExpectedGain/kTableGain, else Path::operator>.
   * kTableGain inserts the candidate into the episodic table before scoring, so the result is
   * order-dependent. Candidates not ending at @p required_terminal_node are discarded unscored.
   *
   * @param candidate The candidate path to evaluate.
   * @param best Running best path, initially invalid; replaced by @p candidate when it scores higher.
   * @param best_score Running best score, initially kDoubleMin; updated alongside @p best.
   * @param required_terminal_node Node the candidate must end at, or nullptr to score them all.
   * @return Whether @p candidate became the best.
   */
  bool updateBest(Path candidate, Path& best, double& best_score, Node* required_terminal_node);

  /**
   * @brief Decide Node::is_gain_suppressed for a search from @p start_node (suppressNonMaximumClusters).
   *
   * Called at the start of every search, so the flags follow the gains the evaluator holds now.
   * @param nodes Every node the search may reach.
   * @param start_node The search's start node.
   */
  void suppressGains(const std::vector<Node*>& nodes, const Node* start_node) const;

  /**
   * @brief suppressGains() over every vertex of @p graph.
   * @param graph The graph the search runs on.
   * @param start_node The search's start node.
   */
  void suppressGains(const GraphType& graph, const Node* start_node) const;

  /**
   * @brief Turns the running best into the answer.
   *
   * With nothing selected, an unconstrained search returns a random step to one of
   * @p fallback_candidates and a constrained one returns the empty path.
   *
   * @param best The running best, as folded by updateBest(); invalid when nothing was selected.
   * @param start_node First node of a fallback step.
   * @param required_terminal_node The destination this search was asked for, or nullptr.
   * @param fallback_candidates Nodes reachable in one step, in a deterministic order.
   * @param rng Engine for the fallback pick; drawn from only when a fallback actually happens.
   * @param score Set to the chosen node's own gain when a fallback is taken; untouched otherwise.
   * @return The selected path, a one-step fallback, or an empty path.
   * @throws std::runtime_error when a fallback is needed but @p fallback_candidates is empty.
   */
  std::vector<Node*> concludeSearch(const Path& best, Node* start_node, Node* required_terminal_node,
                                    const std::vector<Node*>& fallback_candidates, std::mt19937* rng, double* score) const;

 protected:
  /** @brief Optimization objective used for path selection. */
  Objective p_objective_ = Objective::kPathGain;

  /** @brief Whether setupFromParamMap() requires the "objective" key; set before setup. */
  bool objective_required_ = true;

  /** @brief Whether nodes share gain anchors, so a path counts each anchor once (key is_gain_correlated). */
  bool p_is_gain_correlated_ = false;

  /** @brief Radius [m] of the non-maximum suppression over node clusters before each search; 0 suppresses nothing. */
  double p_gain_suppression_radius_ = 0.0;

  /** @brief Whether the gain-cost table uses the optional concave envelope (kTableGain only). */
  bool p_concave_envelope_ = true;

  /** @brief Episodic optimistic gain-vs-cost frontier, used by the kTableGain objective. */
  GainCostTable gain_cost_table_;

  Layer* layer_ = nullptr;
};

}  // namespace open_apero
