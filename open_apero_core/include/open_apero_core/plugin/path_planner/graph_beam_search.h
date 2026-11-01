#pragma once

/**
 * @file graph_beam_search.h
 * @brief What depth-wise and node-wise beam search share: parameters, the previous path and the frame of planPath().
 */

#include <string>
#include <utility>
#include <vector>

#include "open_apero_core/common/seeding.h"
#include "open_apero_core/graph/path.h"
#include "open_apero_core/plugin/path_planner/helpers.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"

namespace open_apero {
namespace path_planner {

/**
 * @brief Beam search over a graph, up to beam_depth edges; a subclass decides which paths a round keeps.
 *
 * planPath() suppresses gains, offers the still-reachable tail of the previous path as candidates
 * (check_previous_path), runs the subclass's beam rounds (searchBeams()) and concludes the search.
 */
class GraphBeamSearch : public PathPlanner {
 public:
  /**
   * @brief Select the next best path from the graph by beam search.
   * @param start_node The node to start the search from.
   * @param graph The graph to search over; a null graph has no path to any destination.
   * @param required_terminal_node Node the selected path must end at, or nullptr to search freely.
   * @return A pair of the best path (as a vector of nodes) and its value.
   */
  std::pair<std::vector<Node*>, double> planPath(Node* start_node, GraphType* graph, Node* required_terminal_node) override;

  /**
   * @brief Resets path planner state for a new episode.
   *
   * Clears the cached previous path, resets the gain-cost table (PathPlanner::resetEpisode())
   * and reseeds the RNG.
   */
  void resetEpisode() override {
    PathPlanner::resetEpisode();
    previous_path_.clear();
    rng_.reseed(agent_.getRandomSeed());
  }

  /**
   * @brief Initialize parameters from the parameter map.
   * @param param_map Pointer to the plugin parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Validates beam_width >= 1 and beam_depth >= 1, then the PathPlanner checks. */
  bool checkParamsValid(std::string* error_message) override;

  /**
   * @brief Directly set search parameters for offline benchmarking.
   * @param obj The optimization objective.
   * @param beam_depth Maximum search depth.
   * @param beam_width Number of candidates retained per level (depth-wise) or per graph node (node-wise).
   * @param check_previous_path Whether to check if the previous path is still valid.
   * @param is_gain_correlated Whether nodes share gain anchors, so a path counts each anchor once.
   * @param gain_suppression_radius Radius [m] of the suppression over node clusters (suppressNonMaximumClusters); 0 suppresses nothing.
   */
  void setupRaw(Objective obj, int beam_depth, int beam_width, bool check_previous_path, bool is_gain_correlated = false,
                double gain_suppression_radius = 0.0);

 protected:
  /**
   * @brief Construct a beam search.
   * @param agent Reference to the parent agent interface.
   * @param raw_seed_label Seeding label of setupRaw(), one per search.
   */
  GraphBeamSearch(AgentInterface& agent,
                  std::string raw_seed_label);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /**
   * @brief The beam rounds: extends paths from @p start_path for up to beam_depth edges within the cost
   *        budget and offers every extension to updateBest().
   * @param start_path The path holding only the start node.
   * @param graph The graph to search over.
   * @param required_terminal_node Node the selected path must end at, or nullptr.
   * @param steps The search's path steps.
   * @param ledger The search's path ledger.
   * @param best The running best, as folded by updateBest().
   * @param best_score The running best's score.
   */
  virtual void searchBeams(const Path& start_path, GraphType* graph, Node* required_terminal_node, PathSteps& steps, PathLedger& ledger,
                           Path& best, double& best_score) = 0;

  // Parameters
  int p_beam_depth_ = 10;
  int p_beam_width_ = 1;
  bool p_check_previous_path_ = true;

  // The best path from last time
  PathNodes previous_path_;

  // Reproducible RNG for the tie-break random neighbor pick.
  SeededRng rng_;

 private:
  const std::string raw_seed_label_;
};

}  // namespace path_planner
}  // namespace open_apero
