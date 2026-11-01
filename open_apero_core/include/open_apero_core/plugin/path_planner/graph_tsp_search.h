#pragma once

/**
 * @file graph_tsp_search.h
 * @brief Path planner that formulates path selection as a Traveling Salesman Problem (TSP).
 */

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/common/seeding.h"
#include "open_apero_core/plugin/path_planner/helpers.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"

namespace open_apero {
namespace path_planner {

/**
 * @brief Bidirectional map between NodeId and dense integer IDs used in the distance matrix.
 *
 * Provides O(1) lookup in both directions between graph node identifiers and
 * the contiguous integer indices used by the TSP solver's distance matrix.
 */
class IdBimap {
 public:
  /**
   * @brief Insert a node ID and assign it the next available dense integer ID.
   * @param node_id The graph node ID to insert.
   */
  void insertId(NodeId node_id);

  /** @brief Return the number of entries in the bimap. */
  inline int size() const { return node_id_to_dist_id_.size(); }

  /** @brief Get the dense integer ID for a given node ID. */
  inline int getDistId(NodeId node_id) const { return node_id_to_dist_id_.at(node_id); }
  /** @brief Get the node ID for a given dense integer ID. */
  inline NodeId getNodeId(int new_id) const { return dist_id_to_node_id_.at(new_id); }

 private:
  std::unordered_map<NodeId, int> node_id_to_dist_id_;
  std::unordered_map<int, NodeId> dist_id_to_node_id_;
};

/**
 * @brief Data model encapsulating the distance matrix and node mappings for the TSP solver.
 *
 * Stores the pairwise distance matrix between nodes, the ID bimap for translating
 * between node IDs and matrix indices, and the designated start and end nodes for
 * the TSP route.
 */
class TspDataModel {
 public:
  /**
   * @brief Construct a TspDataModel.
   * @param dist_matrix Pairwise integer distance matrix.
   * @param map Bidirectional ID map between node IDs and matrix indices.
   * @param start_nodes Designated start node IDs for the route.
   * @param end_nodes Designated end node IDs for the route.
   */
  TspDataModel(const Eigen::MatrixXi& dist_matrix, const IdBimap& map, const std::vector<NodeId>& start_nodes,
               const std::vector<NodeId>& end_nodes);

  /** @brief Return the number of nodes in the model. */
  inline int size() const { return id_bimap_.size(); }
  /** @brief Return the number of vehicles (always 1 for single-agent TSP). */
  inline int getNumVehicles() const { return 1; }

  /** @brief Get the dense integer ID for a given node ID. */
  inline int getDistId(NodeId node_id) const { return id_bimap_.getDistId(node_id); }
  /** @brief Get the node ID for a given dense integer ID. */
  inline NodeId getNodeId(int dist_id) const { return id_bimap_.getNodeId(dist_id); }

  /** @brief Get the start node IDs. */
  inline const std::vector<NodeId>& getStartNodeIds() const { return starts_; }
  /** @brief Get the end node IDs. */
  inline const std::vector<NodeId>& getEndNodeIds() const { return ends_; }

  /** @brief Get the distance between two nodes by their dense integer IDs. */
  inline int getDistance(int dist_id_source, int dist_id_target) const { return distance_matrix_(dist_id_source, dist_id_target); }

  /** @brief Get the dense integer IDs of start nodes. */
  std::vector<int> getStartDistIds() const;
  /** @brief Get the dense integer IDs of end nodes. */
  std::vector<int> getEndDistIds() const;

 private:
  Eigen::MatrixXi distance_matrix_;
  IdBimap id_bimap_;

  std::vector<NodeId> starts_;
  std::vector<NodeId> ends_;
};

/**
 * @brief Selects the next path by solving a Traveling Salesman Problem over the graph.
 *
 * The tour minimizes the summed edge cost through the selected nodes (the top top_node_percentage
 * of the gain range plus frontier nodes). objective is read if present but unused, and the cost
 * budget is ignored here; the agent enforces it during execution. Costs are rounded down to
 * integers after scaling by cost_scale (1000 gives millisecond resolution for time costs).
 */
class GraphTspSearch : public PathPlanner {
 public:
  /**
   * @brief Construct a GraphTspSearch path planner.
   * @param agent Reference to the parent agent interface.
   */
  explicit GraphTspSearch(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /**
   * @brief Select the next best path by solving a TSP over the graph.
   *
   * The tour comes from the routing solver and is not constrained to a destination.
   * @param start_node The node to start the search from.
   * @param graph The graph to search over.
   * @param required_terminal_node Must be nullptr; any destination yields an empty path.
   * @return A pair of the best path (as a vector of nodes) and its value.
   */
  std::pair<std::vector<Node*>, double> planPath(Node* start_node, GraphType* graph, Node* required_terminal_node) override;

  /**
   * @brief Initialize parameters from the parameter map.
   * @param param_map Pointer to the plugin parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Validate that cost_scale is finite and positive.
   * @param error_message Receives the reason on failure.
   * @return True if the parameters are valid.
   */
  bool checkParamsValid(std::string* error_message) override;

  /**
   * @brief Directly set search parameters for offline benchmarking.
   * @param obj The optimization objective.
   * @param top_node_percentage Fraction of the gain range, below the maximum gain, whose nodes the tour visits.
   */
  inline void setupRaw(Objective obj, double top_node_percentage) {
    p_objective_ = obj;
    p_top_node_percentage_ = top_node_percentage;
    rng_.seed(agent_.getRandomSeed(), "graph_tsp_search_raw");
  }

  /** @brief Reseed the tie-break RNG for a new episode. */
  void resetEpisode() override {
    PathPlanner::resetEpisode();
    rng_.reseed(agent_.getRandomSeed());
  }

 protected:
  static PluginRegistry::Registration<GraphTspSearch> registration;

  /**
   * @brief Extract a subset TSP data model containing only the selected nodes.
   * @param full_tsp_data The full TSP data model.
   * @param selected_node_ids Node IDs to include in the subset.
   * @return A reduced TspDataModel for the selected nodes.
   */
  TspDataModel getSelectedTspData(const TspDataModel& full_tsp_data, const std::vector<NodeId>& selected_node_ids);

  /**
   * @brief Reconstruct the shortest path between two nodes using the predecessor matrix.
   * @param start_node_id Start node ID.
   * @param goal_node_id Goal node ID.
   * @param pred Predecessor matrix from all-pairs shortest paths.
   * @param bi_map Bidirectional ID map.
   * @return Ordered list of node IDs forming the shortest path.
   */
  std::vector<NodeId> getShortestPath(NodeId start_node_id, NodeId goal_node_id, const Eigen::MatrixXi& pred, IdBimap& bi_map);

  /**
   * @brief Compute all-pairs shortest paths in the graph using Floyd-Warshall.
   * @param graph The graph to compute paths on.
   * @param dist Output distance matrix.
   * @param pred Output predecessor matrix for path reconstruction.
   * @param bi_map Output bidirectional ID map.
   */
  void getAllPairsShortestPaths(const GraphType* graph, Eigen::MatrixXd& dist, Eigen::MatrixXi& pred, IdBimap& bi_map);

  double p_top_node_percentage_ = 1.0;

  /** @brief Factor applied to edge costs before they are truncated to the solver's integers. */
  double p_cost_scale_ = 1.0;

  /** @brief Whether a failed solve has been reported; it is reported once per instance. */
  bool reported_no_solution_ = false;

  // Reproducible RNG for the tie-break random neighbor pick.
  SeededRng rng_;
};

}  // namespace path_planner
}  // namespace open_apero
