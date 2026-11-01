#include "open_apero_core/plugin/path_planner/graph_tsp_search.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_enums.pb.h>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing_parameters.h>

namespace open_apero {
namespace path_planner {

using namespace operations_research;
using NodeIndex = RoutingIndexManager::NodeIndex;

namespace {

std::vector<NodeId> getTopCandidates(GraphType* graph, double top_node_percentage, bool consider_frontier) {
  double max_gain = std::numeric_limits<double>::lowest();
  double min_gain = std::numeric_limits<double>::max();

  auto vertices = boost::vertices(*graph);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    auto gain = (*graph)[*it]->getSingleNodeGain();
    max_gain = std::max(max_gain, gain);
    min_gain = std::min(min_gain, gain);
  }

  auto gain_range = max_gain - min_gain;
  auto gain_threshold = max_gain - top_node_percentage * gain_range - 1e-4;

  std::vector<NodeId> goal_candidates;
  for (auto it = vertices.first; it != vertices.second; ++it) {
    auto node = (*graph)[*it];
    if (node->getSingleNodeGain() > gain_threshold) {
      goal_candidates.push_back(*it);
    } else if (consider_frontier && node->is_at_frontier) {
      goal_candidates.push_back(*it);
    }
  }

  return goal_candidates;
}

std::vector<NodeId> getTspPath(const TspDataModel& data, const RoutingIndexManager& manager, const RoutingModel& routing,
                               const Assignment& solution) {
  std::vector<NodeId> path;

  int64_t index = routing.Start(0);
  while (!routing.IsEnd(index)) {
    NodeId current = data.getNodeId(manager.IndexToNode(index).value());
    // Ignore invalid descriptors from unreachable candidates.
    if (current == kInvalidNodeId) {
      break;
    }

    path.push_back(current);
    index = solution.Value(routing.NextVar(index));
  }

  NodeId last = data.getNodeId(manager.IndexToNode(index).value());
  if (last != kInvalidNodeId) {
    path.push_back(last);
  }

  return path;
}

}  // namespace

void IdBimap::insertId(NodeId node_id) {
  int new_id = size();
  node_id_to_dist_id_[node_id] = new_id;
  dist_id_to_node_id_[new_id] = node_id;
}

TspDataModel::TspDataModel(const Eigen::MatrixXi& dist_matrix, const IdBimap& map, const std::vector<NodeId>& start_nodes,
                           const std::vector<NodeId>& end_nodes)
    : distance_matrix_(dist_matrix), id_bimap_(map), starts_(start_nodes), ends_(end_nodes) {}

std::vector<int> TspDataModel::getStartDistIds() const {
  std::vector<int> dist_ids;
  for (auto node_id : starts_) {
    dist_ids.push_back(getDistId(node_id));
  }
  return dist_ids;
}

std::vector<int> TspDataModel::getEndDistIds() const {
  std::vector<int> dist_ids;
  for (auto node_id : ends_) {
    dist_ids.push_back(getDistId(node_id));
  }
  return dist_ids;
}

PluginRegistry::Registration<GraphTspSearch> GraphTspSearch::registration("GraphTspSearch");

GraphTspSearch::GraphTspSearch(AgentInterface& agent) : PathPlanner(agent) {
  objective_required_ = false;
}

void GraphTspSearch::setupFromParamMap(Configurable::ParamMap* param_map) {
  PathPlanner::setupFromParamMap(param_map);

  setParam<double>(param_map, "top_node_percentage", &p_top_node_percentage_, 1.0);
  setParam<double>(param_map, "cost_scale", &p_cost_scale_, 1.0);

  rng_.seed(agent_.getRandomSeed(), (*param_map)["param_namespace"]);
}

bool GraphTspSearch::checkParamsValid(std::string* error_message) {
  if (!std::isfinite(p_cost_scale_) || p_cost_scale_ <= 0.0) {
    *error_message = "cost_scale expected finite and > 0.0";
    return false;
  }
  return PathPlanner::checkParamsValid(error_message);
}

std::pair<std::vector<Node*>, double> GraphTspSearch::planPath(Node* start_node, GraphType* graph, Node* required_terminal_node) {
  // The solver's tour is not constrained to a destination.
  if (required_terminal_node != nullptr) {
    return std::pair<std::vector<Node*>, double>({}, kDoubleMin);
  }
  if (graph == nullptr) {
    return std::pair<std::vector<Node*>, double>({start_node}, 0);
  }

  NodeId start_node_id = getNodeId(start_node, graph);

  std::vector<NodeId> best_path = {start_node_id};
  double best_value = std::numeric_limits<double>::lowest();

  Eigen::MatrixXd dist_matrix_d;
  Eigen::MatrixXi pred_matrix;
  IdBimap id_bimap;
  getAllPairsShortestPaths(graph, dist_matrix_d, pred_matrix, id_bimap);

  Eigen::MatrixXi dist_matrix;
  dist_matrix.resize(dist_matrix_d.rows(), dist_matrix_d.cols());
  for (int i = 0; i < dist_matrix_d.rows(); ++i) {
    for (int j = 0; j < dist_matrix_d.cols(); ++j) {
      // Truncating after scaling; a scaled cost beyond the int range saturates like an unreachable pair.
      const double scaled_cost = dist_matrix_d(i, j) * p_cost_scale_;
      if (std::isinf(dist_matrix_d(i, j)) || scaled_cost >= static_cast<double>(std::numeric_limits<int>::max())) {
        dist_matrix(i, j) = std::numeric_limits<int>::max();
      } else {
        dist_matrix(i, j) = static_cast<int>(scaled_cost);
      }
    }
  }

  TspDataModel full_tsp_data(dist_matrix, id_bimap, {start_node_id}, {kInvalidNodeId});

  bool consider_frontier = true;
  std::vector<NodeId> top_candidate_goals = getTopCandidates(graph, p_top_node_percentage_, consider_frontier);
  // A candidate the start cannot reach would enter the tour over a saturated arc, and the tour's path
  // reconstruction then fails: drop it.
  const int start_dist_id = full_tsp_data.getDistId(start_node_id);
  top_candidate_goals.erase(std::remove_if(top_candidate_goals.begin(), top_candidate_goals.end(),
                                           [&](NodeId goal) {
                                             return full_tsp_data.getDistance(start_dist_id, full_tsp_data.getDistId(goal)) ==
                                                    std::numeric_limits<int>::max();
                                           }),
                            top_candidate_goals.end());

  TspDataModel selected_tsp_data = getSelectedTspData(full_tsp_data, top_candidate_goals);

  std::vector<NodeIndex> start_indices;
  for (auto dist_id : selected_tsp_data.getStartDistIds()) {
    start_indices.push_back(NodeIndex(dist_id));
  }

  std::vector<NodeIndex> end_indices;
  for (auto dist_id : selected_tsp_data.getEndDistIds()) {
    end_indices.push_back(NodeIndex(dist_id));
  }

  if (start_indices.size() != 1 || end_indices.size() != 1) {
    throw std::runtime_error("GraphTspSearch: start_indices and end_indices should have size 1");
  }

  RoutingIndexManager manager(selected_tsp_data.size(), selected_tsp_data.getNumVehicles(), start_indices, end_indices);

  RoutingModel routing(manager);

  const int transit_callback_index =
      routing.RegisterTransitCallback([&selected_tsp_data, &manager](const int64_t from_index, const int64_t to_index) -> int64_t {
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return selected_tsp_data.getDistance(from_node, to_node);
      });

  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  const Assignment* solution = routing.SolveWithParameters(searchParameters);
  if (solution == nullptr) {
    if (!reported_no_solution_) {
      agent_.printWarning("GraphTspSearch: the routing solver found no tour; falling back to a neighbor.");
      reported_no_solution_ = true;
    }
    best_path = {start_node_id};
  } else {
    std::vector<NodeId> tsp_path = getTspPath(selected_tsp_data, manager, routing, *solution);

    best_path = {start_node_id};
    for (std::size_t i = 1; i < tsp_path.size(); ++i) {
      // Every stop is reachable from the start, but two dead ends are not from each other: the tour ends at the
      // first leg the graph cannot route.
      if (pred_matrix(id_bimap.getDistId(tsp_path[i - 1]), id_bimap.getDistId(tsp_path[i])) == -1) {
        break;
      }
      std::vector<NodeId> path = getShortestPath(tsp_path[i - 1], tsp_path[i], pred_matrix, id_bimap);
      best_path.insert(best_path.end(), path.begin(), path.end());
    }
    best_value = 0.0;

    for (std::size_t i = 1; i < best_path.size(); ++i) {
      if (best_path[i] == best_path[i - 1]) {
        best_path.erase(best_path.begin() + i);
        --i;
      }
    }
  }

  // A tour with no node besides the start, or no tour: fall back to a random neighbor.
  if (best_path.size() == 1) {
    boost::graph_traits<GraphType>::adjacency_iterator neighborIt, neighborEnd;
    std::tie(neighborIt, neighborEnd) = boost::adjacent_vertices(start_node_id, *graph);
    if (neighborIt != neighborEnd) {
      std::mt19937& gen = rng_.engine;
      std::uniform_int_distribution<> dis(0, std::distance(neighborIt, neighborEnd) - 1);
      std::advance(neighborIt, dis(gen));
      best_path.emplace_back(*neighborIt);
      best_value = 0.0;
    }
  }

  std::vector<Node*> best_path_nodes;
  for (const auto& node_id : best_path) {
    best_path_nodes.emplace_back((*graph)[node_id].get());
  }

  return std::pair<std::vector<Node*>, double>(best_path_nodes, best_value);
}

TspDataModel GraphTspSearch::getSelectedTspData(const TspDataModel& full_tsp_data, const std::vector<NodeId>& selected_node_ids) {
  std::unordered_set<NodeId> selected_node_ids_set(selected_node_ids.begin(), selected_node_ids.end());

  for (const auto& start_node_id : full_tsp_data.getStartNodeIds()) {
    selected_node_ids_set.insert(start_node_id);
  }

  for (const auto& end_node_id : full_tsp_data.getEndNodeIds()) {
    selected_node_ids_set.insert(end_node_id);
  }

  std::vector<int> selected_ids_in_full_data;
  selected_ids_in_full_data.reserve(selected_node_ids.size());
  for (int i = 0; i < full_tsp_data.size(); ++i) {
    NodeId node_id = full_tsp_data.getNodeId(i);
    if (selected_node_ids_set.count(node_id)) {
      selected_ids_in_full_data.push_back(i);
    }
  }

  Eigen::MatrixXi distance_matrix;
  distance_matrix.resize(selected_ids_in_full_data.size(), selected_ids_in_full_data.size());
  for (std::size_t i = 0; i < selected_ids_in_full_data.size(); ++i) {
    for (std::size_t j = 0; j < selected_ids_in_full_data.size(); ++j) {
      distance_matrix(i, j) = full_tsp_data.getDistance(selected_ids_in_full_data[i], selected_ids_in_full_data[j]);
    }
  }

  IdBimap id_bimap;
  for (std::size_t i = 0; i < selected_ids_in_full_data.size(); ++i) {
    int id_in_full_data = selected_ids_in_full_data[i];
    id_bimap.insertId(full_tsp_data.getNodeId(id_in_full_data));
  }

  TspDataModel selected_tsp_data(distance_matrix, id_bimap, full_tsp_data.getStartNodeIds(), full_tsp_data.getEndNodeIds());
  return selected_tsp_data;
}

void GraphTspSearch::getAllPairsShortestPaths(const GraphType* graph, Eigen::MatrixXd& dist, Eigen::MatrixXi& pred, IdBimap& id_bimap) {
  const double inf = std::numeric_limits<double>::infinity();

  id_bimap.insertId(kInvalidNodeId);
  for (auto v : boost::make_iterator_range(boost::vertices(*graph))) {
    id_bimap.insertId(v);
  }

  auto num_vertices = id_bimap.size();

  dist = Eigen::MatrixXd::Constant(num_vertices, num_vertices, inf);
  pred = Eigen::MatrixXi::Constant(num_vertices, num_vertices, -1);  // -1 means no predecessor

  for (int i = 0; i < num_vertices; ++i) {
    dist(i, i) = 0.0;
  }

  auto edges = boost::edges(*graph);
  for (auto it = edges.first; it != edges.second; ++it) {
    auto u = boost::source(*it, *graph);
    auto v = boost::target(*it, *graph);
    auto edge = (*graph)[*it];
    if (edge && edge->cost != inf) {
      int u_idx = id_bimap.getDistId(u);
      int v_idx = id_bimap.getDistId(v);

      dist(u_idx, v_idx) = edge->cost;
      pred(u_idx, v_idx) = u_idx;  // Predecessor of v is u
    }
  }

  // The artificial sink is free to enter and impossible to leave.
  for (int i = 1; i < num_vertices; ++i) {
    dist(i, 0) = 0.0;
    pred(i, 0) = i;
  }

  for (int k = 0; k < num_vertices; ++k) {
    for (int i = 0; i < num_vertices; ++i) {
      for (int j = 0; j < num_vertices; ++j) {
        if (dist(i, k) != inf && dist(k, j) != inf) {
          double dist_i_k_j = dist(i, k) + dist(k, j);
          if (dist_i_k_j < dist(i, j)) {
            dist(i, j) = dist_i_k_j;
            pred(i, j) = pred(k, j);
          }
        }
      }
    }
  }
}

std::vector<NodeId> GraphTspSearch::getShortestPath(NodeId start_node_id, NodeId goal_node_id, const Eigen::MatrixXi& pred,
                                                    IdBimap& id_bimap) {
  int start_idx = id_bimap.getDistId(start_node_id);
  int goal_idx = id_bimap.getDistId(goal_node_id);

  if (pred(start_idx, goal_idx) == -1) {
    throw std::runtime_error("GraphTspSearch: no path between start_node_id and goal_node_id");
  }

  std::vector<NodeId> path;
  int current_idx = goal_idx;
  while (current_idx != start_idx) {
    path.push_back(id_bimap.getNodeId(current_idx));
    current_idx = pred(start_idx, current_idx);

    if (current_idx == -1) {
      throw std::runtime_error("GraphTspSearch: invalid predecessor");
    }
  }
  path.push_back(start_node_id);

  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace path_planner
}  // namespace open_apero
