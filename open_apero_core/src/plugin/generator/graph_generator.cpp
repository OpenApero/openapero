#include "open_apero_core/plugin/generator/graph_generator.h"

#include <stdexcept>
#include <string>
#include <vector>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/layer/layer.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/visualization/visualization_helpers.h"

namespace open_apero {

GraphGenerator::GraphGenerator(AgentInterface& agent) : Generator(agent) {}

void GraphGenerator::setupFromParamMap(Configurable::ParamMap* param_map) {
  Generator::setupFromParamMap(param_map);

  setParam<double>(param_map, "min_edge_length", &p_min_edge_length_, 0.8);
  setParam<double>(param_map, "max_edge_length", &p_max_edge_length_, 100.0);
  setParam<bool>(param_map, "visualize_edges", &p_visualize_edges_, false);
}

bool GraphGenerator::checkParamsValid(std::string* error_message) {
  if (!(p_min_edge_length_ > 0.0)) {
    *error_message = "min_edge_length expected > 0.0";
    return false;
  }
  if (p_max_edge_length_ != 0.0 && !(p_max_edge_length_ >= p_min_edge_length_)) {
    *error_message = "max_edge_length expected 0.0 (no limit) or >= min_edge_length";
    return false;
  }
  return Generator::checkParamsValid(error_message);
}

bool GraphGenerator::computeCost(Edge* edge) {
  return getLayer()->computeCost(edge);
}

bool GraphGenerator::generateEdgeBetweenNodes(Node* start_node, Node* goal_node, Edge* edge) {
  if (!edge->trajectory.empty()) {
    throw std::runtime_error("GraphGenerator::generateEdgeBetweenNodes: the output edge already has a trajectory");
  }

  // Precondition: distinct positions; coincident pairs go to generateRotationEdgeBetweenNodes.
  if (isSamePosition(start_node->robot_state.getBasePosition(), goal_node->robot_state.getBasePosition())) {
    throw std::runtime_error(
        "GraphGenerator::generateEdgeBetweenNodes called with coincident start/goal base positions; this is the translation "
        "builder -- callers must dispatch in-place rotations to generateRotationEdgeBetweenNodes.");
  }

  Edge new_edge;
  if (!robot_->planTrajectory(start_node->robot_state, goal_node->robot_state, &agent_.getMap(), &new_edge.trajectory)) {
    return false;
  }

  new_edge.is_traversable = true;
  computeCost(&new_edge);
  *edge = new_edge;
  return true;
}

bool GraphGenerator::generateRotationEdgeBetweenNodes(Node* start_node, Node* goal_node, Edge* edge) {
  if (!edge->trajectory.empty()) {
    throw std::runtime_error("GraphGenerator::generateRotationEdgeBetweenNodes: the output edge already has a trajectory");
  }

  Edge new_edge;
  if (!robot_->planRotationTrajectory(start_node->robot_state, goal_node->robot_state, &agent_.getMap(), &new_edge.trajectory)) {
    return false;
  }

  new_edge.is_traversable = true;
  computeCost(&new_edge);
  *edge = new_edge;
  return true;
}

bool GraphGenerator::buildRetargetedEdge(const Edge& seed_edge, Node* start_node, Node* goal_node, Edge* edge) {
  if (!edge->trajectory.empty()) {
    throw std::runtime_error("GraphGenerator::buildRetargetedEdge: the output edge already has a trajectory");
  }

  Edge new_edge;
  new_edge.trajectory = seed_edge.trajectory;
  if (!robot_->retargetTrajectoryEndpoints(&new_edge.trajectory, start_node->robot_state, goal_node->robot_state)) {
    return false;
  }

  new_edge.is_traversable = true;
  computeCost(&new_edge);
  *edge = new_edge;
  return true;
}

void GraphGenerator::revalidateEdge(Edge* edge) {
  // Backtrack edges are exempt, as in the pre-flight check (see Edge::is_backtrack).
  if (edge->is_backtrack) {
    return;
  }

  // The trajectory is NOT mutated: the executor follows edge->trajectory by pointer.
  edge->is_traversable = robot_->isTrajectoryTraversable(edge->trajectory, &agent_.getMap());
}

// Generates visualization markers for all nodes and (optionally) edges in the planning graph.
// Overrides the base Generator viz to include intermediate (non-KD-tree) nodes that
// live in the graph, and to optionally render edges.
void GraphGenerator::getVisualization(Node* start_node, VisualizationMarkers* markers) {
  auto all_nodes = getAllNodes(start_node);
  getNodeVisualizationMarkers(markers, all_nodes, true);

  if (p_visualize_edges_) {
    auto all_edges = getAllEdges(start_node);
    getEdgeVisualizationMarkers(markers, all_edges, isEdgeBidirectional());
  }
}

}  // namespace open_apero
