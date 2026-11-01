// RRAG samples positions in an annulus around existing clusters. Each accepted
// position produces an orientation cluster: robot-defined intra-cluster edges
// represent rotation, while yaw-aligned inter-cluster edges represent travel.
// A KD-tree indexes cluster positions, and intermediate nodes support replanning
// from partway through a trajectory.

#include "open_apero_core/plugin/generator/rrag.h"

#include <algorithm>
#include <cstddef>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace generator {

PluginRegistry::Registration<RRAG> RRAG::registration("RRAG");

RRAG::RRAG(AgentInterface& agent) : GraphGenerator(agent) {}

void RRAG::setupFromParamMap(Configurable::ParamMap* param_map) {
  GraphGenerator::setupFromParamMap(param_map);

  setParam<int>(param_map, "sample_max_tries", &p_sample_max_tries_, 1000);

  std::string translation_adjacency_name;
  setParam<std::string>(param_map, "translation_adjacency", &translation_adjacency_name, "straight_line");
  if (translation_adjacency_name == "straight_line") {
    p_translation_adjacency_ = TranslationAdjacency::kStraightLine;
  } else if (translation_adjacency_name == "path_heading") {
    p_translation_adjacency_ = TranslationAdjacency::kPathHeading;
  } else if (translation_adjacency_name == "path_heading_cell") {
    p_translation_adjacency_ = TranslationAdjacency::kPathHeadingCell;
  } else {
    throw std::runtime_error("RRAG: unsupported translation_adjacency \"" + translation_adjacency_name +
                             "\" (expected \"straight_line\", \"path_heading\" or \"path_heading_cell\")");
  }
}

bool RRAG::checkParamsValid(std::string* error_message) {
  if (p_sample_max_tries_ < 0) {
    *error_message = "sample_max_tries expected >= 0 (0 = unlimited)";
    return false;
  }
  return GraphGenerator::checkParamsValid(error_message);
}

std::vector<Node*> RRAG::getAllNodes(Node* /*start_node*/) const {
  std::vector<Node*> all_nodes;
  all_nodes.reserve(boost::num_vertices(*graph_));
  for (const auto& v : boost::make_iterator_range(boost::vertices(*graph_))) {
    all_nodes.push_back((*graph_)[v].get());
  }
  return all_nodes;
}

std::vector<Edge*> RRAG::getAllEdges(Node* /*start_node*/) const {
  std::vector<Edge*> all_edges;
  auto edges = boost::edges(*graph_);
  for (auto it = edges.first; it != edges.second; ++it) {
    all_edges.push_back((*graph_)[*it].get());
  }
  return all_edges;
}

std::pair<GraphType*, Node*> RRAG::generateInitNode(const RobotState& init_state) {
  graph_ = std::make_unique<GraphType>();

  auto new_start_node = std::make_shared<GraphNode>();
  new_start_node->robot_state = init_state;

  // The initial node is always safe — mark every orientation as traversable.
  robot_->sampleOrientations(init_state.getSensorPosition(), &new_start_node->candidate_orientations);
  new_start_node->is_candidate_traversable.assign(new_start_node->candidate_orientations.size(), true);

  new_start_node->is_traversable = true;
  new_start_node->connect_with_start_node = true;

  // The temporary initial node is removed after the robot advances.
  new_start_node->is_intermediate = true;

  boost::add_vertex(new_start_node, *graph_);

  kd_tree_data_.clear();
  kdtree_ = std::unique_ptr<KdTree>(new KdTree(3, kd_tree_data_));

  return {graph_.get(), new_start_node.get()};
}

void RRAG::setNewStartNode(Node* previous_start, Node* new_start) {
  if (previous_start == new_start) {
    throw std::runtime_error("RRAG::setNewStartNode called with same previous and new start node");
  }

  new_start->as<GraphNode>()->connect_with_start_node = true;

  // Remove temporary neighbors left by earlier mid-edge replanning. This is the single
  // place a previous intermediate start node is superseded, so it is removed here even when no
  // edge ever joined it to the new start node.
  auto new_start_id = getNodeId(new_start, graph_.get());
  std::unordered_set<NodeId> intermediate_ids_to_remove;
  if (previous_start->is_intermediate) {
    const NodeId previous_start_id = getNodeId(previous_start, graph_.get());
    if (previous_start_id != kInvalidNodeId) {
      intermediate_ids_to_remove.insert(previous_start_id);
    }
  }
  for (const auto& v : boost::make_iterator_range(boost::inv_adjacent_vertices(new_start_id, *graph_))) {
    if ((*graph_)[v]->is_intermediate) {
      intermediate_ids_to_remove.insert(v);
    }
  }
  for (const auto& v : boost::make_iterator_range(boost::adjacent_vertices(new_start_id, *graph_))) {
    if ((*graph_)[v]->is_intermediate) {
      intermediate_ids_to_remove.insert(v);
    }
  }
  for (const auto& node_id : intermediate_ids_to_remove) {
    boost::clear_vertex(node_id, *graph_);
    boost::remove_vertex(node_id, *graph_);
  }

  if (!intermediate_ids_to_remove.empty()) {
    rebuildKdTree();
  }
}

bool RRAG::findNearestExpansionAnchor(const Eigen::Vector3d& point, NodeClusterPtrs& nearest_node_cluster, double& dist_squared) {
  return findNearestNode(point, nearest_node_cluster, dist_squared);
}

bool RRAG::isSamplePositionAdmissible(const Eigen::Vector3d& /*position*/) const {
  return true;
}

void RRAG::indexNodeCluster(const NodeClusterPtrs& cluster) {
  addToKdTree(cluster);
}

NodeClusterPtrs RRAG::generateNewNode(Node* start_node) {
  if (!graph_) {
    throw std::runtime_error("RRAG::generateNewNode called before generateInitNode created the graph");
  }

  if (start_node == nullptr) {
    throw std::runtime_error("Start node is nullptr");
  }

  int counter = 0;
  while (p_sample_max_tries_ == 0 || counter < p_sample_max_tries_) {
    if (p_sample_max_tries_ > 0) {
      counter++;
    }

    Eigen::Vector3d goal_pos;
    // Spacing requires an existing anchor, so seed an empty graph at the robot pose.
    // That seed is not a free choice of position, so it is exempt from the stationing
    // rule -- and being deterministic, retrying it cannot change the outcome.
    const bool is_bootstrap_seed = kd_tree_data_.size() == 0;
    if (is_bootstrap_seed) {
      goal_pos = start_node->robot_state.getSensorPosition();
    } else {
      if (!robot_->samplePosition(&goal_pos, agent_.getInitialSensorPosition().z())) {
        return {};
      }

      NodeClusterPtrs nearest_node_cluster;
      double out_dist_sqr = 0.0;
      if (!findNearestExpansionAnchor(goal_pos, nearest_node_cluster, out_dist_sqr)) {
        return {};
      }
      const Eigen::Vector3d& v_nearest_pos = nearest_node_cluster.front()->robot_state.getSensorPosition();

      if (out_dist_sqr < p_min_edge_length_ * p_min_edge_length_) {
        continue;
      }

      if (p_max_edge_length_ > 0.0 && out_dist_sqr > p_max_edge_length_ * p_max_edge_length_) {
        goal_pos = v_nearest_pos + (goal_pos - v_nearest_pos).normalized() * p_min_edge_length_;
      }

      if (!isSamplePositionAdmissible(goal_pos)) {
        continue;
      }
    }

    NodeClusterPtrs new_node_cluster =
        buildClusterAt(goal_pos, is_bootstrap_seed ? StationingCheck::kExemptBootstrap : StationingCheck::kEnforce);
    if (new_node_cluster.empty()) {
      // The seed position is fixed, so every remaining try would test it again.
      if (is_bootstrap_seed) {
        return {};
      }
      continue;
    }

    return new_node_cluster;
  }

  return {};
}

NodeClusterPtrs RRAG::buildClusterAt(const Eigen::Vector3d& position, StationingCheck stationing) {
  std::vector<Eigen::Quaterniond> candidate_orientations;
  robot_->sampleOrientations(position, &candidate_orientations);

  std::vector<RobotState> candidate_states;
  std::vector<bool> is_candidate_traversable;
  robot_->checkOrientations(position, candidate_orientations, &candidate_states, &is_candidate_traversable, stationing);

  const bool any_traversable = std::any_of(is_candidate_traversable.begin(), is_candidate_traversable.end(), [](bool t) { return t; });
  if (!any_traversable) {
    return {};
  }

  NodeClusterPtrs new_node_cluster;
  for (std::size_t i = 0; i < candidate_orientations.size(); ++i) {
    auto new_node = std::make_shared<GraphNode>();
    new_node->robot_state = candidate_states.at(i);
    new_node->is_traversable = is_candidate_traversable.at(i);

    new_node->candidate_orientations.clear();
    new_node->is_candidate_traversable.clear();

    boost::add_vertex(new_node, *graph_);
    new_node_cluster.push_back(new_node.get());
  }

  robot_->attachSharedGainAnchors(position, new_node_cluster);

  return new_node_cluster;
}

bool RRAG::generateNewEdges(NodeClusterPtrs& node_cluster, std::vector<Edge*>& new_edges) {
  if (kd_tree_data_.size() > 0) {
    NodeClusterPtrs nearest_node_cluster;
    double out_dist_sqr = 0.0;
    if (!findNearestExpansionAnchor(node_cluster.front()->robot_state.getSensorPosition(), nearest_node_cluster, out_dist_sqr)) {
      // The orphan was never indexed, so no KD-tree rebuild is needed.
      for (auto node : node_cluster) {
        NodeId node_id = getNodeId(node, graph_.get());
        boost::clear_vertex(node_id, *graph_);
        boost::remove_vertex(node_id, *graph_);
      }
      return false;
    }

    auto in_edges = generateEdgesFromClusterToCluster(nearest_node_cluster, node_cluster);
    if (in_edges.empty()) {
      // The orphan was never indexed, so no KD-tree rebuild is needed.
      for (auto node : node_cluster) {
        NodeId node_id = getNodeId(node, graph_.get());
        boost::clear_vertex(node_id, *graph_);
        boost::remove_vertex(node_id, *graph_);
      }
      return false;
    }

    new_edges = in_edges;
  }

  auto inter_cluster_edges = generateInterClusterEdges(node_cluster);
  new_edges.insert(new_edges.end(), inter_cluster_edges.begin(), inter_cluster_edges.end());

  auto intra_cluster_edges = generateIntraClusterEdges(node_cluster);
  new_edges.insert(new_edges.end(), intra_cluster_edges.begin(), intra_cluster_edges.end());

  indexNodeCluster(node_cluster);

  return true;
}

std::pair<Node*, Edge*> RRAG::insertIntermediateNode(Node* start_node, Node* goal_node, Edge* edge, int node_index) {
  // Also covers an empty trajectory, where size() - 1 would wrap.
  if (node_index < 0 || static_cast<std::size_t>(node_index) + 1 >= edge->trajectory.size()) {
    return {goal_node, edge};
  }

  // A split within the pose tolerance of either endpoint would create a zero-motion half: keep the
  // whole edge. Same position with a different yaw is fine (an in-place rotation half).
  if (edge->trajectory[node_index].isSameSensorPose(edge->trajectory.front()) ||
      edge->trajectory[node_index].isSameSensorPose(edge->trajectory.back())) {
    return {goal_node, edge};
  }

  auto start_node_id = getNodeId(start_node, graph_.get());
  auto goal_node_id = getNodeId(goal_node, graph_.get());

  auto trajectory = edge->trajectory;

  // A split inherits traversability from its validated parent edge.
  auto intermediate_node = std::make_shared<GraphNode>();
  intermediate_node->is_intermediate = true;
  intermediate_node->robot_state = trajectory[node_index];
  intermediate_node->is_traversable = edge->is_traversable;
  auto intermediate_node_id = boost::add_vertex(intermediate_node, *graph_);

  auto start_to_intermediate_edge = std::make_shared<Edge>();
  start_to_intermediate_edge->trajectory = RobotTrajectory(trajectory.begin(), trajectory.begin() + node_index + 1);
  setUniformTrajectoryTimes(start_to_intermediate_edge->trajectory, robot_->getTimeStep());
  computeCost(start_to_intermediate_edge.get());
  start_to_intermediate_edge->is_traversable = edge->is_traversable;
  boost::add_edge(start_node_id, intermediate_node_id, start_to_intermediate_edge, *graph_);

  // The second half needs a local time origin.
  auto intermediate_to_goal_edge = std::make_shared<Edge>();
  intermediate_to_goal_edge->trajectory = RobotTrajectory(trajectory.begin() + node_index, trajectory.end());
  setUniformTrajectoryTimes(intermediate_to_goal_edge->trajectory, robot_->getTimeStep());
  computeCost(intermediate_to_goal_edge.get());
  intermediate_to_goal_edge->is_traversable = edge->is_traversable;
  boost::add_edge(intermediate_node_id, goal_node_id, intermediate_to_goal_edge, *graph_);

  return {intermediate_node.get(), start_to_intermediate_edge.get()};
}

Node* RRAG::insertIntermediateNodeAtCurrentState(Node* previous, const Edge& executed_edge) {
  const RobotState& reached_state = executed_edge.trajectory.back();

  // Adopt an exact pose match; a duplicate would require a zero-motion edge.
  std::vector<NodeClusterPtrs> colocated_clusters;
  findNearbyNodes(reached_state.getSensorPosition(), colocated_clusters, p_max_edge_length_);
  for (auto& cluster : colocated_clusters) {
    if (!isSamePosition(cluster.front()->robot_state.getSensorPosition(), reached_state.getSensorPosition())) {
      continue;
    }
    for (Node* node : cluster) {
      if (!node->robot_state.isSameSensorPose(reached_state)) {
        continue;
      }
      agent_.printInfo(
          "insertIntermediateNodeAtCurrentState: reached pose matches an existing node; adopting it instead of "
          "inserting a duplicate.");
      if (node != previous) {
        setNewStartNode(previous, node);
      }
      return node;
    }
  }

  auto new_node = std::make_shared<GraphNode>();
  new_node->robot_state = reached_state;
  new_node->is_intermediate = true;
  new_node->is_traversable = true;
  new_node->connect_with_start_node = true;

  const NodeId new_node_id = boost::add_vertex(new_node, *graph_);
  NodeClusterPtrs new_cluster = {new_node.get()};

  // INVARIANT: exactly one intermediate start node exists. It may have zero out-edges while the
  // robot stands off traversable ground; the clusters it cannot reach are kept regardless.
  if (isStartStateTraversable(new_node.get())) {
    // Intermediate nodes are sources only, never planning destinations.
    std::vector<NodeClusterPtrs> nearby_node_clusters;
    findNearbyNodes(new_node->robot_state.getSensorPosition(), nearby_node_clusters, p_max_edge_length_);
    for (auto& nearby_cluster : nearby_node_clusters) {
      generateEdgesFromClusterToCluster(new_cluster, nearby_cluster);
    }
  }

  if (boost::out_degree(new_node_id, *graph_) == 0) {
    agent_.printInfo("insertIntermediateNodeAtCurrentState: no out-edge could be planned; keeping a detached intermediate start node.");
  }

  setNewStartNode(previous, new_node.get());

  return new_node.get();
}

bool RRAG::containsNode(const Node* node) const {
  if (node == nullptr || graph_ == nullptr) return false;
  auto vertices = boost::vertices(*graph_);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    if ((*graph_)[*it].get() == node) return true;
  }
  return false;
}

void RRAG::updateNodeEdge(Node* start_node) {
  updateNodeTraversability(start_node);
  removeUntraversableNodeClusters(start_node);
  updateEdgeTraversability(start_node);
  removeUntraversableEdges();
  generateMissingEdges(start_node);
  removeDisconnectedNodeClusters(start_node);
}

bool RRAG::setNodeOrientation(Node* /*node*/, const Eigen::Quaterniond& /*orientation*/) {
  throw std::runtime_error("RRAG::setNodeOrientation is not implemented.");
}

bool RRAG::rebuildKdTree() {
  kd_tree_data_.clear();
  for (const auto& v : boost::make_iterator_range(boost::vertices(*graph_))) {
    auto node = (*graph_)[v];
    kd_tree_data_.addNode(node->robot_state.getSensorPosition(), node.get());
  }
  kdtree_ = std::unique_ptr<KdTree>(new KdTree(3, kd_tree_data_));
  return true;
}

std::vector<Edge*> RRAG::generateInterClusterEdges(NodeClusterPtrs& node_cluster) {
  auto robot_state = node_cluster.front()->robot_state;
  for (int i = 1; i < static_cast<int>(node_cluster.size()); ++i) {
    if (!node_cluster[i]->robot_state.isSameSensorPosition(robot_state)) {
      throw std::runtime_error("All the nodes in a node cluster should have the same position");
    }
  }

  std::vector<NodeClusterPtrs> nearby_node_clusters;
  findNearbyNodes(robot_state.getSensorPosition(), nearby_node_clusters, p_max_edge_length_);

  std::vector<Edge*> new_edges;
  for (auto nearby_node_cluster : nearby_node_clusters) {
    std::vector<Edge*> out_edges = generateEdgesFromClusterToCluster(node_cluster, nearby_node_cluster);
    new_edges.insert(new_edges.end(), out_edges.begin(), out_edges.end());

    std::vector<Edge*> in_edges = generateEdgesFromClusterToCluster(nearby_node_cluster, node_cluster);
    new_edges.insert(new_edges.end(), in_edges.begin(), in_edges.end());
  }

  return new_edges;
}

std::vector<Edge*> RRAG::generateIntraClusterEdges(NodeClusterPtrs& node_cluster) {
  std::vector<Edge*> new_edges;
  if (node_cluster.size() < 2) {
    return new_edges;
  }

  std::vector<RobotState> states;
  states.reserve(node_cluster.size());
  for (auto* node : node_cluster) {
    states.push_back(node->robot_state);
  }
  const auto adjacency = robot_->getInplaceAdjacency(states);

  for (const auto& [i, j] : adjacency) {
    Node* node_i = node_cluster[i];
    Node* node_j = node_cluster[j];
    if (node_i == node_j) {
      continue;
    }

    const NodeId id_i = getNodeId(node_i, graph_.get());
    const NodeId id_j = getNodeId(node_j, graph_.get());

    if (!boost::edge(id_i, id_j, *graph_).second) {
      std::shared_ptr<Edge> new_edge = std::make_shared<Edge>();
      if (generateRotationEdgeBetweenNodes(node_i, node_j, new_edge.get())) {
        boost::add_edge(id_i, id_j, new_edge, *graph_);
        new_edges.push_back(new_edge.get());
      }
    }
  }

  return new_edges;
}

std::vector<Edge*> RRAG::generateEdgesFromClusterToCluster(NodeClusterPtrs& from_cluster, NodeClusterPtrs& to_cluster) {
  if (from_cluster.front() == to_cluster.front()) {
    return {};
  }

  // Coincident clusters connect by rotation; translation has no travel direction.
  if (isSamePosition(from_cluster.front()->robot_state.getBasePosition(), to_cluster.front()->robot_state.getBasePosition())) {
    return generateRotationEdges(from_cluster, to_cluster);
  }

  std::vector<RobotState> from_states, to_states;
  from_states.reserve(from_cluster.size());
  to_states.reserve(to_cluster.size());
  for (auto* node : from_cluster) from_states.push_back(node->robot_state);
  for (auto* node : to_cluster) to_states.push_back(node->robot_state);

  if (p_translation_adjacency_ == TranslationAdjacency::kStraightLine) {
    return generateStraightLineEdges(from_cluster, to_cluster, from_states, to_states);
  }
  return generatePathHeadingEdges(from_cluster, to_cluster, from_states, to_states);
}

std::vector<Edge*> RRAG::generateRotationEdges(NodeClusterPtrs& from_cluster, NodeClusterPtrs& to_cluster) {
  if (from_cluster.size() != 1 || !from_cluster.front()->is_intermediate) {
    throw std::runtime_error(
        "generateRotationEdges: coincident clusters whose source is not a single intermediate node. A start-node update is "
        "the only source of coincident clusters (sampled clusters are >= min_edge_length apart); a multi-node or "
        "non-intermediate source means that invariant was violated.");
  }

  EdgeList adjacency;
  for (int j = 0; j < static_cast<int>(to_cluster.size()); ++j) {
    // A pose-identical target needs no rotation edge.
    if (from_cluster.front()->robot_state.isSameSensorOrientation(to_cluster[j]->robot_state)) {
      agent_.printInfo("generateRotationEdges: skipping zero-rotation edge to a pose-identical node.");
      continue;
    }
    adjacency.emplace_back(0, j);
  }

  std::vector<Edge*> new_edges;
  for (const auto& [from_idx, to_idx] : adjacency) {
    Node* from_node = from_cluster[from_idx];
    Node* to_node = to_cluster[to_idx];

    const NodeId from_node_id = getNodeId(from_node, graph_.get());
    const NodeId to_node_id = getNodeId(to_node, graph_.get());
    if (from_node_id == to_node_id) {
      throw std::runtime_error("from_node_id == to_node_id");
    }

    if (!boost::edge(from_node_id, to_node_id, *graph_).second) {
      std::shared_ptr<Edge> edge = std::make_shared<Edge>();
      if (generateRotationEdgeBetweenNodes(from_node, to_node, edge.get())) {
        new_edges.push_back(insertEdge(from_node, to_node, from_node_id, to_node_id, edge));
      }
    }
  }

  return new_edges;
}

std::vector<Edge*> RRAG::generateStraightLineEdges(NodeClusterPtrs& from_cluster, NodeClusterPtrs& to_cluster,
                                                   const std::vector<RobotState>& from_states, const std::vector<RobotState>& to_states) {
  const Eigen::Vector3d chord = to_states.front().getSensorPosition() - from_states.front().getSensorPosition();
  const EdgeList adjacency = robot_->getTranslationAdjacency(from_states, to_states, chord, chord, /*connect_enclosing_cell=*/false);

  std::vector<Edge*> new_edges;
  for (const auto& [from_idx, to_idx] : adjacency) {
    Node* from_node = from_cluster[from_idx];
    Node* to_node = to_cluster[to_idx];

    const NodeId from_node_id = getNodeId(from_node, graph_.get());
    const NodeId to_node_id = getNodeId(to_node, graph_.get());
    if (from_node_id == to_node_id) {
      throw std::runtime_error("from_node_id == to_node_id");
    }

    if (!boost::edge(from_node_id, to_node_id, *graph_).second) {
      std::shared_ptr<Edge> edge = std::make_shared<Edge>();
      if (generateEdgeBetweenNodes(from_node, to_node, edge.get())) {
        new_edges.push_back(insertEdge(from_node, to_node, from_node_id, to_node_id, edge));
      }
    }
  }

  return new_edges;
}

Edge* RRAG::insertEdge(Node* from_node, Node* to_node, NodeId from_node_id, NodeId to_node_id, std::shared_ptr<Edge> edge) {
  boost::add_edge(from_node_id, to_node_id, edge, *graph_);
  if (from_node->as<GraphNode>()->connect_with_start_node) {
    to_node->as<GraphNode>()->connect_with_start_node = true;
  }
  return edge.get();
}

std::vector<Edge*> RRAG::generatePathHeadingEdges(NodeClusterPtrs& from_cluster, NodeClusterPtrs& to_cluster,
                                                  const std::vector<RobotState>& from_states, const std::vector<RobotState>& to_states) {
  // getNodeId is a linear scan and the enclosing-cell mode asks for several pairs over
  // these same two clusters, so look each ID up once.
  std::vector<NodeId> from_ids;
  from_ids.reserve(from_cluster.size());
  for (auto* node : from_cluster) from_ids.push_back(getNodeId(node, graph_.get()));
  std::vector<NodeId> to_ids;
  to_ids.reserve(to_cluster.size());
  for (auto* node : to_cluster) to_ids.push_back(getNodeId(node, graph_.get()));

  // Skip an already connected cluster pair before planning its corridor; a partially pruned cell
  // is therefore not refilled until all its edges are gone.
  const std::unordered_set<NodeId> to_id_set(to_ids.begin(), to_ids.end());
  for (const NodeId from_id : from_ids) {
    for (const auto& edge_id : boost::make_iterator_range(boost::out_edges(from_id, *graph_))) {
      if (to_id_set.count(boost::target(edge_id, *graph_)) > 0) {
        return {};
      }
    }
  }

  // Selecting against the chord with no fan-out yields exactly one pair: the one to plan.
  const Eigen::Vector3d chord = to_states.front().getSensorPosition() - from_states.front().getSensorPosition();
  const EdgeList seed_adjacency = robot_->getTranslationAdjacency(from_states, to_states, chord, chord, /*connect_enclosing_cell=*/false);
  if (seed_adjacency.empty()) {
    return {};
  }
  const int seed_from_idx = seed_adjacency.front().first;
  const int seed_to_idx = seed_adjacency.front().second;

  Edge seed_edge;
  if (!generateEdgeBetweenNodes(from_cluster[seed_from_idx], to_cluster[seed_to_idx], &seed_edge)) {
    return {};
  }

  // A bent corridor's ends differ from the chord, so re-select against the trajectory's
  // own end directions.
  const Eigen::Vector3d departure_direction = trajectoryDepartureDelta(seed_edge.trajectory);
  const Eigen::Vector3d arrival_direction = trajectoryArrivalDelta(seed_edge.trajectory);
  const EdgeList adjacency = robot_->getTranslationAdjacency(from_states, to_states, departure_direction, arrival_direction,
                                                             p_translation_adjacency_ == TranslationAdjacency::kPathHeadingCell);

  std::vector<Edge*> new_edges;
  for (const auto& [from_idx, to_idx] : adjacency) {
    Node* from_node = from_cluster[from_idx];
    Node* to_node = to_cluster[to_idx];

    const NodeId from_node_id = from_ids[from_idx];
    const NodeId to_node_id = to_ids[to_idx];
    if (from_node_id == to_node_id) {
      throw std::runtime_error("from_node_id == to_node_id");
    }

    if (!boost::edge(from_node_id, to_node_id, *graph_).second) {
      std::shared_ptr<Edge> edge = std::make_shared<Edge>();
      // The seed is a retargeting template: it enters the graph only if its own pair was
      // re-selected.
      if (from_idx == seed_from_idx && to_idx == seed_to_idx) {
        *edge = seed_edge;
      } else if (!buildRetargetedEdge(seed_edge, from_node, to_node, edge.get())) {
        continue;
      }
      new_edges.push_back(insertEdge(from_node, to_node, from_node_id, to_node_id, edge));
    }
  }

  return new_edges;
}

void RRAG::updateNodeTraversability(Node* start_node) {
  // Map updates can make a stationing position inadmissible.
  for (const auto& node_cluster : kd_tree_data_.nodes) {
    if (isAffectedByMapUpdate(node_cluster)) {
      refreshClusterTraversability(node_cluster, &agent_.getMap());
    }
  }

  // The start node is judged like any other; removeUntraversableNodeClusters spares it by identity,
  // so its flag stays an honest verdict. The stationing check includes the clearance margin, which
  // the robot's own traversability does not, so say which of the two failed.
  if (!start_node->is_traversable) {
    const std::string cause = isStartStateTraversable(start_node)
                                  ? "the pose is traversable; the stationing rule or the clearance margin rejected it"
                                  : "the pose itself is untraversable";
    agent_.printWarning("RRAG: the start node failed its stationing check (" + cause +
                        "); it stays the start node until the robot moves on.");
  }
}

void RRAG::removeUntraversableNodeClusters(Node* start_node) {
  std::vector<NodeClusterPtrs> all_node_clusters = kd_tree_data_.nodes;

  std::vector<NodeId> untraversable_node_ids;
  for (const auto& node_cluster : all_node_clusters) {
    // The robot stands on the start node: its cluster stays a vertex whatever its flags say, and
    // is pruned like any other once the robot has moved on.
    if (std::find(node_cluster.begin(), node_cluster.end(), start_node) != node_cluster.end()) {
      continue;
    }

    bool has_traversable_node = false;
    for (auto node : node_cluster) {
      if (node->is_traversable) {
        has_traversable_node = true;
        break;
      }
    }

    if (!has_traversable_node) {
      for (auto node : node_cluster) {
        NodeId node_id = getNodeId(node, graph_.get());
        untraversable_node_ids.push_back(node_id);
      }
    }
  }

  if (!untraversable_node_ids.empty()) {
    for (const auto& node_id : untraversable_node_ids) {
      boost::clear_vertex(node_id, *graph_);
      boost::remove_vertex(node_id, *graph_);
    }
    rebuildKdTree();
  }
}

void RRAG::updateEdgeTraversability(Node* start_node) {
  const NodeId start_node_id = getNodeId(start_node, graph_.get());
  for (const auto& edge_id : boost::make_iterator_range(boost::edges(*graph_))) {
    Edge* edge = (*graph_)[edge_id].get();
    const bool is_start_edge = boost::source(edge_id, *graph_) == start_node_id || boost::target(edge_id, *graph_) == start_node_id;
    if (is_start_edge || isAffectedByMapUpdate(edge->trajectory)) {
      revalidateEdge(edge);
    }
  }
}

void RRAG::removeUntraversableEdges() {
  // Collect first to avoid invalidating the edge iterator while iterating over the graph.
  std::vector<EdgeId> untraversable_edges;
  for (const auto& e : boost::make_iterator_range(boost::edges(*graph_))) {
    if (!(*graph_)[e]->is_traversable) {
      untraversable_edges.push_back(e);
    }
  }

  for (const auto& e : untraversable_edges) {
    boost::remove_edge(e, *graph_);
  }
}

void RRAG::generateMissingEdges(Node* start_node) {
  std::vector<NodeClusterPtrs> all_node_clusters = kd_tree_data_.nodes;

  // Each unordered pair of clusters is tried once, in both directions.
  std::set<std::pair<const Node*, const Node*>> tried_pairs;
  // A connection can only appear within an edge length of the update, along the axes the robot samples.
  const Eigen::Vector3d neighbour_reach(p_max_edge_length_, p_max_edge_length_, robot_->samplesHeight() ? p_max_edge_length_ : 0.0);

  for (auto& node_cluster : all_node_clusters) {
    if (isAffectedByMapUpdate(node_cluster)) {
      generateIntraClusterEdges(node_cluster);
    }
    if (!isAffectedByMapUpdate(node_cluster, neighbour_reach)) {
      continue;
    }

    std::vector<NodeClusterPtrs> nearby_node_clusters;
    findNearbyNodes(node_cluster.front()->robot_state.getSensorPosition(), nearby_node_clusters, p_max_edge_length_);
    for (auto& nearby_node_cluster : nearby_node_clusters) {
      if (nearby_node_cluster.front() == node_cluster.front() ||
          !tried_pairs.insert(std::minmax(node_cluster.front(), nearby_node_cluster.front())).second) {
        continue;
      }
      // The box over both endpoint bodies holds the straight steer between them.
      const RobotTrajectory endpoints = {node_cluster.front()->robot_state, nearby_node_cluster.front()->robot_state};
      if (!isAffectedByMapUpdate(endpoints)) {
        continue;
      }
      generateEdgesFromClusterToCluster(node_cluster, nearby_node_cluster);
      generateEdgesFromClusterToCluster(nearby_node_cluster, node_cluster);
    }
  }

  // Intermediate start nodes are not indexed and must be connected explicitly.
  if (start_node->is_intermediate) {
    // Collect first to avoid invalidating the iterator while removing.
    auto start_node_id = getNodeId(start_node, graph_.get());
    std::vector<EdgeId> in_edges;
    for (const auto& edge_id : boost::make_iterator_range(boost::in_edges(start_node_id, *graph_))) {
      in_edges.push_back(edge_id);
    }
    for (const auto& edge_id : in_edges) {
      boost::remove_edge(edge_id, *graph_);
    }

    connectStartNode(start_node);
  }
}

void RRAG::connectStartNode(Node* start_node) {
  if (!start_node->is_intermediate || !isStartStateTraversable(start_node)) {
    return;
  }

  // Intermediate nodes only have outgoing edges.
  NodeClusterPtrs immediate_node_cluster = {start_node};
  std::vector<NodeClusterPtrs> reachable_node_clusters;
  findNearbyNodes(start_node->robot_state.getSensorPosition(), reachable_node_clusters, p_max_edge_length_);
  for (auto& nearby_node_cluster : reachable_node_clusters) {
    generateEdgesFromClusterToCluster(immediate_node_cluster, nearby_node_cluster);
  }
}

bool RRAG::isStartStateTraversable(const Node* start_node) const {
  return robot_->isStateTraversable(start_node->robot_state, &agent_.getMap());
}

void RRAG::removeDisconnectedNodeClusters(Node* start_node) {
  if (!isStartStateTraversable(start_node)) {
    return;
  }

  for (auto v : boost::make_iterator_range(boost::vertices(*graph_))) {
    (*graph_)[v]->connect_with_start_node = false;
  }

  std::vector<Node*> connected_nodes = runBfs(start_node, graph_.get());
  for (auto node : connected_nodes) {
    node->as<GraphNode>()->connect_with_start_node = true;
  }

  std::vector<NodeClusterPtrs> all_node_clusters = kd_tree_data_.nodes;

  std::vector<NodeId> disconnected_node_ids;
  std::size_t disconnected_cluster_count = 0;
  for (const auto& node_cluster : all_node_clusters) {
    bool has_connected_node = false;
    for (auto node : node_cluster) {
      if (node->as<GraphNode>()->connect_with_start_node) {
        has_connected_node = true;
        break;
      }
    }

    if (!has_connected_node) {
      ++disconnected_cluster_count;
      for (auto node : node_cluster) {
        NodeId node_id = getNodeId(node, graph_.get());
        disconnected_node_ids.push_back(node_id);
      }
    }
  }

  if (!disconnected_node_ids.empty()) {
    if (boost::out_degree(getNodeId(start_node, graph_.get()), *graph_) == 0) {
      agent_.printWarning("RRAG: start node has no out-edges; pruning " + std::to_string(disconnected_cluster_count) +
                          " unreachable clusters, regrowing from the start node");
    }
    for (const auto& node_id : disconnected_node_ids) {
      boost::clear_vertex(node_id, *graph_);
      boost::remove_vertex(node_id, *graph_);
    }
    rebuildKdTree();
  }
}

}  // namespace generator
}  // namespace open_apero
