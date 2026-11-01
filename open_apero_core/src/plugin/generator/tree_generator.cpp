#include "open_apero_core/plugin/generator/tree_generator.h"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/map/map.h"

namespace open_apero {

TreeGenerator::TreeGenerator(AgentInterface& agent) : GraphGenerator(agent) {}

std::vector<Node*> TreeGenerator::getAllNodes(Node* /*start_node*/) const {
  std::vector<TreeNode*> all_tree_nodes;
  root_node_->getTree(&all_tree_nodes);

  std::vector<Node*> all_nodes(all_tree_nodes.begin(), all_tree_nodes.end());
  return all_nodes;
}

std::vector<Edge*> TreeGenerator::getAllEdges(Node* /*start_node*/) const {
  std::vector<TreeNode*> all_tree_nodes;
  root_node_->getTree(&all_tree_nodes);

  std::vector<Edge*> all_edges;
  for (const auto& tree_node : all_tree_nodes) {
    all_edges.push_back(&(tree_node->edge));
  }
  return all_edges;
}

std::pair<GraphType*, Node*> TreeGenerator::generateInitNode(const RobotState& init_state) {
  root_node_ = std::make_unique<TreeNode>();
  root_node_->robot_state = init_state;
  root_node_->is_traversable = true;

  // The root node is always safe — mark every orientation as traversable.
  robot_->sampleOrientations(init_state.getSensorPosition(), &root_node_->candidate_orientations);
  root_node_->is_candidate_traversable.assign(root_node_->candidate_orientations.size(), true);

  root_node_->parent = nullptr;

  root_node_->edge.trajectory.push_back(init_state);

  // The initial root is replaced after the first executed edge.
  root_node_->is_intermediate = true;

  // Intermediate roots are not indexed; the first sample seeds from it directly.
  kd_tree_data_.clear();
  kdtree_ = std::unique_ptr<KdTree>(new KdTree(3, kd_tree_data_));

  return {nullptr, root_node_.get()};
}

bool TreeGenerator::generateNewEdges(NodeClusterPtrs& node_cluster, std::vector<Edge*>& new_edges) {
  if (node_cluster.size() > 1) {
    throw std::runtime_error("TreeGenerator::generateNewEdges called with more than one new node");
  }
  auto node = node_cluster.front();

  auto tree_node = node->as<TreeNode>();
  if (!tree_node) {
    throw std::runtime_error("TreeGenerator::generateNewEdges called with non-TreeNode node");
  }

  TreeNode* nearest_node = tree_node->parent;
  // Coincident nodes connect by rotation rather than translation.
  const bool is_rotation = isSamePosition(nearest_node->robot_state.getBasePosition(), tree_node->robot_state.getBasePosition());
  Edge new_edge;
  const bool built = is_rotation ? generateRotationEdgeBetweenNodes(nearest_node, tree_node, &new_edge)
                                 : generateEdgeBetweenNodes(nearest_node, tree_node, &new_edge);
  if (!built) {
    removeNodeFromParent(tree_node);
    return false;
  }

  tree_node->edge = new_edge;
  new_edges.push_back(&(tree_node->edge));

  addToKdTree(node_cluster);

  return true;
}

std::pair<Node*, Edge*> TreeGenerator::insertIntermediateNode(Node* start_node, Node* goal_node, Edge* edge, int node_index) {
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

  auto start_tree_node = start_node->as<TreeNode>();
  auto goal_tree_node = goal_node->as<TreeNode>();
  auto trajectory = edge->trajectory;

  // A split inherits traversability from its validated parent edge.
  auto new_node = start_tree_node->spawnChild();
  new_node->is_intermediate = true;
  new_node->robot_state = trajectory[node_index];
  new_node->is_traversable = edge->is_traversable;

  new_node->edge.trajectory = RobotTrajectory(trajectory.begin(), trajectory.begin() + node_index + 1);
  setUniformTrajectoryTimes(new_node->edge.trajectory, robot_->getTimeStep());
  computeCost(&(new_node->edge));
  new_node->edge.is_traversable = edge->is_traversable;

  moveNodeToNewParent(goal_tree_node, new_node);

  // The second half needs a local time origin.
  goal_tree_node->edge.trajectory = RobotTrajectory(trajectory.begin() + node_index, trajectory.end());
  setUniformTrajectoryTimes(goal_tree_node->edge.trajectory, robot_->getTimeStep());
  computeCost(&(goal_tree_node->edge));
  goal_tree_node->edge.is_traversable = edge->is_traversable;

  return {new_node, &(new_node->edge)};
}

Node* TreeGenerator::insertIntermediateNodeAtCurrentState(Node* previous, const Edge& executed_edge) {
  auto previous_tree_node = previous->as<TreeNode>();
  if (!previous_tree_node) {
    throw std::runtime_error("TreeGenerator::insertIntermediateNodeAtCurrentState: previous is not a TreeNode");
  }

  // Adopt an exact pose match; a duplicate would require a zero-motion edge.
  if (executed_edge.trajectory.back().isSameSensorPose(previous_tree_node->robot_state)) {
    return previous;
  }

  auto new_root_owned = std::make_unique<TreeNode>();
  TreeNode* new_root = new_root_owned.get();
  new_root->robot_state = executed_edge.trajectory.back();
  new_root->is_intermediate = true;
  new_root->is_traversable = true;
  new_root->parent = nullptr;
  // Roots use a single-state trajectory.
  new_root->edge.trajectory.push_back(new_root->robot_state);

  // Coincident roots require an in-place rotation.
  const bool is_rotation = isSamePosition(new_root->robot_state.getBasePosition(), previous_tree_node->robot_state.getBasePosition());
  Edge edge_to_previous;
  const bool built = is_rotation ? generateRotationEdgeBetweenNodes(new_root, previous_tree_node, &edge_to_previous)
                                 : generateEdgeBetweenNodes(new_root, previous_tree_node, &edge_to_previous);
  // INVARIANT: a tree has no vertex without a parent edge, so an unplannable edge drops
  // the whole tree and starts a fresh single-node root at the reached state.
  if (!built) {
    return generateInitNode(executed_edge.trajectory.back()).second;
  }

  // Reparent before transferring root ownership.
  previous_tree_node->edge = edge_to_previous;
  previous_tree_node->edge.is_traversable = true;
  previous_tree_node->parent = new_root;

  new_root_owned->children.push_back(std::move(root_node_));
  root_node_ = std::move(new_root_owned);

  return root_node_.get();
}

// Address-only traversal is safe for a possibly dangling query pointer.
bool TreeGenerator::containsNode(const Node* node) const {
  if (node == nullptr || !root_node_) return false;
  std::vector<const TreeNode*> stack{root_node_.get()};
  while (!stack.empty()) {
    const TreeNode* cur = stack.back();
    stack.pop_back();
    if (static_cast<const Node*>(cur) == node) return true;
    for (const auto& child : cur->children) {
      stack.push_back(child.get());
    }
  }
  return false;
}

void TreeGenerator::updateNodeEdge(Node* node) {
  updateNodeEdgeRecursive(node);

  // Pruning invalidates KD-tree pointers.
  rebuildKdTree();
}

bool TreeGenerator::setNodeOrientation(Node* node, const Eigen::Quaterniond& orientation) {
  auto tree_node = node->as<TreeNode>();
  if (!tree_node) {
    throw std::runtime_error("TreeGenerator::setNodeOrientation called with non-TreeNode node");
  }

  RobotState requested_state = tree_node->robot_state;
  robot_->placeRobotAtSensorPose(&requested_state, {tree_node->robot_state.getSensorPosition(), orientation}, &agent_.getMap());

  // Refuse before mutating: steer cannot build an edge between identical viewpoints, so
  // an orientation that makes the node a pose twin of its parent or of a coincident
  // child has no executable edge on that side and cannot be applied.
  if (tree_node->parent && requested_state.isSameSensorPose(tree_node->parent->robot_state)) {
    return false;
  }
  for (const auto& child : tree_node->children) {
    if (requested_state.isSameSensorPose(child->robot_state)) {
      return false;
    }
  }

  if (tree_node->parent) {
    if (!robot_->retargetTrajectoryTail(&(tree_node->edge.trajectory), requested_state)) {
      // The parent check above should make this unreachable; refuse without mutating.
      return false;
    }
    computeCost(&(tree_node->edge));
  }

  tree_node->robot_state = requested_state;

  for (auto& child : tree_node->children) {
    if (!robot_->retargetTrajectoryHead(&(child->edge.trajectory), tree_node->robot_state)) {
      // The children check above should make this unreachable, but here the node and its
      // tail are already updated -- there is no way to refuse without mutating. Fail loudly.
      throw std::logic_error("TreeGenerator::setNodeOrientation: child head rebuild refused after the pose-twin pre-check passed.");
    }
    computeCost(&(child->edge));
  }

  return true;
}

bool TreeGenerator::rebuildKdTree() {
  kd_tree_data_.clear();
  std::vector<TreeNode*> all_nodes;
  root_node_->getTree(&all_nodes);
  for (int i = 0; i < static_cast<int>(all_nodes.size()); ++i) {
    kd_tree_data_.addNode(all_nodes[i]->robot_state.getSensorPosition(), all_nodes[i]);
  }
  kdtree_ = std::unique_ptr<KdTree>(new KdTree(3, kd_tree_data_));
  return true;
}

void TreeGenerator::updateNodeEdgeRecursive(Node* node) {
  auto& children = node->as<TreeNode>()->children;

  size_t i = 0;
  while (i < children.size()) {
    auto tree_node = children[i]->as<TreeNode>();
    if (isAffectedByMapUpdate(tree_node->edge.trajectory)) {
      revalidateEdge(&tree_node->edge);
    }

    if (!tree_node->edge.is_traversable) {
      children.erase(children.begin() + i);
    } else {
      i++;
    }
  }

  for (auto& child : children) {
    updateNodeEdgeRecursive(child.get());
  }
}

}  // namespace open_apero
