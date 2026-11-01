#include "open_apero_core/plugin/generator/generator.h"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/visualization/visualization_helpers.h"

namespace open_apero {

Generator::Generator(AgentInterface& agent) : Plugin(agent) {}

void Generator::setupFromParamMap(Configurable::ParamMap* /*param_map*/) {
  robot_ = &agent_.getRobot();
}

void Generator::resetEpisode() {
  kd_tree_data_.clear();
  kdtree_.reset();
}

bool Generator::containsNode(const Node* node) const {
  if (node == nullptr) {
    return false;
  }
  // Address-only traversal, like RRAG's and TreeGenerator's overrides: the query
  // pointer is allowed to be one this generator has already freed.
  for (const NodeClusterPtrs& cluster : kd_tree_data_.nodes) {
    for (const Node* indexed_node : cluster) {
      if (indexed_node == node) {
        return true;
      }
    }
  }
  return false;
}

bool Generator::findNearbyNodes(const Eigen::Vector3d& point, std::vector<NodeClusterPtrs>& nearby_node_clusters, double max_length) {
  return findNearbyNodesInIndex(kdtree_.get(), kd_tree_data_, point, &nearby_node_clusters, max_length);
}

void Generator::refreshClusterTraversability(const NodeClusterPtrs& cluster, Map* /*map*/) const {
  if (cluster.empty()) {
    throw std::invalid_argument("Generator::refreshClusterTraversability: cluster is empty");
  }

  std::vector<Eigen::Quaterniond> orientations;
  orientations.reserve(cluster.size());
  for (const Node* node : cluster) {
    orientations.push_back(node->robot_state.getSensorTransform().orientation);
  }

  std::vector<RobotState> states;
  std::vector<bool> is_traversable;
  robot_->checkOrientations(cluster.front()->robot_state.getSensorPosition(), orientations, &states, &is_traversable);

  for (std::size_t i = 0; i < cluster.size(); ++i) {
    cluster[i]->is_traversable = is_traversable.at(i);
  }
}

bool Generator::isAffectedByMapUpdate(const RobotTrajectory& states, const Eigen::Vector3d& reach) const {
  Eigen::Vector3d body_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d body_max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const RobotState& state : states) {
    Eigen::Vector3d state_min;
    Eigen::Vector3d state_max;
    if (!robot_->getBodyBounds(state, &state_min, &state_max)) {
      return true;
    }
    body_min = body_min.cwiseMin(state_min);
    body_max = body_max.cwiseMax(state_max);
  }
  body_min -= reach;
  body_max += reach;
  Eigen::Vector3d updated_min;
  Eigen::Vector3d updated_max;
  const bool map_updated = getMap()->getUpdatedMapRegion(&updated_min, &updated_max);
  return map_updated && boxesOverlap(body_min, body_max, updated_min, updated_max);
}

bool Generator::isAffectedByMapUpdate(const NodeClusterPtrs& cluster, const Eigen::Vector3d& reach) const {
  RobotTrajectory states;
  states.reserve(cluster.size());
  for (const Node* node : cluster) {
    states.push_back(node->robot_state);
  }
  return isAffectedByMapUpdate(states, reach);
}

bool Generator::findNearbyNodesInIndex(const KdTree* kdtree, const KdTreeData& data, const Eigen::Vector3d& point,
                                       std::vector<NodeClusterPtrs>* nearby_node_clusters, double max_length) {
  nearby_node_clusters->clear();
  if (!kdtree) {
    return false;
  }

  const double radius = (max_length > 1e-2) ? max_length : std::numeric_limits<double>::infinity();
  for (const std::size_t index : findPointsWithin(*kdtree, point, radius)) {
    nearby_node_clusters->push_back(data.nodes[index]);
  }
  return !nearby_node_clusters->empty();
}

bool Generator::findNearestNode(const Eigen::Vector3d& point, NodeClusterPtrs& nearest_node_cluster, double& dist_squared) {
  if (!findNearestNodeInIndex(kdtree_.get(), kd_tree_data_, point, &nearest_node_cluster, &dist_squared)) {
    agent_.printWarning("Failed to find nearest node.");
    return false;
  }

  return true;
}

bool Generator::findNearestNodeInIndex(const KdTree* kdtree, const KdTreeData& data, const Eigen::Vector3d& point,
                                       NodeClusterPtrs* nearest_node_cluster, double* dist_squared) {
  nearest_node_cluster->clear();
  if (!kdtree) {
    return false;
  }

  double query_pt[3] = {point.x(), point.y(), point.z()};

  std::size_t ret_index;
  double out_dist_sqr;
  nanoflann::KNNResultSet<double> resultSet(1);
  resultSet.init(&ret_index, &out_dist_sqr);

  if (!kdtree->findNeighbors(resultSet, query_pt)) {
    return false;
  }

  *nearest_node_cluster = data.nodes[ret_index];
  *dist_squared = out_dist_sqr;
  return true;
}

void Generator::addToKdTree(const NodeClusterPtrs& cluster) {
  addClusterToIndex(kdtree_.get(), &kd_tree_data_, cluster);
}

void Generator::addClusterToIndex(KdTree* kdtree, KdTreeData* data, const NodeClusterPtrs& cluster) {
  const int prev_size = data->size();
  for (auto node : cluster) {
    int id = data->addNode(node->robot_state.getSensorPosition(), node);
    if (id != prev_size) {
      throw std::runtime_error(
          "Generator::addClusterToIndex: the cluster must be non-intermediate, co-located and at a position not yet indexed");
    }
  }
  kdtree->addPoints(data->points.size() - 1, data->points.size() - 1);
}

// Flattens the KD-tree's non-intermediate clusters into a node list and renders them.
void Generator::getVisualization(VisualizationMarkers* markers) {
  std::vector<Node*> all_nodes;
  for (const auto& cluster : kd_tree_data_.nodes) {
    for (Node* node : cluster) {
      all_nodes.push_back(node);
    }
  }

  getNodeVisualizationMarkers(markers, all_nodes, true);
}

}  // namespace open_apero
