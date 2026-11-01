#include "open_apero_core/gain/gain_suppression.h"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/common/utils.h"

namespace open_apero {

namespace {

struct NodeCluster {
  Eigen::Vector3d position;
  std::vector<Node*> nodes;
  double score = 0.0;
};

}  // namespace

void suppressNonMaximumClusters(const std::vector<Node*>& nodes, const Node* start_node, const double radius) {
  for (Node* node : nodes) {
    node->is_gain_suppressed = false;
  }
  if (radius <= 0.0) {
    return;
  }

  std::vector<NodeCluster> clusters;
  for (Node* node : nodes) {
    if (node == start_node) {
      continue;
    }
    const Eigen::Vector3d& position = node->robot_state.getSensorPosition();
    auto cluster = std::find_if(clusters.begin(), clusters.end(),
                                [&position](const NodeCluster& candidate) { return isSamePosition(candidate.position, position); });
    if (cluster == clusters.end()) {
      clusters.push_back(NodeCluster{position, {}, 0.0});
      cluster = std::prev(clusters.end());
    }
    cluster->nodes.push_back(node);
    if (node->is_traversable) {
      cluster->score = std::max(cluster->score, node->getSingleNodeGain());
    }
  }

  // Ties keep the order the clusters were met in, so the outcome follows the node order alone.
  std::vector<std::size_t> order;
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].score > 0.0) {
      order.push_back(i);
    }
  }
  std::stable_sort(order.begin(), order.end(), [&clusters](std::size_t a, std::size_t b) { return clusters[a].score > clusters[b].score; });

  const double radius_squared = radius * radius;
  std::vector<const NodeCluster*> kept;
  for (const std::size_t i : order) {
    const NodeCluster& cluster = clusters[i];
    const bool is_suppressed = std::any_of(kept.begin(), kept.end(), [&cluster, radius_squared](const NodeCluster* keeper) {
      return (keeper->position - cluster.position).squaredNorm() < radius_squared;
    });
    if (!is_suppressed) {
      kept.push_back(&cluster);
      continue;
    }
    for (Node* node : cluster.nodes) {
      node->is_gain_suppressed = true;
    }
  }
}

}  // namespace open_apero
