#pragma once

/**
 * @file gain_suppression.h
 * @brief Non-maximum suppression over node clusters, run before each path search.
 */

#include <vector>

#include "open_apero_core/graph/node.h"

namespace open_apero {

/**
 * @brief Greedy non-maximum suppression over node clusters: decide which nodes a path may take gain from.
 *
 * A node cluster is the nodes sharing one sensor position, scored by the best gain among its traversable nodes.
 * From the highest score down, a cluster closer than @p radius to a cluster already kept is suppressed, else kept.
 * Clusters scoring 0 or less take no part. Rewrites Node::is_gain_suppressed on every node of @p nodes.
 *
 * @param nodes The nodes to decide.
 * @param start_node The search's start node: never scored and never suppressed; may be nullptr.
 * @param radius Suppression radius [m] between cluster positions; 0 suppresses nothing.
 */
void suppressNonMaximumClusters(const std::vector<Node*>& nodes, const Node* start_node, double radius);

}  // namespace open_apero
