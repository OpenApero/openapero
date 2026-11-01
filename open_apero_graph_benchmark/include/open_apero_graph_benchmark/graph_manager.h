#pragma once

/**
 * @file graph_manager.h
 * @brief Manages loading, construction, gain assignment, and serialization
 *        of exploration graphs for offline planning benchmarks.
 */

#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <open_apero_core/common/seeding.h>
#include <open_apero_core/common/typedef.h>
#include <open_apero_core/graph/edge.h>
#include <open_apero_core/graph/graph_node.h>
#include <open_apero_core/visualization/visualization_markers.h>

namespace open_apero {

/// @brief Spatial gain-model region with center, radius, and gain range.
struct GainRegion {
  double x_center;
  double y_center;
  double radius;
  double min_gain;
  double max_gain;
};

/// @brief Per-node data loaded from a graph file: position, gain, and adjacency list.
struct NodeData {
  Eigen::Vector3d pos;
  double gain;
  std::vector<int> adjacent_original_ids;
};
/**
 * @brief Loaded graph data, keyed by original node ID.
 *
 * Unordered. generateGraphFromData() walks this container to add Boost vertices, so its iteration order becomes
 * the graph's vertex order and hence the order planners enumerate candidates in -- hash buckets, not the graph file.
 *
 * GraphTspSearch's tour construction is order-sensitive with no cost tie-break, so its results, and the TSP digests
 * in `data/<dataset>/reference.json`, depend on this order: a std::map, for one, changes them. NBS, DBS and SPT
 * results do not depend on it.
 *
 * The same graph file therefore yields the same selection on the same build, not across libstdc++ versions or
 * differing key sets.
 */
using GraphData = std::unordered_map<int, NodeData>;

/**
 * @brief Manages graph construction, gain assignment, and I/O for offline
 *        exploration benchmarks.
 *
 * Reads graph topology and gains from text files, constructs a Boost graph
 * with bidirectional edges, supports random gain re-assignment from a spatial
 * gain model, and provides serialization and visualization utilities.
 */
class GraphManager {
 public:
  GraphManager() = default;
  ~GraphManager() = default;

  /**
   * @brief Load graph node positions, gains, and adjacency from a text file.
   * @param input_graph_filepath Path to a serialized graph file.
   */
  void readGraphDataFromFile(const std::string& input_graph_filepath);

  /**
   * @brief Load a spatial gain model from a text file.
   * @param gain_model_filepath Path to the gain model file.
   */
  void readGainModelFromFile(const std::string& gain_model_filepath);

  /// @brief Clear the graph and all internal lookup maps.
  void reset();

  /**
   * @brief Grow the perceived graph from the loaded data around the node the robot is at.
   *
   * Incremental: keeps the vertices already in the graph, adds every loaded node within @p max_radius of
   * @p current_node_id together with its edges to nodes in the graph, recomputes is_at_frontier for every
   * vertex, and removes the vertices unreachable from the current node (freeing their nodes). reset() starts
   * over from an empty graph.
   * @param current_node_id Original ID of the node the robot is at; it becomes getCurrentNode().
   * @param max_radius Perception radius around the current node.
   * @return Pointer to the internal graph.
   * @throws std::runtime_error if @p current_node_id is not in the loaded data, or @p max_radius is negative.
   */
  GraphType* generateGraphFromData(int current_node_id, double max_radius = std::numeric_limits<double>::max());

  /// @brief Return the current root node pointer.
  Node* getCurrentNode() { return current_node_; }

  /// @brief Return the minimum gain in the complete loaded graph.
  double getMinGain() const { return min_gain_; }

  /// @brief Return the maximum gain in the complete loaded graph.
  double getMaxGain() const { return max_gain_; }

  /// @brief Return the number of nodes in the complete loaded graph.
  std::size_t getTotalNodeCount() const { return graph_data_.size(); }

  /**
   * @brief Map a node pointer back to its original file ID.
   *
   * Checked on purpose: an unchecked operator[] would insert a 0 mapping for an unknown node and hand back
   * ID 0, silently corrupting both the map and every logged or serialized ID derived from it.
   * @param node Pointer to the graph node.
   * @return The original integer ID.
   * @throws std::out_of_range if @p node is not part of the current graph.
   */
  int getOriginalId(const Node* node) const { return node_ptr_to_original_id_map_.at(node); }

  /**
   * @brief Resolve an original file ID to the node currently representing it.
   *
   * Lets callers hold on to IDs rather than raw pointers across generateGraphFromData() calls, which may prune
   * vertices and therefore free the nodes behind them.
   * @param original_id The original integer ID.
   * @return The node, or nullptr if that ID is not part of the current graph.
   */
  Node* getNodeByOriginalId(int original_id) const {
    const auto it = original_id_to_node_ptr_map_.find(original_id);
    return it == original_id_to_node_ptr_map_.end() ? nullptr : it->second;
  }

  /**
   * @brief Seed the gain-sampling RNG.
   *
   * Call before assignNewGains() to make a generated instance reproducible. Follows the repository-wide
   * convention: @p random_seed < 0 seeds nondeterministically.
   * @param random_seed Episode seed, or a negative value for nondeterministic sampling.
   */
  void setGainSamplingSeed(int random_seed) { rng_.seed(random_seed, "graph_manager_gain_model"); }

  /// @brief Randomly assign new gains to all nodes using the loaded gain model.
  void assignNewGains();

  /**
   * @brief Serialize the current graph (nodes, gains, adjacency) to a text file.
   * @param output_graph_filepath Output file path.
   */
  void writeGraphToFile(const std::string& output_graph_filepath);

  /**
   * @brief Generate visualization markers for the graph nodes and optionally edges.
   *
   * Node colors use the complete loaded graph's gain range so their meaning is
   * stable as the perceived graph grows. The current node is highlighted by a
   * separate marker.
   * @param markers Output visualization markers.
   * @param visualize_edges If true, include edge markers.
   */
  void getVisualizationMarkers(VisualizationMarkers* markers, bool visualize_edges);

 private:
  /**
   * @brief Assign a random gain to a single node from the first gain-model region containing it.
   * @param node The graph node whose gain is sampled.
   * @throws std::runtime_error if no gain-model region contains the node.
   */
  void sampleGain(GraphNode* node);

  GraphType graph_;
  Node* current_node_ = nullptr;
  std::unordered_map<const Node*, int> node_ptr_to_original_id_map_;
  std::unordered_map<int, Node*> original_id_to_node_ptr_map_;
  std::unordered_map<int, double> original_id_to_gain_map_;
  std::unordered_map<int, NodeId> original_id_to_node_id_map_;

  GraphData graph_data_;
  double max_gain_ = kDoubleMin;
  double min_gain_ = kDoubleMax;

  std::vector<GainRegion> gain_regions_;

  /// @brief Gain-sampling RNG. Seeded nondeterministically (-1) until setGainSamplingSeed() is called.
  SeededRng rng_{makeSeededEngine(-1, "graph_manager_gain_model"), "graph_manager_gain_model"};
};

}  // namespace open_apero
