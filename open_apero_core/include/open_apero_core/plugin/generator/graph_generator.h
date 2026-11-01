#pragma once

/**
 * @file graph_generator.h
 * @brief Base class for node and edge generation in the planning graph/tree.
 *
 * GraphGenerator extends Generator with edges: connecting samples via
 * RobotModel::planTrajectory and computing traversal costs. Implementations include
 * RRAG (graph) and TreeGenerator's RRT and RRTStar (trees).
 */

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/plugin/generator/generator.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/**
 * @brief Abstract base class for generating planning nodes and connecting edges.
 *
 * Manages collision-free edge construction on top of the sampling/KD-tree machinery inherited
 * from Generator. Subclasses implement the specific graph or tree expansion strategy.
 *
 * The start node is the vertex the robot's state stands on: the owning PlanningLayer's current
 * node, and the source of every search. A tree also calls it its root.
 */
class GraphGenerator : public Generator {
 public:
  /** @brief Constructs a GraphGenerator and binds it to the agent. */
  explicit GraphGenerator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  virtual ~GraphGenerator() = default;

  /** Keeps Generator's KD-tree overload visible next to the graph overload below. */
  using Generator::getVisualization;

  /** @brief Loads edge-side parameters (min/max edge length) from the config map. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Validates the edge lengths: min_edge_length > 0, max_edge_length 0 (no limit) or >= min_edge_length.
   * @param error_message Populated with a description of the first validation failure.
   * @return true if all parameters are valid.
   */
  bool checkParamsValid(std::string* error_message) override;

  /**
   * @brief Returns every node of the structure.
   * @param start_node Unused by the built-in generators.
   * @return Vector of pointers to all nodes.
   */
  virtual std::vector<Node*> getAllNodes(Node* start_node) const = 0;

  /**
   * @brief Returns every edge of the structure.
   * @param start_node Unused by the built-in generators.
   * @return Vector of pointers to all edges.
   */
  virtual std::vector<Edge*> getAllEdges(Node* start_node) const = 0;

  /**
   * @brief Creates the initial planning node from the robot's starting state.
   * @param init_state The initial robot state.
   * @return A pair of the graph and the initial node pointer. The graph is nullptr for a tree generator
   *         (TreeGenerator), whose nodes are linked through TreeNode::parent and TreeNode::children instead.
   */
  virtual std::pair<GraphType*, Node*> generateInitNode(const RobotState& init_state) = 0;

  /**
   * @brief Updates generator state after the robot moves to a new start node.
   * @param previous_start The old start node.
   * @param new_start The new start node.
   */
  virtual void setNewStartNode(Node* previous_start, Node* new_start) = 0;

  /**
   * @brief Connects a newly sampled node cluster to the existing graph with traversable edges.
   * @param node_cluster The node cluster to connect.
   * @param new_edges Output vector of newly created edges.
   * @return true if at least one edge was successfully generated.
   */
  virtual bool generateNewEdges(NodeClusterPtrs& node_cluster, std::vector<Edge*>& new_edges) = 0;

  /**
   * @brief Inserts an intermediate node at a specified point along an existing edge.
   * @param start_node The source node of the edge.
   * @param goal_node The target node of the edge.
   * @param edge The edge to split.
   * @param node_index Index of the intermediate point along the trajectory.
   * @return A pair of the new intermediate node and the edge from start_node to it.
   */
  virtual std::pair<Node*, Edge*> insertIntermediateNode(Node* start_node, Node* goal_node, Edge* edge, int node_index) = 0;

  /**
   * @brief Makes the robot's current state, reached off this structure, the new start node (an
   *        existing node at exactly that pose is adopted).
   *
   * Only @p executed_edge.trajectory.back() is used; the edge is not imported. Never returns
   * nullptr: without a plannable out-edge a graph keeps a detached intermediate start node and a
   * tree starts a fresh single-node root. A graph removes the previous intermediate start node, a
   * tree reparents it under the new one; do not re-seed.
   *
   * @param previous The generator's current start node (already part of this structure).
   * @param executed_edge Edge whose endpoint gives the new node's robot state.
   * @return Raw pointer to the new start node.
   */
  virtual Node* insertIntermediateNodeAtCurrentState(Node* previous, const Edge& executed_edge) = 0;

  /**
   * @brief Gives an intermediate start node its outgoing edges to clusters within reach.
   *
   * Called by the layer after expansion when the start node has no out-edges, so
   * clusters born this cycle connect in the same cycle. No-op for a tree, whose start
   * node is connected by construction.
   */
  virtual void connectStartNode(Node* /*start_node*/) {}

  /**
   * @brief Whether @p node is still a member of this generator's structure, intermediate nodes included.
   *
   * Pure virtual again: Generator's KD-tree default excludes intermediate nodes. Pointer-only
   * check; must not dereference @p node.
   */
  virtual bool containsNode(const Node* node) const = 0;

  /**
   * @brief Re-checks the nodes and edges this cycle's map update can affect, and prunes what became untraversable.
   *
   * Affected: the map region updated since the last cycle meets the robot's body there (isAffectedByMapUpdate()).
   * @param start_node The current start node.
   */
  virtual void updateNodeEdge(Node* start_node) = 0;

  /**
   * @brief Changes the orientation of a node and updates all connected edges.
   * @param node The node to reorient.
   * @param orientation The new orientation quaternion.
   * @return true on success.
   */
  virtual bool setNodeOrientation(Node* node, const Eigen::Quaterniond& orientation) = 0;

  /**
   * @brief Returns whether the generator creates bidirectional edges.
   * @return true for graph-based generators (edges traverse in both directions);
   *         false for tree-based generators (edges are directed from parent to child).
   */
  virtual bool isEdgeBidirectional() const = 0;

  /**
   * @brief Computes and sets the traversal cost on an edge.
   *
   * Forwards to Layer::computeCost, i.e. the agent's cost computer.
   *
   * @param edge The edge to cost.
   * @return true on success.
   */
  virtual bool computeCost(Edge* edge);

  /**
   * @brief Generates visualization markers for the planning graph (nodes + optional edges).
   *
   * @param start_node The current start node.
   * @param markers    Output marker container.
   */
  virtual void getVisualization(Node* start_node, VisualizationMarkers* markers);

 protected:
  /**
   * @brief Generates a collision-free translation edge between distinct positions.
   *
   * Start and goal must have distinct base positions. Use
   * generateRotationEdgeBetweenNodes() for coincident nodes.
   *
   * @param start_node Source node.
   * @param goal_node Target node (distinct position from start_node).
   * @param edge Output edge to populate.
   * @return true if a feasible trajectory was found; false otherwise.
   */
  bool generateEdgeBetweenNodes(Node* start_node, Node* goal_node, Edge* edge);

  /**
   * @brief Generates a rotation-only edge between coincident nodes (see RobotModel::planRotationTrajectory).
   *
   * @param start_node Source node.
   * @param goal_node Target node (same position, different orientation).
   * @param edge Output edge to populate.
   * @return true if a feasible rotation trajectory was found; false otherwise.
   */
  bool generateRotationEdgeBetweenNodes(Node* start_node, Node* goal_node, Edge* edge);

  /**
   * @brief Builds a translation edge by re-aiming a validated seed edge at another node pair.
   *
   * Only the seed's two end rotations are rebuilt. Precondition: the seed edge's endpoints share
   * their positions with @p start_node and @p goal_node.
   *
   * @param seed_edge  Validated edge whose trajectory is reused.
   * @param start_node Source node, supplying the new departure orientation.
   * @param goal_node  Target node, supplying the new arrival orientation.
   * @param edge       Output edge to populate.
   * @return true if the retargeted trajectory was accepted; false otherwise.
   */
  bool buildRetargetedEdge(const Edge& seed_edge, Node* start_node, Node* goal_node, Edge* edge);

  /**
   * @brief Re-validate an existing edge against the current map (in place).
   *
   * Only is_traversable is updated; the trajectory is never changed, since the executor follows
   * it by pointer. Backtrack edges are left untouched (see Edge::is_backtrack).
   *
   * @param edge The edge to refresh (only its is_traversable flag is updated).
   */
  void revalidateEdge(Edge* edge);

  double p_min_edge_length_ = 0.8;    // m, minimum distance of a new sample from its nearest cluster; > 0
  double p_max_edge_length_ = 100.0;  // m (set 0.0 to ignore)
  bool p_visualize_edges_ = false;
};

}  // namespace open_apero
