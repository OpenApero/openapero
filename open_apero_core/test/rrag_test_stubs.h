#pragma once

/**
 * @file rrag_test_stubs.h
 * @brief The agent, map, robot and generator probe the RRAG-level tests drive a graph with.
 *
 * Shared by test_stationing_positions.cpp, test_detached_intermediate_start_node.cpp,
 * test_planning_layer_start_node.cpp, test_rrag_cluster_construction.cpp and test_map_update_gating.cpp.
 * The robot answers traversability from scripted lists: forbidden_positions refuses stationing,
 * blocked_positions and blocked_orientations make a state untraversable; with shares_gain_anchor it
 * gives every member of a new cluster one shared GainAnchor. The map reports a scripted updated region.
 */

#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/graph/adjacency_list.hpp>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/generator/rrag.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "open_apero_core/state/robot_state.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// Reports updated_min/updated_max as the updated region while has_update, and no region otherwise.
class StubMap : public Map {
 public:
  explicit StubMap(AgentInterface& agent) : Map(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}

  bool getUpdatedMapRegion(Eigen::Vector3d* updated_map_min, Eigen::Vector3d* updated_map_max) override {
    if (!has_update) {
      return false;
    }
    *updated_map_min = updated_min;
    *updated_map_max = updated_max;
    return true;
  }

  bool has_update = false;
  Eigen::Vector3d updated_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d updated_max = Eigen::Vector3d::Zero();
};

/// ConfigAgent with no budget and seed 5; it keeps every log line so a test can assert on what a decline reported.
class StubAgent : public ConfigAgent {
 public:
  StubAgent() {
    setCostBudget(0.0);
    setRandomSeed(5);
  }
};

/// A robot that refuses to station at scripted positions and cannot stand at all at
/// others, standing in for a stationing clearance rule and blocked regions. Every stationing
/// query is recorded in stationing_queries.
class StationingRobot : public robot_model::SimpleRobot {
 public:
  explicit StationingRobot(AgentInterface& agent) : robot_model::SimpleRobot(agent) {
    Configurable::ParamMap params{{"num_orientation_candidates", "4"}, {"safe_radius", "0.0"}};
    setupFromParamMap(&params);
  }

  bool isStationingPositionAdmissible(const Eigen::Vector3d& position, Map* /*map*/) const override {
    stationing_queries.push_back(position);
    for (const auto& forbidden : forbidden_positions) {
      if (position.isApprox(forbidden)) {
        return false;
      }
    }
    return true;
  }

  /// The base transform is the identity offset of the sensor here, so a blocked position
  /// blocks the state whichever frame a caller thinks in.
  Traversability getStateTraversabilityRaw(const RobotState& state, Map* /*map*/) const override {
    for (const auto& blocked : blocked_positions) {
      if (state.getSensorPosition().isApprox(blocked)) {
        return Traversability::kUntraversable;
      }
    }
    for (const auto& blocked : blocked_orientations) {
      if (state.getSensorOrientation().angularDistance(blocked) < 1e-6) {
        return Traversability::kUntraversable;
      }
    }
    return Traversability::kTraversable;
  }

  /// A blocked orientation makes checkOrientations judge every candidate on its own.
  bool isTraversabilityOrientationRelated() const override {
    return !blocked_orientations.empty() || robot_model::SimpleRobot::isTraversabilityOrientationRelated();
  }

  /// With body_radius, the body is the cube of that half-width around the sensor position; without, no bounds.
  bool getBodyBounds(const RobotState& state, Eigen::Vector3d* min, Eigen::Vector3d* max) const override {
    if (!body_radius) {
      return robot_model::SimpleRobot::getBodyBounds(state, min, max);
    }
    *min = state.getSensorPosition() - Eigen::Vector3d::Constant(*body_radius);
    *max = state.getSensorPosition() + Eigen::Vector3d::Constant(*body_radius);
    return true;
  }

  /// One attachSharedGainAnchors call: the position and the cluster it was handed.
  struct SharedAnchorCall {
    Eigen::Vector3d position;
    NodeClusterPtrs cluster;
  };

  /// Records every call in shared_anchor_calls; with shares_gain_anchor, the members share one anchor.
  void attachSharedGainAnchors(const Eigen::Vector3d& position, const NodeClusterPtrs& cluster) const override {
    shared_anchor_calls.push_back({position, cluster});
    if (!shares_gain_anchor) {
      return;
    }
    const GainAnchorPtr anchor = std::make_shared<GainAnchor>(0.0);
    for (Node* node : cluster) {
      node->gain_anchors.push_back(anchor);
    }
  }

  std::vector<Eigen::Vector3d> forbidden_positions;
  std::vector<Eigen::Vector3d> blocked_positions;
  std::vector<Eigen::Quaterniond> blocked_orientations;
  std::optional<double> body_radius;
  bool shares_gain_anchor = false;
  mutable std::vector<Eigen::Vector3d> stationing_queries;
  mutable std::vector<SharedAnchorCall> shared_anchor_calls;
};

/// Exposes RRAG's protected maintenance passes and lets a test place clusters directly,
/// so a rule can be pinned without driving a full sampling cycle or an owning layer.
class RefreshProbe : public generator::RRAG {
 public:
  explicit RefreshProbe(AgentInterface& agent) : RefreshProbe(agent, Configurable::ParamMap{}) {}

  /// @p extra_params overrides the defaults below, e.g. to bring max_edge_length within
  /// reach of a test's cluster spacing.
  RefreshProbe(AgentInterface& agent, const Configurable::ParamMap& extra_params) : generator::RRAG(agent) {
    Configurable::ParamMap params{{"param_namespace", "/test_stationing"}};
    for (const auto& entry : extra_params) {
      params[entry.first] = entry.second;
    }
    setupFromParamMap(&params);
  }

  using generator::RRAG::buildClusterAt;
  using generator::RRAG::generateEdgesFromClusterToCluster;
  using generator::RRAG::generateMissingEdges;
  using generator::RRAG::removeDisconnectedNodeClusters;
  using generator::RRAG::removeUntraversableEdges;
  using generator::RRAG::removeUntraversableNodeClusters;
  using generator::RRAG::updateEdgeTraversability;
  using generator::RRAG::updateNodeTraversability;

  // Every edge builder costs its result through the owning layer; there is none here.
  bool computeCost(Edge* edge) override {
    edge->cost = static_cast<double>(edge->trajectory.size());
    return true;
  }

  GraphType* graph() { return graph_.get(); }

  std::size_t numVertices() const { return boost::num_vertices(*graph_); }

  std::size_t outDegree(const Node* node) const { return boost::out_degree(getNodeId(node, graph_.get()), *graph_); }

  std::size_t numEdges() const { return boost::num_edges(*graph_); }

  /// Is this node in the spatial index the refresh pass iterates?
  bool isIndexed(const Node* node) const {
    for (const auto& cluster : kd_tree_data_.nodes) {
      for (const Node* indexed : cluster) {
        if (indexed == node) {
          return true;
        }
      }
    }
    return false;
  }

  /// A full candidate cluster at @p position, in the graph and in the spatial index. The
  /// cluster is complete because the intra-cluster edge builder requires one node per
  /// orientation candidate.
  NodeClusterPtrs placeCluster(const Eigen::Vector3d& position) {
    std::vector<Eigen::Quaterniond> orientations;
    agent_.getRobot().sampleOrientations(position, &orientations);

    NodeClusterPtrs cluster;
    for (const auto& orientation : orientations) {
      auto node = std::make_shared<GraphNode>();
      agent_.getRobot().placeRobotAtSensorPose(&node->robot_state, {position, orientation});
      node->is_traversable = true;
      boost::add_vertex(node, *graph_);
      cluster.push_back(node.get());
    }
    indexNodeCluster(cluster);
    return cluster;
  }
};

inline RobotState makeState(const RobotModel& robot, const Eigen::Vector3d& position, double yaw = 0.0) {
  RobotState state;
  robot.placeRobotAtSensorPose(&state, {position, quaternionFromYaw(yaw)});
  return state;
}

}  // namespace
}  // namespace open_apero
