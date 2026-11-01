/**
 * @file test_gain_suppression_search.cpp
 * @brief Tests that both beam searches suppress node-cluster gains once setupRaw() sets a gain suppression radius.
 *
 * Checked by behavior: the radius must reach the search and change what a path gains. A negative
 * gain_suppression_radius parameter is rejected.
 */

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/plugin/path_planner/graph_depth_wise_beam_search.h"
#include "open_apero_core/plugin/path_planner/graph_node_wise_beam_search.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/plugin.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/**
 * @brief A corridor start(0) -> near(0.7, gain 1) -> doorway(2.1, gain 10) -> far(3.8, gain 10) along x.
 *
 * Edge costs are the distances between the nodes; each node is its own cluster. With a 1.5 m radius the
 * doorway suppresses the near node (1.4 m away), while the doorway and far (1.7 m apart) both keep their gain.
 */
class GainSuppressionSearchTest : public ::testing::Test {
 protected:
  static constexpr double kRadius = 1.5;
  static constexpr int kBeamDepth = 4;
  static constexpr int kBeamWidth = 4;

  void SetUp() override {
    start_ = addNode(0.0, 0.0);
    near_ = addNode(0.7, 1.0);
    doorway_ = addNode(2.1, 10.0);
    far_ = addNode(3.8, 10.0);

    addEdge(start_, near_);
    addEdge(near_, doorway_);
    addEdge(doorway_, far_);
  }

  NodeId addNode(double x, double gain) {
    auto node = std::make_shared<GraphNode>();
    node->is_traversable = true;
    node->robot_state.setSensorPosition(Eigen::Vector3d(x, 0.0, 0.0));
    if (gain > 0.0) {
      node->gain_anchors = {std::make_shared<GainAnchor>(gain)};
    }
    return boost::add_vertex(node, graph_);
  }

  Node* node(NodeId id) const { return graph_[id].get(); }

  void addEdge(NodeId from, NodeId to) {
    auto edge = std::make_shared<Edge>();
    edge->cost = (node(to)->robot_state.getSensorPosition() - node(from)->robot_state.getSensorPosition()).norm();
    edge->is_traversable = true;
    boost::add_edge(from, to, edge, graph_);
  }

  /// @brief The path's cost, summed edge by edge from the start as a search does.
  double costAlong(const std::vector<Node*>& path) const {
    double cost = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      cost += (path[i]->robot_state.getSensorPosition() - path[i - 1]->robot_state.getSensorPosition()).norm();
    }
    return cost;
  }

  /// @brief The search returned the whole corridor, scored kPathRatio with @p expected_gain.
  void expectCorridor(const std::pair<std::vector<Node*>, double>& plan, double expected_gain) {
    const std::vector<Node*> corridor{node(start_), node(near_), node(doorway_), node(far_)};
    ASSERT_EQ(plan.first, corridor);
    EXPECT_DOUBLE_EQ(plan.second, expected_gain / costAlong(corridor));
  }

  /// @brief With the radius the near node is a waypoint worth 0; without it, it adds its 1.
  template <typename Search>
  void expectTheNearNodeIsAWaypointOnlyWithTheRadius() {
    Search with_radius(agent_);
    with_radius.setupRaw(Objective::kPathRatio, kBeamDepth, kBeamWidth, /*check_previous_path=*/false, /*is_gain_correlated=*/true,
                         kRadius);
    expectCorridor(with_radius.planPath(node(start_), &graph_, nullptr), 20.0);
    EXPECT_TRUE(node(near_)->is_gain_suppressed);
    EXPECT_FALSE(node(doorway_)->is_gain_suppressed);
    EXPECT_FALSE(node(far_)->is_gain_suppressed);

    Search without_radius(agent_);
    without_radius.setupRaw(Objective::kPathRatio, kBeamDepth, kBeamWidth, /*check_previous_path=*/false, /*is_gain_correlated=*/true, 0.0);
    expectCorridor(without_radius.planPath(node(start_), &graph_, nullptr), 21.0);
    EXPECT_FALSE(node(near_)->is_gain_suppressed);
  }

  /// @brief A later setupRaw() without the trailing argument switches suppression off again.
  template <typename Search>
  void expectOmittingTheRadiusSuppressesNothing() {
    Search search(agent_);
    search.setupRaw(Objective::kPathRatio, kBeamDepth, kBeamWidth, /*check_previous_path=*/false, /*is_gain_correlated=*/true, kRadius);
    expectCorridor(search.planPath(node(start_), &graph_, nullptr), 20.0);

    search.setupRaw(Objective::kPathRatio, kBeamDepth, kBeamWidth, /*check_previous_path=*/false, /*is_gain_correlated=*/true);
    expectCorridor(search.planPath(node(start_), &graph_, nullptr), 21.0);
  }

  SearchOnlyAgent agent_;
  GraphType graph_;
  NodeId start_ = kInvalidNodeId;
  NodeId near_ = kInvalidNodeId;
  NodeId doorway_ = kInvalidNodeId;
  NodeId far_ = kInvalidNodeId;
};

TEST_F(GainSuppressionSearchTest, NodeWiseBeamSearchTreatsTheNearNodeAsAWaypointOnlyWithTheRadius) {
  expectTheNearNodeIsAWaypointOnlyWithTheRadius<path_planner::GraphNodeWiseBeamSearch>();
}

TEST_F(GainSuppressionSearchTest, DepthWiseBeamSearchTreatsTheNearNodeAsAWaypointOnlyWithTheRadius) {
  expectTheNearNodeIsAWaypointOnlyWithTheRadius<path_planner::GraphDepthWiseBeamSearch>();
}

TEST_F(GainSuppressionSearchTest, NodeWiseBeamSearchSuppressesNothingOnceSetupRawOmitsTheRadius) {
  expectOmittingTheRadiusSuppressesNothing<path_planner::GraphNodeWiseBeamSearch>();
}

TEST_F(GainSuppressionSearchTest, DepthWiseBeamSearchSuppressesNothingOnceSetupRawOmitsTheRadius) {
  expectOmittingTheRadiusSuppressesNothing<path_planner::GraphDepthWiseBeamSearch>();
}

TEST_F(GainSuppressionSearchTest, ANegativeGainSuppressionRadiusIsRejected) {
  const auto setUpWith = [this](const std::string& radius) {
    Configurable::ParamMap params{{"param_namespace", "/nbs"}, {"objective", "path_ratio"}, {"gain_suppression_radius", radius}};
    path_planner::GraphNodeWiseBeamSearch search(agent_);
    search.setupFromParamMap(&params);
    search.assureParamsValid();
  };

  EXPECT_NO_THROW(setUpWith("0"));
  EXPECT_NO_THROW(setUpWith("1.5"));
  EXPECT_THROW(setUpWith("-0.5"), std::invalid_argument);
}

}  // namespace
}  // namespace open_apero
