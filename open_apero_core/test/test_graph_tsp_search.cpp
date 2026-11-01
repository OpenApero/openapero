/**
 * @file test_graph_tsp_search.cpp
 * @brief GraphTspSearch: optional objective, cost_scale validation, sub-unit cost resolution, and
 *        candidates the graph cannot route between.
 *
 * The routing solver sees integer costs. At the default cost_scale of 1 sub-unit differences are
 * truncated away; cost_scale 1000 keeps them, which can reorder the tour.
 */

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <gtest/gtest.h>

#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/path_planner/graph_node_wise_beam_search.h"
#include "open_apero_core/plugin/path_planner/graph_tsp_search.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/plugin.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/**
 * Start S and two equally valuable nodes A and B, joined by directed edges. Touring S-A-B costs
 * 0.9 + 0.9 = 1.8 and S-B-A 1.0 + 0.5 = 1.5; truncated to integers the first costs 0 and the second 1.
 */
class TwoCandidateGraph {
 public:
  TwoCandidateGraph() {
    start = addNode(0.0);
    a = addNode(10.0);
    b = addNode(10.0);
    addEdge(start, a, 0.9);
    addEdge(a, b, 0.9);
    addEdge(start, b, 1.0);
    addEdge(b, a, 0.5);
  }

  GraphType graph;
  Node* start = nullptr;
  Node* a = nullptr;
  Node* b = nullptr;

 private:
  Node* addNode(double gain) {
    auto node = std::make_shared<GraphNode>();
    node->gain_anchors = {std::make_shared<GainAnchor>(gain)};
    boost::add_vertex(node, graph);
    return node.get();
  }

  void addEdge(Node* from, Node* to, double cost) {
    auto edge = std::make_shared<Edge>();
    edge->cost = cost;
    edge->is_traversable = true;
    boost::add_edge(getNodeId(from, &graph), getNodeId(to, &graph), edge, graph);
  }
};

class GraphTspSearchTest : public ::testing::Test {
 protected:
  /// A planner set up from @p params, validated the way the factory validates it.
  std::unique_ptr<path_planner::GraphTspSearch> makePlanner(Configurable::ParamMap params) {
    params.emplace("param_namespace", "/tsp");
    auto planner = std::make_unique<path_planner::GraphTspSearch>(agent_);
    planner->setupFromParamMap(&params);
    planner->assureParamsValid();
    return planner;
  }

  ConfigAgent agent_;
};

TEST_F(GraphTspSearchTest, ObjectiveIsOptional) {
  EXPECT_NO_THROW(makePlanner({}));
  EXPECT_NO_THROW(makePlanner({{"objective", "path_ratio"}}));
  EXPECT_THROW(makePlanner({{"objective", "no_such_objective"}}), std::runtime_error);
}

TEST_F(GraphTspSearchTest, OtherPlannersStillRequireAnObjective) {
  path_planner::GraphNodeWiseBeamSearch beam_search(agent_);
  Configurable::ParamMap params{{"param_namespace", "/nbs"}};
  EXPECT_THROW(beam_search.setupFromParamMap(&params), std::runtime_error);
}

TEST_F(GraphTspSearchTest, CostScaleMustBePositiveAndFinite) {
  EXPECT_NO_THROW(makePlanner({{"cost_scale", "1000"}}));
  EXPECT_THROW(makePlanner({{"cost_scale", "0"}}), std::invalid_argument);
  EXPECT_THROW(makePlanner({{"cost_scale", "-1"}}), std::invalid_argument);
}

TEST_F(GraphTspSearchTest, DefaultScaleTruncatesSubUnitCosts) {
  TwoCandidateGraph fixture;
  auto planner = makePlanner({});
  const std::vector<Node*> path = planner->planPath(fixture.start, &fixture.graph, nullptr).first;
  // S-A-B truncates to 0 and wins, although S-B-A is cheaper.
  const std::vector<Node*> expected{fixture.start, fixture.a, fixture.b};
  EXPECT_EQ(path, expected);
}

TEST_F(GraphTspSearchTest, MillisecondScaleResolvesSubUnitCosts) {
  TwoCandidateGraph fixture;
  auto planner = makePlanner({{"cost_scale", "1000"}});
  const std::vector<Node*> path = planner->planPath(fixture.start, &fixture.graph, nullptr).first;
  const std::vector<Node*> expected{fixture.start, fixture.b, fixture.a};
  EXPECT_EQ(path, expected);
}

TEST_F(GraphTspSearchTest, SetupRawKeepsTheUnitScale) {
  TwoCandidateGraph fixture;
  path_planner::GraphTspSearch planner(agent_);
  planner.setupRaw(Objective::kPathGain, 1.0);
  const std::vector<Node*> expected{fixture.start, fixture.a, fixture.b};
  EXPECT_EQ(planner.planPath(fixture.start, &fixture.graph, nullptr).first, expected);
}

/// A start and nodes reached from it only by the given one-way edges.
struct OneWayGraph {
  GraphType graph;

  Node* addNode(double gain) {
    auto node = std::make_shared<GraphNode>();
    node->gain_anchors = {std::make_shared<GainAnchor>(gain)};
    boost::add_vertex(node, graph);
    return node.get();
  }

  void addEdge(Node* from, Node* to, double cost) {
    auto edge = std::make_shared<Edge>();
    edge->cost = cost;
    edge->is_traversable = true;
    boost::add_edge(getNodeId(from, &graph), getNodeId(to, &graph), edge, graph);
  }
};

TEST_F(GraphTspSearchTest, ACandidateTheStartCannotReachIsDropped) {
  OneWayGraph g;
  Node* start = g.addNode(0.0);
  Node* reachable = g.addNode(5.0);
  g.addNode(10.0);  // isolated, the highest gain
  g.addEdge(start, reachable, 1.0);
  g.addEdge(reachable, start, 1.0);
  path_planner::GraphTspSearch planner(agent_);
  planner.setupRaw(Objective::kPathGain, 1.0);

  std::vector<Node*> path;
  ASSERT_NO_THROW(path = planner.planPath(start, &g.graph, nullptr).first);
  EXPECT_EQ(path, (std::vector<Node*>{start, reachable}));
}

TEST_F(GraphTspSearchTest, TheTourEndsAtTheFirstLegTheGraphCannotRoute) {
  // Two dead ends, each reached from the start, neither from the other.
  OneWayGraph g;
  Node* start = g.addNode(0.0);
  Node* a = g.addNode(5.0);
  Node* b = g.addNode(5.0);
  g.addEdge(start, a, 1.0);
  g.addEdge(start, b, 1.0);
  path_planner::GraphTspSearch planner(agent_);
  planner.setupRaw(Objective::kPathGain, 1.0);

  std::vector<Node*> path;
  ASSERT_NO_THROW(path = planner.planPath(start, &g.graph, nullptr).first);
  ASSERT_EQ(path.size(), 2u);
  EXPECT_EQ(path.front(), start);
  EXPECT_TRUE(path.back() == a || path.back() == b);
}

}  // namespace
}  // namespace open_apero
