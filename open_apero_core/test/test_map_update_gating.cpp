/**
 * @file test_map_update_gating.cpp
 * @brief Graph maintenance re-checks a node, an edge or a missing connection only where the map region updated since
 *        the last cycle meets the robot's body bounds.
 *
 * A robot without bounds has every verdict re-checked.
 */

#include <cmath>
#include <cstddef>
#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/geometry_primitives.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/generator/rrt.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "open_apero_core/state/robot_state.h"
#include "rrag_test_stubs.h"

namespace open_apero {
namespace {

void expectVectorNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected) {
  EXPECT_LT((actual - expected).cwiseAbs().maxCoeff(), 1e-9) << "got " << actual.transpose() << ", expected " << expected.transpose();
}

/// Scripts the cube of half-width @p half_width around @p center as the map's updated region.
void setUpdatedRegion(StubMap* map, const Eigen::Vector3d& center, double half_width) {
  map->has_update = true;
  map->updated_min = center - Eigen::Vector3d::Constant(half_width);
  map->updated_max = center + Eigen::Vector3d::Constant(half_width);
}

/// The sensor position halfway along an edge's trajectory.
Eigen::Vector3d midpoint(const Edge& edge) {
  return edge.trajectory[edge.trajectory.size() / 2].getSensorPosition();
}

std::size_t numTraversable(const NodeClusterPtrs& cluster) {
  std::size_t count = 0;
  for (const Node* node : cluster) {
    if (node->is_traversable) {
      ++count;
    }
  }
  return count;
}

/// The single edge joining any node of @p from to any node of @p to, or nullptr.
Edge* edgeBetweenClusters(const NodeClusterPtrs& from, const NodeClusterPtrs& to, const GraphType* graph) {
  for (Node* from_node : from) {
    for (Node* to_node : to) {
      if (Edge* edge = getEdge(from_node, to_node, graph)) {
        return edge;
      }
    }
  }
  return nullptr;
}

TEST(BoxesOverlap, EveryAxisMustOverlapAndTouchingFacesDoNot) {
  const Eigen::Vector3d min = Eigen::Vector3d::Zero();
  const Eigen::Vector3d max = Eigen::Vector3d::Ones();
  EXPECT_TRUE(boxesOverlap(min, max, Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(2.0, 2.0, 2.0)));
  EXPECT_FALSE(boxesOverlap(min, max, Eigen::Vector3d(0.5, 0.5, 1.5), Eigen::Vector3d(2.0, 2.0, 2.0))) << "disjoint in z alone";
  EXPECT_FALSE(boxesOverlap(min, max, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(2.0, 1.0, 1.0))) << "touching faces";
  EXPECT_FALSE(boxesOverlap(min, max, Eigen::Vector3d::Constant(kDoubleNan), max)) << "a NaN corner";
}

TEST(GeometryPrimitiveBounds, SpanEverySphereCylinderAndPrism) {
  GeometryPrimitiveCollection primitives;
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  EXPECT_FALSE(primitives.getBounds(&min, &max));

  primitives.spheres.push_back({Eigen::Vector3d(0.0, 0.0, 1.0), 0.5});
  primitives.cylinders.push_back({Eigen::Vector3d(2.0, 0.0, 0.0), 0.3, 1.5});
  primitives.triangles.push_back(
      {{Eigen::Vector3d(-1.0, -2.0, 0.2), Eigen::Vector3d(0.0, -1.0, 0.2), Eigen::Vector3d(-0.5, -1.5, 0.2)}, 0.4});
  ASSERT_TRUE(primitives.getBounds(&min, &max));
  expectVectorNear(min, Eigen::Vector3d(-1.0, -2.0, 0.0));
  expectVectorNear(max, Eigen::Vector3d(2.3, 0.5, 1.5));
}

TEST(SimpleRobotBodyBounds, SpanEverySphereGrownByItsMarginAtThePlacedState) {
  StubAgent agent;
  StubMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);
  Configurable::ParamMap params{{"collision_sphere_x", "0.5,0.0"},
                                {"collision_sphere_y", "0.0,0.0"},
                                {"collision_sphere_z", "0.5,1.0"},
                                {"collision_sphere_r", "0.2,0.3"},
                                {"collision_sphere_safe_margins", "0.05,0.1"},
                                {"contact_sphere_x", "0.0"},
                                {"contact_sphere_y", "0.0"},
                                {"contact_sphere_z", "-0.1"},
                                {"contact_sphere_r", "0.1"},
                                {"base_in_sensor_position_z", "-1.0"}};
  robot.setupFromParamMap(&params);

  // The base 1 m below the sensor and facing +y, so the sphere 0.5 m ahead of it lies at +y.
  RobotState state;
  robot.placeRobotAtSensorPose(&state, {Eigen::Vector3d(2.0, 3.0, 1.5), quaternionFromYaw(M_PI / 2.0)});

  Eigen::Vector3d min;
  Eigen::Vector3d max;
  ASSERT_TRUE(robot.getBodyBounds(state, &min, &max));
  // Collision spheres at (2, 3.5, 1.0) +- 0.25 and (2, 3, 1.5) +- 0.4, the contact sphere at (2, 3, 0.4) +- 0.1.
  expectVectorNear(min, Eigen::Vector3d(1.6, 2.6, 0.3));
  expectVectorNear(max, Eigen::Vector3d(2.4, 3.75, 1.9));
}

TEST(SimpleRobotBodyBounds, ARobotWithoutSpheresReportsNone) {
  StubAgent agent;
  StubMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);

  Eigen::Vector3d min;
  Eigen::Vector3d max;
  EXPECT_FALSE(robot.getBodyBounds(makeState(robot, Eigen::Vector3d::Zero()), &min, &max));
}

/// A start node at the origin and a robot whose body is the cube of half-width 0.3 m around its sensor.
class MapUpdateGatingTest : public ::testing::Test {
 protected:
  MapUpdateGatingTest() {
    robot_.body_radius = 0.3;
    agent_.provide(&map_, &robot_);
    generator_ = std::make_unique<RefreshProbe>(agent_, Configurable::ParamMap{{"max_edge_length", "6.0"}});
    std::tie(std::ignore, start_node_) = generator_->generateInitNode(makeState(robot_, Eigen::Vector3d::Zero()));
  }

  /// The node half of updateNodeEdge().
  void refreshNodes(Node* start_node) { generator_->updateNodeTraversability(start_node); }

  /// The one edge from @p from to @p to.
  Edge* connect(NodeClusterPtrs from, NodeClusterPtrs to) {
    const std::vector<Edge*> edges = generator_->generateEdgesFromClusterToCluster(from, to);
    EXPECT_EQ(edges.size(), 1u);
    return edges.empty() ? nullptr : edges.front();
  }

  StubAgent agent_;
  StubMap map_{agent_};
  StationingRobot robot_{agent_};
  std::unique_ptr<RefreshProbe> generator_;
  Node* start_node_ = nullptr;
};

TEST_F(MapUpdateGatingTest, OnlyTheClusterTheUpdateReachesIsRefreshed) {
  const Eigen::Vector3d reached(2.0, 0.0, 0.0);
  const Eigen::Vector3d missed(-2.0, 0.0, 0.0);
  NodeClusterPtrs reached_cluster = generator_->placeCluster(reached);
  NodeClusterPtrs missed_cluster = generator_->placeCluster(missed);
  robot_.forbidden_positions = {reached, missed};

  refreshNodes(start_node_);
  EXPECT_EQ(numTraversable(reached_cluster), reached_cluster.size()) << "no map update, no refresh";
  EXPECT_EQ(numTraversable(missed_cluster), missed_cluster.size());

  setUpdatedRegion(&map_, reached, 0.1);
  refreshNodes(start_node_);
  EXPECT_EQ(numTraversable(reached_cluster), 0u);
  EXPECT_EQ(numTraversable(missed_cluster), missed_cluster.size()) << "outside the update the verdict stands";
}

// The occupied cluster is spared by identity, not by its flags, so an update elsewhere neither
// rewrites the start node's verdict nor lets the prune take the vertex the robot stands on.
TEST_F(MapUpdateGatingTest, TheOccupiedClusterSurvivesThePruneWhereverTheUpdateIs) {
  NodeClusterPtrs cluster = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  Node* start_node = cluster.front();
  for (Node* node : cluster) {
    node->is_traversable = false;
  }
  setUpdatedRegion(&map_, Eigen::Vector3d(-5.0, 0.0, 0.0), 0.1);

  refreshNodes(start_node);
  generator_->removeUntraversableNodeClusters(start_node);

  EXPECT_FALSE(start_node->is_traversable) << "the verdict is not rewritten for the start node";
  for (Node* node : cluster) {
    EXPECT_TRUE(generator_->containsNode(node));
  }
  EXPECT_FALSE(agent_.warnings.empty()) << "a demoted start node is reported";
  EXPECT_TRUE(agent_.errors.empty()) << "but it is no error";
}

TEST_F(MapUpdateGatingTest, OnlyTheEdgeTheUpdateReachesIsRevalidatedBesidesTheStartNodesOwn) {
  NodeClusterPtrs a = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  NodeClusterPtrs b = generator_->placeCluster(Eigen::Vector3d(5.0, 0.0, 0.0));
  NodeClusterPtrs c = generator_->placeCluster(Eigen::Vector3d(2.0, 4.0, 0.0));
  NodeClusterPtrs d = generator_->placeCluster(Eigen::Vector3d(5.0, 4.0, 0.0));
  Edge* reached = connect(a, b);
  Edge* missed = connect(c, d);
  Edge* start_edge = connect({start_node_}, a);
  ASSERT_TRUE(reached && missed && start_edge);
  robot_.blocked_positions = {midpoint(*reached), midpoint(*missed), midpoint(*start_edge)};
  setUpdatedRegion(&map_, midpoint(*reached), 0.1);

  generator_->updateEdgeTraversability(start_node_);

  EXPECT_FALSE(reached->is_traversable);
  EXPECT_TRUE(missed->is_traversable) << "outside the update the verdict stands";
  EXPECT_FALSE(start_edge->is_traversable) << "the start node's edges are revalidated regardless";
}

TEST_F(MapUpdateGatingTest, AMissingEdgeReturnsOnceAnUpdateReachesTheSegment) {
  NodeClusterPtrs a = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  NodeClusterPtrs b = generator_->placeCluster(Eigen::Vector3d(5.0, 0.0, 0.0));
  Edge* edge = connect(a, b);
  ASSERT_NE(edge, nullptr);
  const Eigen::Vector3d blocked = midpoint(*edge);
  robot_.blocked_positions = {blocked};
  setUpdatedRegion(&map_, blocked, 0.1);
  generator_->updateEdgeTraversability(start_node_);
  generator_->removeUntraversableEdges();
  ASSERT_EQ(edgeBetweenClusters(a, b, generator_->graph()), nullptr);

  robot_.blocked_positions.clear();
  setUpdatedRegion(&map_, Eigen::Vector3d(3.5, 5.0, 0.0), 0.1);
  generator_->generateMissingEdges(start_node_);
  EXPECT_EQ(edgeBetweenClusters(a, b, generator_->graph()), nullptr) << "an update elsewhere leaves the connection missing";

  setUpdatedRegion(&map_, blocked, 0.1);
  generator_->generateMissingEdges(start_node_);
  EXPECT_NE(edgeBetweenClusters(a, b, generator_->graph()), nullptr);
  EXPECT_NE(edgeBetweenClusters(b, a, generator_->graph()), nullptr);
}

TEST_F(MapUpdateGatingTest, ARobotWithoutBodyBoundsHasEveryClusterRefreshed) {
  robot_.body_radius.reset();
  const Eigen::Vector3d near(2.0, 0.0, 0.0);
  const Eigen::Vector3d far(-2.0, 0.0, 0.0);
  NodeClusterPtrs near_cluster = generator_->placeCluster(near);
  NodeClusterPtrs far_cluster = generator_->placeCluster(far);
  robot_.forbidden_positions = {near, far};

  refreshNodes(start_node_);

  EXPECT_EQ(numTraversable(near_cluster), 0u);
  EXPECT_EQ(numTraversable(far_cluster), 0u);
}

TEST_F(MapUpdateGatingTest, AnUpdateBesideTheCorridorDoesNotRetryAMissingEdge) {
  NodeClusterPtrs a = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  NodeClusterPtrs b = generator_->placeCluster(Eigen::Vector3d(5.0, 0.0, 0.0));
  Edge* edge = connect(a, b);
  ASSERT_NE(edge, nullptr);
  robot_.blocked_positions = {midpoint(*edge)};
  setUpdatedRegion(&map_, midpoint(*edge), 0.1);
  generator_->updateEdgeTraversability(start_node_);
  generator_->removeUntraversableEdges();
  ASSERT_EQ(edgeBetweenClusters(a, b, generator_->graph()), nullptr);
  robot_.blocked_positions.clear();

  // Beside the corridor, outside the box over the endpoint bodies.
  setUpdatedRegion(&map_, Eigen::Vector3d(3.5, 1.2, 0.0), 0.1);
  generator_->generateMissingEdges(start_node_);
  EXPECT_EQ(edgeBetweenClusters(a, b, generator_->graph()), nullptr);
}

/// Prices edges by their length in states; there is no owning layer to ask.
class RrtProbe : public generator::RRT {
 public:
  explicit RrtProbe(AgentInterface& agent) : RRT(agent) {
    Configurable::ParamMap params{{"param_namespace", "/test_rrt"}};
    setupFromParamMap(&params);
  }

  bool computeCost(Edge* edge) override {
    edge->cost = static_cast<double>(edge->trajectory.size());
    return true;
  }
};

TEST(TreeMapUpdateGating, OnlyTheEdgeTheUpdateReachesIsRevalidated) {
  StubAgent agent;
  StubMap map(agent);
  StationingRobot robot(agent);
  robot.body_radius = 0.3;
  agent.provide(&map, &robot);
  RrtProbe rrt(agent);
  TreeNode* root = rrt.generateInitNode(makeState(robot, Eigen::Vector3d::Zero())).second->as<TreeNode>();

  const auto add_child = [&robot, root](const Eigen::Vector3d& position) {
    TreeNode* child = root->spawnChild();
    child->robot_state = makeState(robot, position);
    child->edge.trajectory = robot.steer(root->robot_state, child->robot_state);
    child->edge.is_traversable = true;
    return child;
  };
  TreeNode* reached = add_child(Eigen::Vector3d(2.0, 0.0, 0.0));
  TreeNode* missed = add_child(Eigen::Vector3d(-3.0, 0.0, 0.0));
  const Eigen::Vector3d reached_midpoint = midpoint(reached->edge);
  robot.blocked_positions = {reached_midpoint, midpoint(missed->edge)};
  setUpdatedRegion(&map, reached_midpoint, 0.1);

  rrt.updateNodeEdge(root);

  ASSERT_EQ(root->children.size(), 1u) << "the edge the update reaches is cut";
  EXPECT_EQ(root->children.front().get(), missed) << "outside the update the verdict stands";
  EXPECT_TRUE(missed->edge.is_traversable);
}

}  // namespace
}  // namespace open_apero
