/**
 * @file test_stationing_positions.cpp
 * @brief Where the robot may be stationed, as opposed to merely driven through.
 *
 * RobotModel::isStationingPositionAdmissible must hold both in checkOrientations (node creation)
 * and in the traversability refresh of existing nodes; the default admits everywhere.
 */

#include <cstddef>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "open_apero_core/state/robot_state.h"
#include "rrag_test_stubs.h"

namespace open_apero {
namespace {

TEST(StationingPositions, DefaultRobotStationsAnywhere) {
  StubAgent agent;
  StubMap map(agent);
  robot_model::SimpleRobot robot(agent);
  agent.provide(&map, &robot);

  EXPECT_TRUE(robot.isStationingPositionAdmissible(Eigen::Vector3d(7.0, -3.0, 1.0), &map));
}

// The creation-time gate. Callers build one node per candidate whenever any is
// traversable and read the states either way, so a rejected position must still
// come back with usable states -- just no traversable candidate among them.
TEST(StationingPositions, CheckOrientationsRejectsEveryCandidateAtAForbiddenPosition) {
  StubAgent agent;
  StubMap map(agent);
  StationingRobot robot(agent);
  agent.provide(&map, &robot);
  const Eigen::Vector3d forbidden(1.0, 2.0, 0.0);
  robot.forbidden_positions.push_back(forbidden);

  std::vector<Eigen::Quaterniond> orientations;
  robot.sampleOrientations(forbidden, &orientations);
  std::vector<RobotState> states;
  std::vector<bool> is_traversable;
  robot.checkOrientations(forbidden, orientations, &states, &is_traversable);

  ASSERT_EQ(states.size(), orientations.size());
  ASSERT_EQ(is_traversable.size(), orientations.size());
  for (size_t i = 0; i < states.size(); ++i) {
    EXPECT_FALSE(is_traversable[i]);
    EXPECT_TRUE(states[i].getSensorPosition().isApprox(forbidden));
  }
}

TEST(StationingPositions, CheckOrientationsIsUnchangedAtAnAdmissiblePosition) {
  StubAgent agent;
  StubMap map(agent);
  StationingRobot robot(agent);
  agent.provide(&map, &robot);
  const Eigen::Vector3d position(1.0, 2.0, 0.0);
  robot.forbidden_positions.push_back(Eigen::Vector3d(9.0, 9.0, 0.0));

  std::vector<Eigen::Quaterniond> orientations;
  robot.sampleOrientations(position, &orientations);
  std::vector<RobotState> states;
  std::vector<bool> is_traversable;
  robot.checkOrientations(position, orientations, &states, &is_traversable);

  ASSERT_EQ(is_traversable.size(), orientations.size());
  for (size_t i = 0; i < is_traversable.size(); ++i) {
    EXPECT_TRUE(is_traversable[i]);
  }
}

// Creation-time filtering alone would not hold: the map keeps changing, and a
// position that was a fine place to stop can be revealed as a cramped one. Such
// a node has to stop being selectable.
TEST(StationingPositions, RefreshDemotesAClusterThatBecameInadmissible) {
  StubAgent agent;
  StubMap map(agent);
  StationingRobot robot(agent);
  agent.provide(&map, &robot);
  RefreshProbe generator(agent);
  Node* start_node = nullptr;
  std::tie(std::ignore, start_node) = generator.generateInitNode(makeState(robot, Eigen::Vector3d::Zero()));

  const Eigen::Vector3d position(2.0, 0.0, 0.0);
  NodeClusterPtrs cluster = generator.placeCluster(position);
  generator.updateNodeTraversability(start_node);
  for (Node* node : cluster) {
    ASSERT_TRUE(node->is_traversable) << "fixture must start traversable";
  }

  robot.forbidden_positions.push_back(position);
  generator.updateNodeTraversability(start_node);

  for (Node* node : cluster) {
    EXPECT_FALSE(node->is_traversable);
  }
}

// The refresh judges the occupied node like any other; sparing it is the prune's job, by identity,
// and only while the robot stands there. The start node must be an indexed cluster member:
// generateInitNode's intermediate init_node is never visited by the refresh.
TEST(StationingPositions, RefreshDemotesTheOccupiedNodeAndThePruneSparesItsClusterUntilTheRobotLeaves) {
  StubAgent agent;
  StubMap map(agent);
  StationingRobot robot(agent);
  agent.provide(&map, &robot);
  RefreshProbe generator(agent);
  Node* init_node = nullptr;
  std::tie(std::ignore, init_node) = generator.generateInitNode(makeState(robot, Eigen::Vector3d::Zero()));

  const Eigen::Vector3d occupied(2.0, 0.0, 0.0);
  NodeClusterPtrs cluster = generator.placeCluster(occupied);
  Node* start_node = cluster.front();
  ASSERT_TRUE(generator.isIndexed(start_node)) << "the refresh must actually visit this node";

  robot.forbidden_positions.push_back(occupied);
  generator.updateNodeTraversability(start_node);

  for (const Node* node : cluster) {
    EXPECT_FALSE(node->is_traversable) << "the occupied node's verdict is as honest as its neighbours'";
  }
  EXPECT_FALSE(agent.warnings.empty()) << "the demoted init_node is reported";
  EXPECT_TRUE(agent.errors.empty()) << "but it is no error: the init_node is kept";

  generator.removeUntraversableNodeClusters(start_node);
  for (const Node* node : cluster) {
    EXPECT_TRUE(generator.containsNode(node)) << "spared while the robot stands there";
  }

  generator.removeUntraversableNodeClusters(init_node);
  for (const Node* node : cluster) {
    EXPECT_FALSE(generator.containsNode(node)) << "pruned once the robot has moved on";
  }
}

// Refresh runs the cluster's stored orientations back through checkOrientations, so
// it must not disturb the states it re-derives -- placement is idempotent for a
// stored state, and the refresh depends on that.
TEST(StationingPositions, RefreshLeavesNodeStatesUnchanged) {
  StubAgent agent;
  StubMap map(agent);
  StationingRobot robot(agent);
  agent.provide(&map, &robot);
  RefreshProbe generator(agent);
  Node* start_node = nullptr;
  std::tie(std::ignore, start_node) = generator.generateInitNode(makeState(robot, Eigen::Vector3d::Zero()));

  NodeClusterPtrs cluster = generator.placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  std::vector<RobotState> before;
  for (const Node* node : cluster) {
    before.push_back(node->robot_state);
  }

  generator.updateNodeTraversability(start_node);

  for (size_t i = 0; i < cluster.size(); ++i) {
    EXPECT_TRUE(cluster[i]->robot_state.getSensorPosition().isApprox(before[i].getSensorPosition()));
    EXPECT_TRUE(cluster[i]->robot_state.getSensorTransform().orientation.isApprox(before[i].getSensorTransform().orientation));
    EXPECT_TRUE(cluster[i]->robot_state.getBasePosition().isApprox(before[i].getBasePosition()));
  }
}

// The bootstrap exemption is named at the call site rather than inferred from the
// robot's pose, so it admits the planning start node wherever it is.
TEST(StationingPositions, ExemptBootstrapAdmitsAForbiddenPosition) {
  StubAgent agent;
  StubMap map(agent);
  StationingRobot robot(agent);
  agent.provide(&map, &robot);
  const Eigen::Vector3d forbidden(1.0, 2.0, 0.0);
  robot.forbidden_positions.push_back(forbidden);

  std::vector<Eigen::Quaterniond> orientations;
  robot.sampleOrientations(forbidden, &orientations);
  std::vector<RobotState> states;
  std::vector<bool> is_traversable;
  robot.checkOrientations(forbidden, orientations, &states, &is_traversable, StationingCheck::kExemptBootstrap);

  ASSERT_EQ(is_traversable.size(), orientations.size());
  for (size_t i = 0; i < is_traversable.size(); ++i) {
    EXPECT_TRUE(is_traversable[i]);
  }
}

/// Breaks the placement contract: a placement layer must commit to a verdict or
/// return kUnset, never kPotentiallyTraversable.
class UncommittedPlacementRobot : public StationingRobot {
 public:
  explicit UncommittedPlacementRobot(AgentInterface& agent) : StationingRobot(agent) {}

  Traversability placeRobotAtSensorPose(RobotState* robot_state, const Transform3d& sensor_transform, Map* map) const override {
    StationingRobot::placeRobotAtSensorPose(robot_state, sensor_transform, map);
    return Traversability::kPotentiallyTraversable;
  }
};

// The placement contract is checked before the stationing short-circuit, so a
// placement that breaks it throws at a rejected position too.
TEST(StationingPositions, UncommittedPlacementStillThrowsAtARejectedPosition) {
  StubAgent agent;
  StubMap map(agent);
  UncommittedPlacementRobot robot(agent);
  agent.provide(&map, &robot);
  const Eigen::Vector3d forbidden(1.0, 2.0, 0.0);
  robot.forbidden_positions.push_back(forbidden);

  std::vector<Eigen::Quaterniond> orientations;
  robot.sampleOrientations(forbidden, &orientations);
  std::vector<RobotState> states;
  std::vector<bool> is_traversable;

  EXPECT_THROW(robot.checkOrientations(forbidden, orientations, &states, &is_traversable), std::logic_error);
}

}  // namespace
}  // namespace open_apero
