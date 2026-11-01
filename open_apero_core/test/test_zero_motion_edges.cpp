/**
 * @file test_zero_motion_edges.cpp
 * @brief No edge may connect two states at the same pose, and no trajectory may hold
 *        two consecutive states at the same pose.
 *
 * Covers steer's sub-tolerance last step, which a replanning-horizon split would otherwise turn
 * into a zero-motion edge.
 */

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/generator/tree_generator.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "open_apero_core/state/robot_state.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

class StubMap : public Map {
 public:
  explicit StubMap(AgentInterface& agent) : Map(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
};

// SimpleRobot::setupFromParamMap configures its bounding volume through the factory; ConfigAgent's empty
// in-memory factory yields the (unbounded) defaults.
class StubAgent : public ConfigAgent {
 public:
  StubAgent() {
    setCostBudget(0.0);
    setRandomSeed(-1);
  }
};

// The sweep's robot parameters: max_yaw_step = 1.6 * 0.2 = 0.32 rad and
// max_walk_step = 0.5 * 0.2 = 0.1 m per state.
class SweepRobot : public robot_model::SimpleRobot {
 public:
  explicit SweepRobot(AgentInterface& agent) : robot_model::SimpleRobot(agent) {
    Configurable::ParamMap params{{"max_linear_velocity", "0.5"}, {"max_yaw_rate", "1.6"}, {"time_step", "0.2"}};
    setupFromParamMap(&params);
  }
};

// The split path calls computeCost, which needs an owning layer; bypass it. The
// remaining pure virtuals are sampling and start-node hooks the split never touches.
class SplitProbe : public TreeGenerator {
 public:
  explicit SplitProbe(AgentInterface& agent) : TreeGenerator(agent) {
    Configurable::ParamMap params;
    setupFromParamMap(&params);
  }
  bool computeCost(Edge* edge) override {
    edge->cost = static_cast<double>(edge->trajectory.size());
    return true;
  }
  NodeClusterPtrs generateNewNode(Node* /*start_node*/) override { return {}; }
  void setNewStartNode(Node* /*previous_start*/, Node* /*new_start*/) override {}
};

RobotState makeState(const RobotModel& robot, const Eigen::Vector3d& position, double yaw) {
  RobotState state;
  robot.placeRobotAtSensorPose(&state, {position, quaternionFromYaw(yaw)});
  return state;
}

void expectNoConsecutiveDuplicates(const RobotTrajectory& trajectory) {
  for (size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_FALSE(trajectory[i].isSameSensorPose(trajectory[i - 1])) << "states " << i - 1 << " and " << i << " share one pose";
  }
}

/// The last state carries the goal's sensor pose bit for bit.
void expectEndsExactlyOn(const RobotTrajectory& trajectory, const RobotState& goal) {
  ASSERT_FALSE(trajectory.empty());
  EXPECT_TRUE(trajectory.back().getSensorPosition() == goal.getSensorPosition());
  EXPECT_TRUE(trajectory.back().getSensorOrientation().coeffs() == goal.getSensorOrientation().coeffs());
}

// A full step would leave a sub-tolerance residual no consumer can tell apart from its
// predecessor, so the last two steps share the remainder.
TEST(ZeroMotionEdges, SteerSharesASubToleranceResidualBetweenTheLastTwoSteps) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);

  const Eigen::Vector3d position(0.5, -0.6, 2.3);
  const RobotState start = makeState(robot, position, 0.0);
  const RobotState goal = makeState(robot, position, 0.321);

  const RobotTrajectory trajectory = robot.steer(start, goal);

  ASSERT_EQ(trajectory.size(), 3u);
  EXPECT_NEAR(trajectory[0].getSensorYaw(), 0.0, 1e-12);
  EXPECT_NEAR(trajectory[1].getSensorYaw(), 0.1605, 1e-12);
  EXPECT_NEAR(trajectory[2].getSensorYaw(), 0.321, 1e-12);
  expectNoConsecutiveDuplicates(trajectory);
  expectEndsExactlyOn(trajectory, goal);
}

TEST(ZeroMotionEdges, SteerStepsAtTheMaxRateThenAbsorbsTheRemainder) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);

  const Eigen::Vector3d position(0.5, -0.6, 2.3);
  const RobotState start = makeState(robot, position, 0.0);
  const RobotState goal = makeState(robot, position, 0.5);

  const RobotTrajectory trajectory = robot.steer(start, goal);

  ASSERT_EQ(trajectory.size(), 3u);
  EXPECT_NEAR(trajectory[0].getSensorYaw(), 0.0, 1e-12);
  EXPECT_NEAR(trajectory[1].getSensorYaw(), 0.32, 1e-12);
  EXPECT_NEAR(trajectory[2].getSensorYaw(), 0.5, 1e-12);
  expectNoConsecutiveDuplicates(trajectory);
  expectEndsExactlyOn(trajectory, goal);
}

// Walk to the goal position, then rotate by just over one full step.
TEST(ZeroMotionEdges, SteerWalkThenRotateHasNoDuplicateStates) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);

  // Travel along +x with matching start yaw, so the walk needs no pre-rotation.
  const RobotState start = makeState(robot, {0.0, 0.0, 0.0}, 0.0);
  const RobotState goal = makeState(robot, {0.323, 0.0, 0.0}, 0.321);

  const RobotTrajectory trajectory = robot.steer(start, goal);

  // start + 4 walk states (3 full + residual) + 2 rotation steps sharing the sub-tolerance residual.
  EXPECT_EQ(trajectory.size(), 7u);
  expectNoConsecutiveDuplicates(trajectory);
  expectEndsExactlyOn(trajectory, goal);
}

// The goal yaw sits 2 mrad off the travel heading, inside the pose tolerance: the last
// rotation is absorbed and the trajectory still ends on the goal, within the rates.
TEST(ZeroMotionEdges, SteerEndsExactlyOnAGoalYawWithinPoseTolerance) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);

  const RobotState start = makeState(robot, {0.0, 0.0, 0.0}, 0.7);
  const RobotState goal = makeState(robot, {0.1 * std::cos(M_PI / 4.0), 0.1 * std::sin(M_PI / 4.0), 0.0}, M_PI / 4.0 + 0.002);

  const RobotTrajectory trajectory = robot.steer(start, goal);

  ASSERT_GE(trajectory.size(), 2u);
  const Eigen::Vector3d end_position = trajectory.back().getSensorPosition();
  EXPECT_EQ(end_position.x(), goal.getSensorPosition().x());
  EXPECT_EQ(end_position.y(), goal.getSensorPosition().y());
  EXPECT_EQ(end_position.z(), goal.getSensorPosition().z());
  EXPECT_LE(std::abs(angleDifference(trajectory.back().getSensorYaw(), goal.getSensorYaw())), 1e-12);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_LE(std::abs(angleDifference(trajectory[i - 1].getSensorYaw(), trajectory[i].getSensorYaw())), 0.32 + 1e-12) << "step " << i;
    EXPECT_LE((trajectory[i].getSensorPosition() - trajectory[i - 1].getSensorPosition()).norm(), 0.1 + 1e-12) << "step " << i;
  }
  expectNoConsecutiveDuplicates(trajectory);
}

// An edge can still hold a duplicate pair (e.g. from a robot model with a different dedup
// tolerance); splitting there must refuse and commit to the whole edge.
TEST(ZeroMotionEdges, SplitRefusesRemainderTwinnedWithGoal) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);
  SplitProbe generator(agent);

  // Five real steps, then a sub-tolerance rotation residual.
  const Eigen::Vector3d end_position(0.4, 0.0, 0.0);
  RobotTrajectory trajectory;
  for (int i = 0; i <= 3; ++i) {
    trajectory.push_back(makeState(robot, {0.1 * i, 0.0, 0.0}, 0.0));
  }
  trajectory.push_back(makeState(robot, end_position, 0.0));
  trajectory.push_back(makeState(robot, end_position, 0.32));
  trajectory.push_back(makeState(robot, end_position, 0.321));
  setUniformTrajectoryTimes(trajectory, robot.getTimeStep());

  TreeNode start_node;
  start_node.robot_state = trajectory.front();
  TreeNode* goal_node = start_node.spawnChild();
  goal_node->robot_state = trajectory.back();
  goal_node->edge.trajectory = trajectory;

  const auto result = generator.insertIntermediateNode(&start_node, goal_node, &goal_node->edge, 5);

  EXPECT_EQ(result.first, goal_node);
  EXPECT_EQ(result.second, &goal_node->edge);
  EXPECT_EQ(goal_node->edge.trajectory.size(), 7u);
}

TEST(ZeroMotionEdges, SplitRefusesPrefixTwinnedWithStart) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);
  SplitProbe generator(agent);

  const Eigen::Vector3d position(0.0, 0.0, 0.0);
  RobotTrajectory trajectory;
  trajectory.push_back(makeState(robot, position, 0.0));
  trajectory.push_back(makeState(robot, position, 0.001));  // sub-tolerance first step
  trajectory.push_back(makeState(robot, position, 0.32));
  setUniformTrajectoryTimes(trajectory, robot.getTimeStep());

  TreeNode start_node;
  start_node.robot_state = trajectory.front();
  TreeNode* goal_node = start_node.spawnChild();
  goal_node->robot_state = trajectory.back();
  goal_node->edge.trajectory = trajectory;

  const auto result = generator.insertIntermediateNode(&start_node, goal_node, &goal_node->edge, 1);

  EXPECT_EQ(result.first, goal_node);
  EXPECT_EQ(result.second, &goal_node->edge);
  EXPECT_EQ(goal_node->edge.trajectory.size(), 3u);
}

// A rotation-phase state shares the endpoint's position but differs in yaw by full
// rotation steps -- real motion. The split may mint an intermediate node there; the
// remainder is a valid in-place rotation edge. Only pose twins are refused.
TEST(ZeroMotionEdges, SplitWorksAtRotationPhaseState) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);
  SplitProbe generator(agent);

  // Walk to the goal position, then three full rotation steps.
  const Eigen::Vector3d end_position(0.3, 0.0, 0.0);
  RobotTrajectory trajectory;
  for (int i = 0; i <= 3; ++i) {
    trajectory.push_back(makeState(robot, {0.1 * i, 0.0, 0.0}, 0.0));
  }
  trajectory.push_back(makeState(robot, end_position, 0.32));
  trajectory.push_back(makeState(robot, end_position, 0.64));
  trajectory.push_back(makeState(robot, end_position, 0.96));
  setUniformTrajectoryTimes(trajectory, robot.getTimeStep());

  TreeNode start_node;
  start_node.robot_state = trajectory.front();
  TreeNode* goal_node = start_node.spawnChild();
  goal_node->robot_state = trajectory.back();
  goal_node->edge.trajectory = trajectory;

  // Index 4 is at the goal's position but 0.64 rad from the goal yaw: two full
  // rotation steps remain on the goal side.
  const auto result = generator.insertIntermediateNode(&start_node, goal_node, &goal_node->edge, 4);

  ASSERT_NE(result.first, goal_node);
  EXPECT_TRUE(result.first->robot_state.isSameSensorPose(trajectory[4]));
  EXPECT_EQ(result.second->trajectory.size(), 5u);
  EXPECT_EQ(goal_node->edge.trajectory.size(), 3u);
  expectNoConsecutiveDuplicates(result.second->trajectory);
  expectNoConsecutiveDuplicates(goal_node->edge.trajectory);
}

TEST(ZeroMotionEdges, SplitStillWorksAtARealMotionState) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);
  SplitProbe generator(agent);

  RobotTrajectory trajectory;
  for (int i = 0; i <= 6; ++i) {
    trajectory.push_back(makeState(robot, {0.1 * i, 0.0, 0.0}, 0.0));
  }
  setUniformTrajectoryTimes(trajectory, robot.getTimeStep());

  TreeNode start_node;
  start_node.robot_state = trajectory.front();
  TreeNode* goal_node = start_node.spawnChild();
  goal_node->robot_state = trajectory.back();
  goal_node->edge.trajectory = trajectory;

  const auto result = generator.insertIntermediateNode(&start_node, goal_node, &goal_node->edge, 3);

  ASSERT_NE(result.first, goal_node);
  EXPECT_TRUE(result.first->robot_state.isSameSensorPose(trajectory[3]));
  EXPECT_EQ(result.second->trajectory.size(), 4u);
  EXPECT_EQ(goal_node->edge.trajectory.size(), 4u);
}

// A whole-rotation edge trims to its first state; if the requested yaw is within the
// pose tolerance of that state there is nothing left to build -- the call must refuse
// and leave the trajectory untouched instead of collapsing it to a single state.
TEST(ZeroMotionEdges, RetargetTailRefusesWholeRotationToStartYaw) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);

  const Eigen::Vector3d position(0.5, -0.6, 2.3);
  RobotTrajectory trajectory;
  trajectory.push_back(makeState(robot, position, 0.0));
  trajectory.push_back(makeState(robot, position, 0.32));
  trajectory.push_back(makeState(robot, position, 0.64));
  setUniformTrajectoryTimes(trajectory, robot.getTimeStep());

  const RobotState goal = makeState(robot, position, 0.001);

  EXPECT_FALSE(robot.retargetTrajectoryTail(&trajectory, goal));
  ASSERT_EQ(trajectory.size(), 3u);
  EXPECT_TRUE(trajectory[1].isSameSensorPose(makeState(robot, position, 0.32)));
  EXPECT_TRUE(trajectory[2].isSameSensorPose(makeState(robot, position, 0.64)));
}

TEST(ZeroMotionEdges, RetargetTailRebuildsWholeRotationToNewYaw) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);

  const Eigen::Vector3d position(0.5, -0.6, 2.3);
  RobotTrajectory trajectory;
  trajectory.push_back(makeState(robot, position, 0.0));
  trajectory.push_back(makeState(robot, position, 0.32));
  setUniformTrajectoryTimes(trajectory, robot.getTimeStep());

  const RobotState goal = makeState(robot, position, 1.0);

  EXPECT_TRUE(robot.retargetTrajectoryTail(&trajectory, goal));
  EXPECT_GE(trajectory.size(), 2u);
  expectNoConsecutiveDuplicates(trajectory);
  EXPECT_TRUE(trajectory.front().isSameSensorPose(makeState(robot, position, 0.0)));
  EXPECT_TRUE(trajectory.back().isSameSensorPose(goal, 1e-9, 1e-9));
}

// Pins what retargetTrajectoryTail does: cut the trailing in-place
// rotation of a translation edge and re-steer to the new yaw.
TEST(ZeroMotionEdges, RetargetTailTrimsTrailingRotationOfTranslationEdge) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);

  const Eigen::Vector3d end_position(0.2, 0.0, 0.0);
  RobotTrajectory trajectory;
  trajectory.push_back(makeState(robot, {0.0, 0.0, 0.0}, 0.0));
  trajectory.push_back(makeState(robot, {0.1, 0.0, 0.0}, 0.0));
  trajectory.push_back(makeState(robot, end_position, 0.0));
  trajectory.push_back(makeState(robot, end_position, 0.32));
  trajectory.push_back(makeState(robot, end_position, 0.64));
  setUniformTrajectoryTimes(trajectory, robot.getTimeStep());

  const RobotState goal = makeState(robot, end_position, -0.3);

  EXPECT_TRUE(robot.retargetTrajectoryTail(&trajectory, goal));
  ASSERT_EQ(trajectory.size(), 4u);
  expectNoConsecutiveDuplicates(trajectory);
  EXPECT_TRUE(trajectory[2].isSameSensorPose(makeState(robot, end_position, 0.0)));
  EXPECT_TRUE(trajectory.back().isSameSensorPose(goal, 1e-9, 1e-9));
}

TEST(ZeroMotionEdges, RetargetHeadRefusalLeavesOrderIntact) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);

  const Eigen::Vector3d position(0.5, -0.6, 2.3);
  RobotTrajectory trajectory;
  trajectory.push_back(makeState(robot, position, 0.0));
  trajectory.push_back(makeState(robot, position, 0.32));
  trajectory.push_back(makeState(robot, position, 0.64));
  setUniformTrajectoryTimes(trajectory, robot.getTimeStep());

  const RobotState goal = makeState(robot, position, 0.641);

  EXPECT_FALSE(robot.retargetTrajectoryHead(&trajectory, goal));
  ASSERT_EQ(trajectory.size(), 3u);
  EXPECT_TRUE(trajectory.front().isSameSensorPose(makeState(robot, position, 0.0)));
  EXPECT_TRUE(trajectory.back().isSameSensorPose(makeState(robot, position, 0.64)));
}

// Reorienting a node to within the pose tolerance of a coincident child would make
// that child a pose twin with no executable edge. The whole reorientation must be
// refused up front -- node state, tail, and child edges all untouched -- rather than
// half-applied with the child edge flagged for a pruning pass that revalidation can
// undo (revalidateEdge recomputes is_traversable from geometry alone).
TEST(ZeroMotionEdges, ReorientationRefusedWhenChildWouldBecomePoseTwin) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);
  SplitProbe generator(agent);

  const Eigen::Vector3d node_position(0.2, 0.0, 0.0);
  TreeNode root;
  root.robot_state = makeState(robot, {0.0, 0.0, 0.0}, 0.0);

  TreeNode* node = root.spawnChild();
  node->robot_state = makeState(robot, node_position, 0.0);
  node->edge.trajectory = {makeState(robot, {0.0, 0.0, 0.0}, 0.0), makeState(robot, {0.1, 0.0, 0.0}, 0.0),
                           makeState(robot, node_position, 0.0)};
  setUniformTrajectoryTimes(node->edge.trajectory, robot.getTimeStep());

  TreeNode* child = node->spawnChild();
  child->robot_state = makeState(robot, node_position, 0.64);
  child->edge.trajectory = {makeState(robot, node_position, 0.0), makeState(robot, node_position, 0.32),
                            makeState(robot, node_position, 0.64)};
  child->edge.is_traversable = true;
  setUniformTrajectoryTimes(child->edge.trajectory, robot.getTimeStep());

  // Within the pose tolerance of the child's yaw.
  EXPECT_FALSE(generator.setNodeOrientation(node, quaternionFromYaw(0.641)));

  EXPECT_TRUE(node->robot_state.isSameSensorPose(makeState(robot, node_position, 0.0)));
  ASSERT_EQ(node->edge.trajectory.size(), 3u);
  EXPECT_TRUE(node->edge.trajectory.back().isSameSensorPose(makeState(robot, node_position, 0.0)));
  ASSERT_EQ(child->edge.trajectory.size(), 3u);
  EXPECT_TRUE(child->edge.trajectory.front().isSameSensorPose(makeState(robot, node_position, 0.0)));
  EXPECT_TRUE(child->edge.is_traversable);
}

TEST(ZeroMotionEdges, ReorientationRebuildsTailAndCoincidentChildHead) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);
  SplitProbe generator(agent);

  const Eigen::Vector3d node_position(0.2, 0.0, 0.0);
  TreeNode root;
  root.robot_state = makeState(robot, {0.0, 0.0, 0.0}, 0.0);

  TreeNode* node = root.spawnChild();
  node->robot_state = makeState(robot, node_position, 0.0);
  node->edge.trajectory = {makeState(robot, {0.0, 0.0, 0.0}, 0.0), makeState(robot, {0.1, 0.0, 0.0}, 0.0),
                           makeState(robot, node_position, 0.0)};
  setUniformTrajectoryTimes(node->edge.trajectory, robot.getTimeStep());

  TreeNode* child = node->spawnChild();
  child->robot_state = makeState(robot, node_position, 0.64);
  child->edge.trajectory = {makeState(robot, node_position, 0.0), makeState(robot, node_position, 0.32),
                            makeState(robot, node_position, 0.64)};
  child->edge.is_traversable = true;
  setUniformTrajectoryTimes(child->edge.trajectory, robot.getTimeStep());

  EXPECT_TRUE(generator.setNodeOrientation(node, quaternionFromYaw(-0.5)));

  const RobotState expected = makeState(robot, node_position, -0.5);
  EXPECT_TRUE(node->robot_state.isSameSensorPose(expected, 1e-9, 1e-9));
  EXPECT_TRUE(node->edge.trajectory.back().isSameSensorPose(node->robot_state, 1e-9, 1e-9));
  expectNoConsecutiveDuplicates(node->edge.trajectory);
  EXPECT_TRUE(child->edge.trajectory.front().isSameSensorPose(node->robot_state, 1e-9, 1e-9));
  EXPECT_TRUE(child->edge.trajectory.back().isSameSensorPose(makeState(robot, node_position, 0.64)));
  expectNoConsecutiveDuplicates(child->edge.trajectory);
}

TEST(ZeroMotionEdges, RetargetHeadRebuildsLeadingRotation) {
  StubAgent agent;
  StubMap map(agent);
  SweepRobot robot(agent);
  agent.provide(&map, &robot);

  const Eigen::Vector3d position(0.5, -0.6, 2.3);
  RobotTrajectory trajectory;
  trajectory.push_back(makeState(robot, position, 0.0));
  trajectory.push_back(makeState(robot, position, 0.32));
  trajectory.push_back(makeState(robot, position, 0.64));
  setUniformTrajectoryTimes(trajectory, robot.getTimeStep());

  const RobotState goal = makeState(robot, position, -0.5);

  EXPECT_TRUE(robot.retargetTrajectoryHead(&trajectory, goal));
  EXPECT_GE(trajectory.size(), 2u);
  expectNoConsecutiveDuplicates(trajectory);
  EXPECT_TRUE(trajectory.front().isSameSensorPose(goal, 1e-9, 1e-9));
  EXPECT_TRUE(trajectory.back().isSameSensorPose(makeState(robot, position, 0.64)));
}

}  // namespace
}  // namespace open_apero
