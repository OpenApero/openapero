/**
 * @file test_retarget_trajectory_endpoints.cpp
 * @brief Re-aiming both end rotations of a validated trajectory at new endpoint states.
 *
 * The sequence of positions survives, so an orientation-independent body needs no second map
 * query, and a refusal at either end leaves the caller's trajectory unchanged.
 */

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/map/tsdf_map.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "open_apero_core/state/robot_state.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// A distance every collision sphere reads as clear, and one it reads as a collision.
constexpr double kFarFromObstacles = 5.0;
constexpr double kInsideObstacle = 0.05;

// SimpleRobot::setupFromParamMap configures its bounding volume through the factory; ConfigAgent's empty
// in-memory factory yields the (unbounded) defaults.
class StubAgent : public ConfigAgent {
 public:
  StubAgent() {
    setCostBudget(0.0);
    setRandomSeed(-1);
  }
};

/// A TSDF map whose distance field is whatever a test scripts it to be, counting the
/// queries it answers so a test can assert the retarget never asked.
class ScriptedTsdfMap : public map::TsdfMap {
 public:
  explicit ScriptedTsdfMap(AgentInterface& agent) : TsdfMap(agent) {}

  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& /*point*/) override { return Occupancy::kUnknown; }
  double getVoxelSize() override { return 0.1; }
  bool getVoxelCenter(Eigen::Vector3d* /*center*/, const Eigen::Vector3d& /*point*/) override { return false; }
  double getVoxelWeight(const Eigen::Vector3d& /*point*/) override { return 0.0; }
  double getMaximumWeight() override { return 0.0; }

  bool getDistanceAtPosition(const Eigen::Vector3d& /*position*/, double* distance) override {
    ++distance_queries;
    if (!uniform_distance.has_value()) {
      return false;
    }
    *distance = *uniform_distance;
    return true;
  }

  std::optional<double> uniform_distance;
  int distance_queries = 0;
};

/// The step sizes the tests count in: max_yaw_step = 1.6 * 0.2 = 0.32 rad and
/// max_walk_step = 0.5 * 0.2 = 0.1 m per state.
Configurable::ParamMap steppingParams() {
  return Configurable::ParamMap{
      {"param_namespace", "/test_retarget"}, {"max_linear_velocity", "0.5"}, {"max_yaw_rate", "1.6"},       {"time_step", "0.2"},
      {"collision_sphere_x", "0.0"},         {"collision_sphere_y", "0.0"},  {"collision_sphere_z", "0.0"}, {"collision_sphere_r", "0.25"},
      {"collision_check_yaw", "0"}};
}

/// The same body, declared orientation-dependent. Centered geometry deduces the opposite,
/// and it is the declared verdict that decides whether the retarget re-checks its result.
Configurable::ParamMap yawCheckedParams() {
  Configurable::ParamMap params = steppingParams();
  params["collision_check_yaw"] = "1";
  return params;
}

class TestRobot : public robot_model::SimpleRobot {
 public:
  TestRobot(AgentInterface& agent, Configurable::ParamMap params) : robot_model::SimpleRobot(agent) { setupFromParamMap(&params); }

  RobotState stateAt(const Eigen::Vector3d& position, double yaw) const {
    RobotState state;
    placeRobotAtSensorPose(&state, {position, quaternionFromYaw(yaw)});
    return state;
  }
};

/// The positions a trajectory visits, with each in-place run folded into one entry.
std::vector<Eigen::Vector3d> distinctPositions(const RobotTrajectory& trajectory) {
  std::vector<Eigen::Vector3d> positions;
  for (const auto& state : trajectory) {
    if (positions.empty() || !isSamePosition(positions.back(), state.getSensorPosition())) {
      positions.push_back(state.getSensorPosition());
    }
  }
  return positions;
}

void expectSamePositions(const std::vector<Eigen::Vector3d>& actual, const std::vector<Eigen::Vector3d>& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_LT((actual[i] - expected[i]).norm(), 1e-9) << "position " << i;
  }
}

/// Every state, not just the ends: a half-rebuilt head has the same length and positions
/// as the source and differs only in the orientations along it.
void expectSameTrajectory(const RobotTrajectory& actual, const RobotTrajectory& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_TRUE(actual[i].isSameSensorPose(expected[i])) << "state " << i;
  }
}

class RetargetTrajectoryEndpointsTest : public ::testing::Test {
 protected:
  RetargetTrajectoryEndpointsTest() : map_(agent_) { agent_.provide(&map_, nullptr); }

  StubAgent agent_;
  ScriptedTsdfMap map_;
};

// A walk-then-rotate edge re-aimed at both endpoints: both ends land on the requested
// states and the corridor between them is unchanged.
TEST_F(RetargetTrajectoryEndpointsTest, ReaimsBothEndsAndKeepsThePositionSequence) {
  TestRobot robot(agent_, steppingParams());

  const Eigen::Vector3d start_position(0.0, 0.0, 0.0);
  const Eigen::Vector3d goal_position(0.5, 0.0, 0.0);
  RobotTrajectory trajectory = robot.steer(robot.stateAt(start_position, 0.0), robot.stateAt(goal_position, 0.8));
  const std::vector<Eigen::Vector3d> source_positions = distinctPositions(trajectory);

  const RobotState start_state = robot.stateAt(start_position, -0.7);
  const RobotState goal_state = robot.stateAt(goal_position, 1.2);

  ASSERT_TRUE(robot.retargetTrajectoryEndpoints(&trajectory, start_state, goal_state));

  EXPECT_TRUE(trajectory.front().isSameSensorPose(start_state, 1e-9, 1e-9));
  EXPECT_TRUE(trajectory.back().isSameSensorPose(goal_state, 1e-9, 1e-9));
  expectSamePositions(distinctPositions(trajectory), source_positions);
}

// The refusal the local copy exists for: the head rebuild succeeds and the tail then
// declines, leaving nothing to rotate toward a goal already at that pose.
TEST_F(RetargetTrajectoryEndpointsTest, ATailRefusalLeavesTheTrajectoryUntouched) {
  TestRobot robot(agent_, steppingParams());

  const Eigen::Vector3d position(0.5, -0.6, 2.3);
  RobotTrajectory trajectory{robot.stateAt(position, 0.0), robot.stateAt(position, 0.32), robot.stateAt(position, 0.64)};
  setUniformTrajectoryTimes(trajectory, robot.getTimeStep());
  const RobotTrajectory source_trajectory = trajectory;

  // The head reaches -0.5; the goal sits within the pose tolerance of it.
  EXPECT_FALSE(robot.retargetTrajectoryEndpoints(&trajectory, robot.stateAt(position, -0.5), robot.stateAt(position, -0.501)));

  expectSameTrajectory(trajectory, source_trajectory);
}

// A mismatched endpoint position is a contract violation, not a refusal -- and it still
// must not consume the trajectory.
TEST_F(RetargetTrajectoryEndpointsTest, AMismatchedEndpointPositionThrowsAndLeavesTheTrajectoryUntouched) {
  TestRobot robot(agent_, steppingParams());

  const Eigen::Vector3d start_position(0.0, 0.0, 0.0);
  const Eigen::Vector3d goal_position(0.5, 0.0, 0.0);
  RobotTrajectory trajectory = robot.steer(robot.stateAt(start_position, 0.0), robot.stateAt(goal_position, 0.8));
  const RobotTrajectory source_trajectory = trajectory;

  // The head matches and is rebuilt on the copy before the tail rejects its position.
  EXPECT_THROW(robot.retargetTrajectoryEndpoints(&trajectory, robot.stateAt(start_position, -0.7),
                                                 robot.stateAt(Eigen::Vector3d(0.6, 0.0, 0.0), 1.2)),
               std::invalid_argument);

  expectSameTrajectory(trajectory, source_trajectory);
}

// The map is scripted to reject every pose, and the retarget still succeeds without
// asking it anything.
TEST_F(RetargetTrajectoryEndpointsTest, AnOrientationIndependentRobotIsVouchedForWithoutQueryingTheMap) {
  TestRobot robot(agent_, steppingParams());
  map_.uniform_distance = kInsideObstacle;

  const Eigen::Vector3d start_position(0.0, 0.0, 0.0);
  const Eigen::Vector3d goal_position(0.5, 0.0, 0.0);
  RobotTrajectory trajectory = robot.steer(robot.stateAt(start_position, 0.0), robot.stateAt(goal_position, 0.8));

  EXPECT_TRUE(robot.retargetTrajectoryEndpoints(&trajectory, robot.stateAt(start_position, -0.7), robot.stateAt(goal_position, 1.2)));
  EXPECT_EQ(map_.distance_queries, 0);
}

TEST_F(RetargetTrajectoryEndpointsTest, AnOrientationDependentRobotRevalidatesAndAcceptsAClearMap) {
  TestRobot robot(agent_, yawCheckedParams());
  map_.uniform_distance = kFarFromObstacles;

  const Eigen::Vector3d start_position(0.0, 0.0, 0.0);
  const Eigen::Vector3d goal_position(0.5, 0.0, 0.0);
  RobotTrajectory trajectory = robot.steer(robot.stateAt(start_position, 0.0), robot.stateAt(goal_position, 0.8));
  const std::vector<Eigen::Vector3d> source_positions = distinctPositions(trajectory);
  const RobotState goal_state = robot.stateAt(goal_position, 1.2);

  ASSERT_TRUE(robot.retargetTrajectoryEndpoints(&trajectory, robot.stateAt(start_position, -0.7), goal_state));

  EXPECT_GT(map_.distance_queries, 0);
  EXPECT_TRUE(trajectory.back().isSameSensorPose(goal_state, 1e-9, 1e-9));
  expectSamePositions(distinctPositions(trajectory), source_positions);
}

// Same trajectory and endpoints, but a map that refuses them: a yaw-dependent body
// re-checks the rebuilt orientations, so the answer is a refusal.
TEST_F(RetargetTrajectoryEndpointsTest, AnOrientationDependentRobotRefusesWhenRevalidationFails) {
  TestRobot robot(agent_, yawCheckedParams());
  map_.uniform_distance = kInsideObstacle;

  const Eigen::Vector3d start_position(0.0, 0.0, 0.0);
  const Eigen::Vector3d goal_position(0.5, 0.0, 0.0);
  RobotTrajectory trajectory = robot.steer(robot.stateAt(start_position, 0.0), robot.stateAt(goal_position, 0.8));
  const RobotTrajectory source_trajectory = trajectory;

  EXPECT_FALSE(robot.retargetTrajectoryEndpoints(&trajectory, robot.stateAt(start_position, -0.7), robot.stateAt(goal_position, 1.2)));

  expectSameTrajectory(trajectory, source_trajectory);
}

}  // namespace
}  // namespace open_apero
