/**
 * @file test_simple_fallback_robot.cpp
 * @brief The straight line first, the sampling planner only when it is blocked.
 *
 * Both branches are pinned against a wall with a single gap, together with the objective the
 * detour is optimized under, the state space names and the seeding of OMPL's generator.
 */

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <ompl/util/RandomNumbers.h>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/map/tsdf_map.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/plugin/robot_model/ompl_adapters.h"
#include "open_apero_core/plugin/robot_model/simple_fallback_robot.h"
#include "open_apero_core/state/robot_state.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// The wall spans x, is thin in y, and is interrupted by a gap around x = 0.
constexpr double kWallHalfThickness = 0.1;
constexpr double kWallReach = 5.0;
constexpr double kGapHalfWidth = 0.6;

/// Body radius, small enough to fit through the gap and too large to pass the wall.
constexpr double kBodyRadius = 0.25;

/// Start and goal sit on opposite sides of the wall, far from the gap, so the
/// straight line between them crosses solid wall.
const Eigen::Vector3d kStartPosition(2.0, -1.5, 0.0);
const Eigen::Vector3d kGoalPosition(2.0, 1.5, 0.0);

/// Horizontal distance from @p point to the axis-aligned box [min, max].
double distanceToBox(const Eigen::Vector2d& point, const Eigen::Vector2d& min, const Eigen::Vector2d& max) {
  const Eigen::Vector2d outside = (min - point).cwiseMax(point - max).cwiseMax(Eigen::Vector2d::Zero());
  return outside.norm();
}

class WallFactory : public PluginFactory {
 public:
  bool getParamMapAndType(Configurable::ParamMap* map, std::string* /*type*/, std::string /*args*/) override {
    // A volume wide enough to hold the wall, the gap, and both endpoints. The
    // local OMPL bounds are clipped against it, so it cannot be left unset.
    *map = Configurable::ParamMap{
        {"x_min", "-6.0"}, {"x_max", "6.0"}, {"y_min", "-6.0"}, {"y_max", "6.0"}, {"z_min", "-1.0"}, {"z_max", "1.0"},
    };
    return true;
  }

 protected:
  void printVerbose(const Configurable::ParamMap& /*map*/) override {}
  void printError(const std::string& /*message*/) override {}
};

/// ConfigAgent serving WallFactory, with no budget and the default seed.
class StubAgent : public ConfigAgent {
 public:
  StubAgent() {
    useFactory(&wall_factory_);
    setCostBudget(0.0);
    setRandomSeed(-1);
  }

 private:
  WallFactory wall_factory_;
};

/// A fully observed distance field holding one wall with one gap in it.
class WallMap : public map::TsdfMap {
 public:
  explicit WallMap(AgentInterface& agent) : TsdfMap(agent) {}

  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& /*point*/) override { return Occupancy::kUnknown; }
  double getVoxelSize() override { return 0.1; }
  bool getVoxelCenter(Eigen::Vector3d* /*center*/, const Eigen::Vector3d& /*point*/) override { return false; }
  double getVoxelWeight(const Eigen::Vector3d& /*point*/) override { return 0.0; }
  double getMaximumWeight() override { return 0.0; }

  bool getDistanceAtPosition(const Eigen::Vector3d& position, double* distance) override {
    const Eigen::Vector2d point = position.head<2>();
    const double left = distanceToBox(point, {-kWallReach, -kWallHalfThickness}, {-kGapHalfWidth, kWallHalfThickness});
    const double right = distanceToBox(point, {kGapHalfWidth, -kWallHalfThickness}, {kWallReach, kWallHalfThickness});
    *distance = std::min(left, right);
    return true;
  }
};

/// The fallback robot with a body sphere and no feet, so traversability is purely
/// a question of clearing the wall.
Configurable::ParamMap fallbackParams() {
  return Configurable::ParamMap{
      {"param_namespace", "/test_fallback"},
      {"max_linear_velocity", "1.0"},
      {"max_yaw_rate", "3.0"},
      {"time_step", "0.1"},
      {"sample_z", "0"},
      {"collision_sphere_x", "0.0"},
      {"collision_sphere_y", "0.0"},
      {"collision_sphere_z", "0.0"},
      {"collision_sphere_r", std::to_string(kBodyRadius)},
      {"collision_sphere_safe_margins", "0.5"},
      {"ompl_planner", "RRTstar"},
      {"ompl_state_space_type", "R3"},
      {"ompl_log_level", "none"},
      {"ompl_planning_time", "2.0"},
      {"ompl_resolution", "0.06"},
      {"ompl_max_range", "3.0"},
      {"ompl_objective", "clearance"},
  };
}

RobotState stateAt(const robot_model::SimpleFallbackRobot& robot, const Eigen::Vector3d& position) {
  RobotState state;
  robot.placeRobotAtSensorPose(&state, {position, Eigen::Quaterniond::Identity()});
  return state;
}

TEST(FallbackConnector, TheStraightLineIsUsedWhenNothingIsInTheWay) {
  StubAgent agent;
  WallMap map(agent);
  robot_model::SimpleFallbackRobot robot(agent);
  agent.provide(&map, &robot);
  Configurable::ParamMap params = fallbackParams();
  robot.setupFromParamMap(&params);

  // Both endpoints well clear of the wall, on the same side of it.
  const RobotState start = stateAt(robot, kStartPosition);
  const RobotState goal = stateAt(robot, {3.0, -1.5, 0.0});

  RobotTrajectory trajectory;
  ASSERT_TRUE(robot.planTrajectory(start, goal, &map, &trajectory));
  ASSERT_FALSE(trajectory.empty());
  EXPECT_TRUE(trajectory.back().isSameSensorPosition(goal));
  // A steered line holds y; a detour would have to leave it.
  for (const auto& state : trajectory) {
    EXPECT_NEAR(state.getSensorPosition().y(), kStartPosition.y(), 1e-6);
  }
}

TEST(FallbackConnector, ABlockedLineIsReroutedThroughTheGap) {
  StubAgent agent;
  WallMap map(agent);
  robot_model::SimpleFallbackRobot robot(agent);
  agent.provide(&map, &robot);
  Configurable::ParamMap params = fallbackParams();
  robot.setupFromParamMap(&params);

  const RobotState start = stateAt(robot, kStartPosition);
  const RobotState goal = stateAt(robot, kGoalPosition);

  // The straight line crosses solid wall, so the connection only exists via the gap.
  RobotTrajectory straight = robot.steer(start, goal, &map);
  ASSERT_FALSE(robot.isTrajectoryTraversable(straight, &map));

  RobotTrajectory trajectory;
  ASSERT_TRUE(robot.planTrajectory(start, goal, &map, &trajectory));
  EXPECT_TRUE(robot.isTrajectoryTraversable(trajectory, &map));
  EXPECT_TRUE(trajectory.back().isSameSensorPosition(goal));

  // Every crossing of the wall band happens inside the gap.
  for (const auto& state : trajectory) {
    const Eigen::Vector3d& position = state.getSensorPosition();
    if (std::abs(position.y()) <= kWallHalfThickness) {
      EXPECT_LT(std::abs(position.x()), kGapHalfWidth) << "crossed the wall outside the gap";
    }
  }
}

TEST(FallbackConnector, WithoutAPlannerABlockedLineSimplyFails) {
  StubAgent agent;
  WallMap map(agent);
  robot_model::SimpleFallbackRobot robot(agent);
  agent.provide(&map, &robot);
  Configurable::ParamMap params = fallbackParams();
  // Leaving the planner unconfigured means omitting the key: an explicit empty
  // value is not a parseable string, so it is the absent key that selects "none".
  params.erase("ompl_planner");
  robot.setupFromParamMap(&params);

  RobotTrajectory trajectory;
  EXPECT_FALSE(robot.planTrajectory(stateAt(robot, kStartPosition), stateAt(robot, kGoalPosition), &map, &trajectory));
}

TEST(FallbackConnector, TheTimeObjectiveAlsoRoutesThroughTheGap) {
  StubAgent agent;
  WallMap map(agent);
  robot_model::SimpleFallbackRobot robot(agent);
  agent.provide(&map, &robot);
  Configurable::ParamMap params = fallbackParams();
  params["ompl_objective"] = "time";
  robot.setupFromParamMap(&params);

  RobotTrajectory trajectory;
  ASSERT_TRUE(robot.planTrajectory(stateAt(robot, kStartPosition), stateAt(robot, kGoalPosition), &map, &trajectory));
  EXPECT_TRUE(robot.isTrajectoryTraversable(trajectory, &map));
}

TEST(FallbackConnector, AnUnknownObjectiveIsRefusedAtSetup) {
  StubAgent agent;
  WallMap map(agent);
  robot_model::SimpleFallbackRobot robot(agent);
  agent.provide(&map, &robot);
  Configurable::ParamMap params = fallbackParams();
  params["ompl_objective"] = "shortest";
  EXPECT_THROW(robot.setupFromParamMap(&params), std::runtime_error);
}

TEST(FallbackStateSpace, AliasesResolveToTheCanonicalNames) {
  EXPECT_EQ(robot_model::resolveStateSpaceTypeAlias("R2"), "R3");
  EXPECT_EQ(robot_model::resolveStateSpaceTypeAlias("SE2"), "R3Yaw");
  EXPECT_EQ(robot_model::resolveStateSpaceTypeAlias("R3"), "R3");
  EXPECT_EQ(robot_model::resolveStateSpaceTypeAlias("R3Yaw"), "R3Yaw");
  EXPECT_EQ(robot_model::resolveStateSpaceTypeAlias("SE3"), "SE3");
}

TEST(FallbackStateSpace, EveryNameRoutesThroughTheGap) {
  for (const std::string type : {"R3", "R2", "R3Yaw", "SE2"}) {
    SCOPED_TRACE(type);
    StubAgent agent;
    WallMap map(agent);
    robot_model::SimpleFallbackRobot robot(agent);
    agent.provide(&map, &robot);
    Configurable::ParamMap params = fallbackParams();
    params["ompl_state_space_type"] = type;
    params["ompl_planner"] = "RRTconnect";  // stops at its first solution
    robot.setupFromParamMap(&params);

    RobotTrajectory trajectory;
    ASSERT_TRUE(robot.planTrajectory(stateAt(robot, kStartPosition), stateAt(robot, kGoalPosition), &map, &trajectory));
    EXPECT_TRUE(robot.isTrajectoryTraversable(trajectory, &map));
  }
}

TEST(FallbackStateSpace, AnUnknownNameIsRefusedAtSetup) {
  StubAgent agent;
  WallMap map(agent);
  robot_model::SimpleFallbackRobot robot(agent);
  agent.provide(&map, &robot);
  Configurable::ParamMap params = fallbackParams();
  params["ompl_state_space_type"] = "SE3";
  EXPECT_THROW(robot.setupFromParamMap(&params), std::runtime_error);
}

/// The first value an OMPL generator created right after setting up a robot under @p random_seed draws.
double firstOmplDrawAfterSetup(int random_seed) {
  StubAgent agent;
  agent.setRandomSeed(random_seed);
  WallMap map(agent);
  robot_model::SimpleFallbackRobot robot(agent);
  agent.provide(&map, &robot);
  Configurable::ParamMap params = fallbackParams();
  robot.setupFromParamMap(&params);
  ompl::RNG rng;
  return rng.uniform01();
}

TEST(FallbackSeeding, ASeededAgentSeedsOmplsGlobalGenerator) {
  EXPECT_EQ(firstOmplDrawAfterSetup(3), firstOmplDrawAfterSetup(3));
  EXPECT_NE(firstOmplDrawAfterSetup(3), firstOmplDrawAfterSetup(4));
}

}  // namespace
}  // namespace open_apero
