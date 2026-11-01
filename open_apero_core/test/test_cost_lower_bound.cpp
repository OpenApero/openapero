/**
 * @file test_cost_lower_bound.cpp
 * @brief No edge the robot plans costs less than the cost computer's lower bound.
 *
 * TrajectoryTime and TrajectoryLength are checked on translations and rotations between random poses in a free world.
 */

#include <cmath>
#include <random>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/plugin/cost_computer/trajectory_length.h"
#include "open_apero_core/plugin/cost_computer/trajectory_time.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "open_apero_core/state/robot_state.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

TEST(CostLowerBound, NoPlannedEdgeUndercutsIt) {
  SearchOnlyAgent agent;
  FreeTsdfMap map(agent);
  robot_model::SimpleRobot robot(agent);
  Configurable::ParamMap params{{"max_linear_velocity", "0.5"}, {"max_yaw_rate", "1.6"}, {"time_step", "0.2"}};
  robot.setupFromParamMap(&params);
  cost_computer::TrajectoryTime time(agent);
  cost_computer::TrajectoryLength length(agent);
  agent.provide(&map, &robot, &time);

  std::mt19937 rng(11);
  std::uniform_real_distribution<double> position(-8.0, 8.0);
  std::uniform_real_distribution<double> yaw(-M_PI, M_PI);
  const auto state_at = [&](const Eigen::Vector3d& p, double heading) {
    RobotState state;
    robot.placeRobotAtSensorPose(&state, {p, quaternionFromYaw(heading)});
    return state;
  };

  int num_planned = 0;
  for (int i = 0; i < 2000; ++i) {
    const RobotState start = state_at(Eigen::Vector3d(position(rng), position(rng), 0.0), yaw(rng));
    const bool rotation = i % 5 == 0;
    const RobotState goal = state_at(rotation ? start.getSensorPosition() : Eigen::Vector3d(position(rng), position(rng), 0.0), yaw(rng));
    if (start.isSameSensorPose(goal)) {
      continue;
    }
    Edge edge;
    const bool planned = rotation ? robot.planRotationTrajectory(start, goal, &map, &edge.trajectory)
                                  : robot.planTrajectory(start, goal, &map, &edge.trajectory);
    if (!planned) {
      continue;
    }
    ++num_planned;
    time.computeCost(&edge);
    EXPECT_GE(edge.cost, time.getCostLowerBound(start, goal) - 1e-9);
    length.computeCost(&edge);
    EXPECT_GE(edge.cost, length.getCostLowerBound(start, goal) - 1e-9);
  }
  EXPECT_GT(num_planned, 1000);
}

}  // namespace
}  // namespace open_apero
