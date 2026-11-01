#include "open_apero_core/plugin/robot_model/simple_fallback_robot.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <ompl/util/RandomNumbers.h>

#include "open_apero_core/common/seeding.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/plugin_factory.h"

namespace open_apero {
namespace robot_model {

namespace ob = ompl::base;
namespace og = ompl::geometric;

PluginRegistry::Registration<SimpleFallbackRobot> SimpleFallbackRobot::registration("SimpleFallbackRobot");

SimpleFallbackRobot::SimpleFallbackRobot(AgentInterface& agent) : SimpleRobot(agent) {}

void SimpleFallbackRobot::setupFromParamMap(Configurable::ParamMap* param_map) {
  SimpleRobot::setupFromParamMap(param_map);

  std::string ompl_log_level;
  setParam<std::string>(param_map, "ompl_log_level", &ompl_log_level, std::string("info"));
  std::string ompl_planner_name;
  setParam<std::string>(param_map, "ompl_planner", &ompl_planner_name, "");
  setParam<double>(param_map, "ompl_planning_time", &p_ompl_planning_time_, 0.01);
  setParam<double>(param_map, "ompl_resolution", &p_ompl_resolution_, 0.01);
  setParam<double>(param_map, "ompl_max_range", &p_ompl_max_range_, 2.0);
  setParam<std::string>(param_map, "ompl_state_space_type", &p_ompl_state_space_type_, "R3");
  p_ompl_state_space_type_ = resolveStateSpaceTypeAlias(p_ompl_state_space_type_);

  std::string ompl_objective_name;
  setParam<std::string>(param_map, "ompl_objective", &ompl_objective_name, "time");
  if (ompl_objective_name == "time") {
    ompl_objective_ = FallbackObjective::kTime;
  } else if (ompl_objective_name == "clearance") {
    ompl_objective_ = FallbackObjective::kClearance;
  } else {
    throw std::runtime_error("Unsupported OMPL objective: " + ompl_objective_name + " (expected \"time\" or \"clearance\")");
  }

  if (ompl_planner_name.empty()) {
    ompl_planner_ = nullptr;
    return;
  }

  if (p_ompl_state_space_type_ == "R3") {
    ompl_state_space_ = std::make_shared<ob::RealVectorStateSpace>(3);
  } else if (p_ompl_state_space_type_ == "R3Yaw") {
    ob::StateSpacePtr r3(new ob::RealVectorStateSpace(3));
    ob::StateSpacePtr so2(new ob::SO2StateSpace());
    ompl_state_space_ = r3 + so2;
  } else {
    throw std::runtime_error("Unsupported OMPL state space type: " + p_ompl_state_space_type_);
  }

  // Build OMPL SpaceInformation, validity checker, and motion validator once.
  // ProblemDefinitions are recreated per-call, but they all reuse this SI so the
  // planner — which was constructed against this SI — remains consistent.
  ompl_space_info_ = std::make_shared<ob::SpaceInformation>(ompl_state_space_);
  ompl_space_info_->setStateValidityChecker(
      std::make_shared<OpenAperoStateValidityChecker>(ompl_space_info_, p_ompl_state_space_type_, this));
  ompl_space_info_->setMotionValidator(std::make_shared<OpenAperoMotionValidator>(ompl_space_info_, p_ompl_state_space_type_, this));

  if (ompl_log_level == "info") {
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_INFO);
  } else if (ompl_log_level == "warn") {
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);
  } else if (ompl_log_level == "error") {
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);
  } else if (ompl_log_level == "none") {
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);
  } else {
    throw std::runtime_error("Unsupported OMPL log level: " + ompl_log_level);
  }

  // Before the planner exists: each OMPL RNG draws its own seed from the global one when constructed.
  // The generator is process-global; a later setup reseeds it for the RNGs created afterwards.
  if (agent_.getRandomSeed() >= 0) {
    std::uint_fast32_t ompl_seed = makeSeededEngine(agent_.getRandomSeed(), (*param_map)["param_namespace"])();
    ompl::RNG::setSeed(ompl_seed == 0 ? 1 : ompl_seed);  // OMPL rejects a zero seed.
  }

  if (ompl_planner_name == "RRT") {
    ompl_planner_.reset(new og::RRT(ompl_space_info_));
    ompl_planner_->as<og::RRT>()->setRange(p_ompl_max_range_);
  } else if (ompl_planner_name == "RRTconnect") {
    ompl_planner_.reset(new og::RRTConnect(ompl_space_info_));
    ompl_planner_->as<og::RRTConnect>()->setRange(p_ompl_max_range_);
  } else if (ompl_planner_name == "RRTstar") {
    ompl_planner_.reset(new og::RRTstar(ompl_space_info_));
    ompl_planner_->as<og::RRTstar>()->setRange(p_ompl_max_range_);
  } else if (ompl_planner_name == "RRTsharp") {
    ompl_planner_.reset(new og::RRTsharp(ompl_space_info_));
    ompl_planner_->as<og::RRTsharp>()->setRange(p_ompl_max_range_);
  } else if (ompl_planner_name == "InformedRRTstar") {
    ompl_planner_.reset(new og::InformedRRTstar(ompl_space_info_));
    ompl_planner_->as<og::InformedRRTstar>()->setRange(p_ompl_max_range_);
  } else {
    throw std::runtime_error("Unsupported OMPL planner: " + ompl_planner_name);
  }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> SimpleFallbackRobot::getLocalStateSpaceBounds(const Eigen::Vector3d& start,
                                                                                          const Eigen::Vector3d& goal) const {
  Eigen::Vector3d lower_bound = start.cwiseMin(goal);
  Eigen::Vector3d upper_bound = start.cwiseMax(goal);

  for (const auto& p : {start, goal}) {
    for (int dx : {-1, 1}) {
      for (int dy : {-1, 1}) {
        for (int dz : {-1, 1}) {
          Eigen::Vector3d offset(dx * p_ompl_max_range_, dy * p_ompl_max_range_, dz * p_ompl_max_range_);
          lower_bound = lower_bound.cwiseMin(p + offset);
          upper_bound = upper_bound.cwiseMax(p + offset);
        }
      }
    }
  }

  // Only an active, unrotated volume has world-frame extents to clip to.
  if (bounding_volume_ && bounding_volume_->is_setup && bounding_volume_->rotation == 0.0) {
    lower_bound.x() = std::max(lower_bound.x(), bounding_volume_->x_min);
    lower_bound.y() = std::max(lower_bound.y(), bounding_volume_->y_min);
    lower_bound.z() = std::max(lower_bound.z(), bounding_volume_->z_min);
    upper_bound.x() = std::min(upper_bound.x(), bounding_volume_->x_max);
    upper_bound.y() = std::min(upper_bound.y(), bounding_volume_->y_max);
    upper_bound.z() = std::min(upper_bound.z(), bounding_volume_->z_max);
  }

  // 2D planning: flatten z to the start height so the planner doesn't search a redundant axis.
  if (!p_sample_z_) {
    lower_bound.z() = start.z();
    upper_bound.z() = start.z();
  }

  return {lower_bound, upper_bound};
}

bool SimpleFallbackRobot::planTrajectory(const RobotState& start, const RobotState& goal, Map* map, RobotTrajectory* trajectory) {
  if (SimpleRobot::planTrajectory(start, goal, map, trajectory)) {
    return true;
  }
  // Base steer failed or produced an untraversable trajectory. Only attempt the
  // OMPL fallback when both endpoints are themselves traversable (with planner-
  // level optimism), to avoid wasting time planning into/out of obstacles.
  if (trajectory->empty() || !isStateTraversable(start, map) || !isStateTraversable(goal, map)) {
    return false;
  }
  return planFallback(start, goal, map, trajectory);
}

bool SimpleFallbackRobot::planFallback(const RobotState& start, const RobotState& goal, Map* map, RobotTrajectory* trajectory) {
  if (!ompl_planner_) {
    return false;
  }

  auto bounds = getLocalStateSpaceBounds(start.getSensorPosition(), goal.getSensorPosition());
  setStateSpaceBounds(ompl_state_space_, bounds.first, bounds.second, p_ompl_state_space_type_);

  double max_extent = ompl_state_space_->getMaximumExtent();
  ompl_state_space_->setLongestValidSegmentFraction(p_ompl_resolution_ / max_extent);

  auto problem_def = std::make_shared<ob::ProblemDefinition>(ompl_space_info_);
  if (ompl_objective_ == FallbackObjective::kClearance) {
    problem_def->setOptimizationObjective(std::make_shared<ClearanceObjective>(ompl_space_info_));
  } else {
    problem_def->setOptimizationObjective(std::make_shared<TimeObjective>(ompl_space_info_, p_ompl_state_space_type_, this));
  }
  setStartAndGoalStates(problem_def, ompl_state_space_, start, goal, p_ompl_state_space_type_);

  ompl_planner_->clear();
  ompl_planner_->setProblemDefinition(problem_def);
  ompl_planner_->setup();

  if (ompl_planner_->solve(p_ompl_planning_time_) != ob::PlannerStatus::EXACT_SOLUTION) {
    return false;
  }

  RobotTrajectory waypoints;
  const auto path = problem_def->getSolutionPath()->as<og::PathGeometric>();
  const int num_waypoints = path->getStateCount();
  for (std::size_t i = 0; i < static_cast<std::size_t>(num_waypoints); ++i) {
    Eigen::Vector3d pos;
    double yaw;
    getPoseFromOmplState(path->getState(i), pos, yaw, p_ompl_state_space_type_);

    RobotState current;
    if (i == 0) {
      placeRobotAtSensorPose(&current, {pos, start.getSensorOrientation()}, map);
    } else if (i == static_cast<std::size_t>(num_waypoints - 1)) {
      placeRobotAtSensorPose(&current, {pos, goal.getSensorOrientation()}, map);
    } else {
      if (p_ompl_state_space_type_ == "R3") {
        const Eigen::Vector3d direction = pos - waypoints.back().getSensorPosition();
        placeRobotAtSensorPose(&current, {pos, quaternionFromYaw(std::atan2(direction.y(), direction.x()))}, map);
      } else {
        placeRobotAtSensorPose(&current, {pos, quaternionFromYaw(yaw)}, map);
      }
    }
    waypoints.push_back(current);
  }

  trajectory->clear();
  for (int i = 0; i < static_cast<int>(waypoints.size()) - 1; ++i) {
    // OMPL can emit pose-equal adjacent waypoints; there is no segment to steer
    // between them (steer refuses identical-pose pairs by contract).
    if (waypoints[i].isSameSensorPose(waypoints[i + 1])) {
      continue;
    }
    auto segment = steer(waypoints[i], waypoints[i + 1], map);
    for (auto& state : segment) {
      if (trajectory->empty() || !state.isSameSensorPose(trajectory->back())) {
        trajectory->push_back(state);
      }
    }
  }

  setUniformTrajectoryTimes(*trajectory, getTimeStep());
  if (trajectory->empty()) {
    return false;
  }

  // The steered reconstruction rotates through headings OMPL never validated: check it as a whole.
  return isTrajectoryTraversable(*trajectory, map);
}

}  // namespace robot_model
}  // namespace open_apero
