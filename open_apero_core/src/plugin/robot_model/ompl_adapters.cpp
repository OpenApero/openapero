#include "open_apero_core/plugin/robot_model/ompl_adapters.h"

#include <stdexcept>

#include <Eigen/Geometry>

namespace open_apero {
namespace robot_model {

namespace ob = ompl::base;

std::string resolveStateSpaceTypeAlias(const std::string& state_space_type) {
  if (state_space_type == "R2") {
    return "R3";
  }
  if (state_space_type == "SE2") {
    return "R3Yaw";
  }
  return state_space_type;
}

void getPoseFromOmplState(const ob::State* state, Eigen::Vector3d& pos, double& yaw, const std::string& state_space_type) {
  if (state_space_type == "R3") {
    const auto* r3 = state->as<ob::RealVectorStateSpace::StateType>();
    pos = Eigen::Vector3d(r3->values[0], r3->values[1], r3->values[2]);
    yaw = 0.0;
  } else if (state_space_type == "R3Yaw") {
    const auto* s = state->as<ob::CompoundState>();
    const auto* r3 = s->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* so2 = s->as<ob::SO2StateSpace::StateType>(1);
    pos = Eigen::Vector3d(r3->values[0], r3->values[1], r3->values[2]);
    yaw = so2->value;
  } else {
    throw std::runtime_error("Unknown state space type: " + state_space_type);
  }
}

void setStateSpaceBounds(const ob::StateSpacePtr& space, const Eigen::Vector3d& lower, const Eigen::Vector3d& upper,
                         const std::string& state_space_type) {
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, lower.x());
  bounds.setLow(1, lower.y());
  bounds.setLow(2, lower.z());
  bounds.setHigh(0, upper.x());
  bounds.setHigh(1, upper.y());
  bounds.setHigh(2, upper.z());

  if (state_space_type == "R3") {
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
  } else if (state_space_type == "R3Yaw") {
    space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
  }
}

void setStartAndGoalStates(const ob::ProblemDefinitionPtr& pdef, const ob::StateSpacePtr& space, const RobotState& start_pose,
                           const RobotState& goal_pose, const std::string& state_space_type) {
  if (state_space_type == "R3") {
    ob::ScopedState<> ompl_start(space);
    ompl_start[0] = start_pose.getSensorPosition().x();
    ompl_start[1] = start_pose.getSensorPosition().y();
    ompl_start[2] = start_pose.getSensorPosition().z();

    ob::ScopedState<> ompl_goal(space);
    ompl_goal[0] = goal_pose.getSensorPosition().x();
    ompl_goal[1] = goal_pose.getSensorPosition().y();
    ompl_goal[2] = goal_pose.getSensorPosition().z();

    pdef->setStartAndGoalStates(ompl_start, ompl_goal);
  } else if (state_space_type == "R3Yaw") {
    ob::ScopedState<ob::CompoundStateSpace> ompl_start(space);
    auto ompl_start_r3 = ompl_start->as<ob::RealVectorStateSpace::StateType>(0);
    auto ompl_start_so2 = ompl_start->as<ob::SO2StateSpace::StateType>(1);
    ompl_start_r3->values[0] = start_pose.getSensorPosition().x();
    ompl_start_r3->values[1] = start_pose.getSensorPosition().y();
    ompl_start_r3->values[2] = start_pose.getSensorPosition().z();
    ompl_start_so2->value = start_pose.getSensorYaw();

    ob::ScopedState<ob::CompoundStateSpace> ompl_goal(space);
    auto ompl_goal_r3 = ompl_goal->as<ob::RealVectorStateSpace::StateType>(0);
    auto ompl_goal_so2 = ompl_goal->as<ob::SO2StateSpace::StateType>(1);
    ompl_goal_r3->values[0] = goal_pose.getSensorPosition().x();
    ompl_goal_r3->values[1] = goal_pose.getSensorPosition().y();
    ompl_goal_r3->values[2] = goal_pose.getSensorPosition().z();
    ompl_goal_so2->value = goal_pose.getSensorYaw();

    pdef->setStartAndGoalStates(ompl_start, ompl_goal);
  } else {
    throw std::runtime_error("Unknown state space type: " + state_space_type);
  }
}

bool OpenAperoStateValidityChecker::isValid(const ob::State* state) const {
  Eigen::Vector3d pos;
  double yaw;
  getPoseFromOmplState(state, pos, yaw, state_space_type_);

  Map* map = robot_->getMap();
  RobotState temp_state;
  robot_->placeRobotAtSensorPose(&temp_state, {pos, quaternionFromYaw(yaw)}, map);
  return robot_->isStateTraversable(temp_state, map);
}

double OpenAperoStateValidityChecker::clearance(const ob::State* state) const {
  Eigen::Vector3d pos;
  double yaw;
  getPoseFromOmplState(state, pos, yaw, state_space_type_);

  Map* map = robot_->getMap();
  RobotState temp_state;
  robot_->placeRobotAtSensorPose(&temp_state, {pos, quaternionFromYaw(yaw)}, map);

  // An untraversable state has no margin left to report. Answering with the floor
  // rather than 0 keeps ClearanceObjective's reciprocal finite, and the validity
  // checker has already excluded such states from any path.
  if (!robot_->isStateTraversable(temp_state, map)) {
    return robot_->getClearanceMarginEps();
  }
  return robot_->getStateClearance(temp_state, map);
}

bool OpenAperoMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2) const {
  Map* map = robot_->getMap();
  RobotState start_robot_state;
  RobotState end_robot_state;

  if (state_space_type_ == "R3") {
    const auto* ompl_start = s1->as<ob::RealVectorStateSpace::StateType>();
    const auto* ompl_end = s2->as<ob::RealVectorStateSpace::StateType>();

    Eigen::Vector3d start_pos(ompl_start->values[0], ompl_start->values[1], ompl_start->values[2]);
    Eigen::Vector3d end_pos(ompl_end->values[0], ompl_end->values[1], ompl_end->values[2]);

    const Eigen::Vector3d direction = end_pos - start_pos;
    const double yaw = atan2(direction.y(), direction.x());
    Eigen::Quaterniond yaw_quaternion = quaternionFromYaw(yaw);

    robot_->placeRobotAtSensorPose(&start_robot_state, {start_pos, yaw_quaternion}, map);
    robot_->placeRobotAtSensorPose(&end_robot_state, {end_pos, yaw_quaternion}, map);
  } else if (state_space_type_ == "R3Yaw") {
    const auto* ompl_start = s1->as<ob::CompoundState>();
    const auto* ompl_end = s2->as<ob::CompoundState>();

    const auto* ompl_start_r3 = ompl_start->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* ompl_start_so2 = ompl_start->as<ob::SO2StateSpace::StateType>(1);
    Eigen::Vector3d start_pos(ompl_start_r3->values[0], ompl_start_r3->values[1], ompl_start_r3->values[2]);
    robot_->placeRobotAtSensorPose(&start_robot_state, {start_pos, quaternionFromYaw(ompl_start_so2->value)}, map);

    const auto* ompl_end_r3 = ompl_end->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* ompl_end_so2 = ompl_end->as<ob::SO2StateSpace::StateType>(1);
    Eigen::Vector3d end_pos(ompl_end_r3->values[0], ompl_end_r3->values[1], ompl_end_r3->values[2]);
    robot_->placeRobotAtSensorPose(&end_robot_state, {end_pos, quaternionFromYaw(ompl_end_so2->value)}, map);
  } else {
    throw std::runtime_error("Unknown state space type: " + state_space_type_);
  }

  // Coincident samples: there is no motion to validate (steer refuses
  // identical-pose pairs by contract); the motion is valid iff the state is.
  if (start_robot_state.isSameSensorPose(end_robot_state)) {
    return robot_->isStateTraversable(start_robot_state, map);
  }

  RobotTrajectory trajectory = robot_->steer(start_robot_state, end_robot_state, map);
  return robot_->isTrajectoryTraversable(trajectory, map);
}

bool OpenAperoMotionValidator::checkMotion(const ob::State* /*s1*/, const ob::State* /*s2*/,
                                           std::pair<ob::State*, double>& /*lastValid*/) const {
  // A steered motion has no partial result to report; only planners using the plain overload
  // (the RRT family) are supported.
  throw std::runtime_error(
      "OpenAperoMotionValidator::checkMotion: the overload reporting lastValid is not supported (RRT-family planners only).");
}

ob::Cost TimeObjective::motionCost(const ob::State* s1, const ob::State* s2) const {
  Map* map = robot_->getMap();
  Eigen::Vector3d pos1;
  double yaw1;
  getPoseFromOmplState(s1, pos1, yaw1, state_space_type_);

  RobotState s1_state;
  robot_->placeRobotAtSensorPose(&s1_state, {pos1, quaternionFromYaw(yaw1)}, map);

  Eigen::Vector3d pos2;
  double yaw2;
  getPoseFromOmplState(s2, pos2, yaw2, state_space_type_);

  RobotState s2_state;
  robot_->placeRobotAtSensorPose(&s2_state, {pos2, quaternionFromYaw(yaw2)}, map);

  // Coincident samples: no motion, no cost (steer refuses identical-pose pairs
  // by contract).
  if (s1_state.isSameSensorPose(s2_state)) {
    return ob::Cost(0.0);
  }

  RobotTrajectory trajectory = robot_->steer(s1_state, s2_state, map);
  return ob::Cost(trajectory.back().getTimeFromStart() - trajectory.front().getTimeFromStart());
}

}  // namespace robot_model
}  // namespace open_apero
