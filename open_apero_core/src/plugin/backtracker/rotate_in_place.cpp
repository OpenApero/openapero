#include "open_apero_core/plugin/backtracker/rotate_in_place.h"

#include <cmath>
#include <string>

#include <Eigen/Core>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace backtracker {

PluginRegistry::Registration<RotateInPlace> RotateInPlace::registration("RotateInPlace");

RotateInPlace::RotateInPlace(AgentInterface& agent) : Backtracker(agent) {}

bool RotateInPlace::buildTrajectory(const RobotState& state, RobotTrajectory* out) {
  if (!rotate(state, out)) {
    return false;
  }

  debitInitialRotation(*out);
  return true;
}

void RotateInPlace::debitInitialRotation(const RobotTrajectory& trajectory) {
  if (remaining_init_rotate_angle_ <= 0.0) {
    return;
  }

  double swept_angle = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    swept_angle += std::abs(angleDifference(trajectory[i - 1].getSensorYaw(), trajectory[i].getSensorYaw()));
  }
  remaining_init_rotate_angle_ -= swept_angle;
}

// Rotates yaw by p_rotation_step_ at the robot's time step, pitch unchanged; front() equals @p state
// exactly so the recorded history stays continuous.
bool RotateInPlace::rotate(const RobotState& state, RobotTrajectory* out) {
  const auto& robot = agent_.getRobot();

  const Eigen::Vector3d init_position = state.getSensorPosition();
  Eigen::Vector3d euler_angles_zyx = state.getSensorEulerAnglesZyx();
  euler_angles_zyx(2) = 0.0;
  double& yaw = euler_angles_zyx(0);
  const double final_yaw = yaw + p_rotation_step_;
  const double dt = robot.getTimeStep();
  const double yaw_rate_max = robot.getMaxYawRate();
  const double yaw_step = yaw_rate_max * dt;

  RobotState robot_state;
  robot.placeRobotAtSensorPose(&robot_state, {init_position, quaternionFromEulerAnglesZyx(euler_angles_zyx)}, &agent_.getMap());
  out->clear();
  out->push_back(robot_state);

  while (yaw < final_yaw) {
    yaw += yaw_step;
    robot.placeRobotAtSensorPose(&robot_state, {init_position, quaternionFromEulerAnglesZyx(euler_angles_zyx)}, &agent_.getMap());
    out->push_back(robot_state);
  }

  setUniformTrajectoryTimes(*out, dt);
  return out->size() >= 2;
}

void RotateInPlace::setupFromParamMap(Configurable::ParamMap* param_map) {
  setParam<double>(param_map, "rotation_step", &p_rotation_step_, M_PI);
  setParam<double>(param_map, "initial_rotate_angle", &p_initial_rotate_angle_, 2 * M_PI);
  remaining_init_rotate_angle_ = p_initial_rotate_angle_;
}

bool RotateInPlace::checkParamsValid(std::string* error_message) {
  if (!(p_rotation_step_ > 0.0)) {
    *error_message = "rotation_step expected > 0.0";
    return false;
  }
  if (!(p_initial_rotate_angle_ >= 0.0)) {
    *error_message = "initial_rotate_angle expected >= 0.0";
    return false;
  }
  return Backtracker::checkParamsValid(error_message);
}

bool RotateInPlace::isInitialExplorationDone() {
  return remaining_init_rotate_angle_ <= 0.0;
}

}  // namespace backtracker
}  // namespace open_apero
