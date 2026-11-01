#include "open_apero_core/state/robot_state.h"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/logging/logging.h"

namespace open_apero {

// Default constructor: NaN time indicates an uninitialized state.
// Position/orientation use identity defaults (origin, no rotation).
RobotState::RobotState()
    : time_from_start_(std::numeric_limits<double>::quiet_NaN()),
      transform_W_B_({Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()}),
      transform_W_S_({Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()}) {}

RobotState::RobotState(double time_from_start, const Transform3d& base_transform, const Transform3d& sensor_transform)
    : time_from_start_(time_from_start), transform_W_B_(base_transform), transform_W_S_(sensor_transform) {}

bool RobotState::isSameSensorPose(const RobotState& other, double position_epsilon, double orientation_dot_epsilon) const {
  return transform_W_S_.isSamePose(other.transform_W_S_, position_epsilon, orientation_dot_epsilon);
}

bool RobotState::isSameSensorPosition(const RobotState& other, double position_epsilon) const {
  return transform_W_S_.isSamePosition(other.transform_W_S_, position_epsilon);
}

bool RobotState::isSameSensorOrientation(const RobotState& other, double orientation_dot_epsilon) const {
  return transform_W_S_.isSameOrientation(other.transform_W_S_, orientation_dot_epsilon);
}

bool RobotState::isSensorPoseReached(const RobotState& current, double pos_threshold, double angle_threshold) const {
  return current.transform_W_S_.isPoseReached(transform_W_S_, pos_threshold, angle_threshold);
}

double RobotState::getTimeFromStart() const {
  return time_from_start_;
}

const Eigen::Vector3d& RobotState::getBasePosition() const {
  return transform_W_B_.position;
}

const Eigen::Quaterniond& RobotState::getBaseOrientation() const {
  return transform_W_B_.orientation;
}

const Transform3d& RobotState::getBaseTransform() const {
  return transform_W_B_;
}

const Eigen::Vector3d& RobotState::getSensorPosition() const {
  return transform_W_S_.position;
}

const Eigen::Quaterniond& RobotState::getSensorOrientation() const {
  return transform_W_S_.orientation;
}

double RobotState::getSensorYaw() const {
  return yawFromQuaternion(transform_W_S_.orientation);
}

Eigen::Vector3d RobotState::getSensorEulerAnglesZyx() const {
  return eulerAnglesZyxFromQuaternion(transform_W_S_.orientation);
}

const Transform3d& RobotState::getSensorTransform() const {
  return transform_W_S_;
}

const Eigen::VectorXd& RobotState::getJointPosition() const {
  return joint_position_;
}

void RobotState::setTimeFromStart(double time_from_start) {
  time_from_start_ = time_from_start;
}

void RobotState::setBasePosition(const Eigen::Vector3d& base_position) {
  transform_W_B_.position = base_position;
}

void RobotState::setBaseOrientation(const Eigen::Quaterniond& base_orientation) {
  transform_W_B_.orientation = base_orientation;
}

void RobotState::setBaseTransform(const Transform3d& base_transform) {
  transform_W_B_ = base_transform;
}

void RobotState::setSensorPosition(const Eigen::Vector3d& sensor_position) {
  transform_W_S_.position = sensor_position;
}

void RobotState::setSensorOrientation(const Eigen::Quaterniond& sensor_orientation) {
  transform_W_S_.orientation = sensor_orientation;
}

void RobotState::setSensorTransform(const Transform3d& sensor_transform) {
  transform_W_S_ = sensor_transform;
}

void RobotState::setJointPosition(const Eigen::VectorXd& joint_position) {
  joint_position_ = joint_position;
}

std::string RobotState::toString() const {
  std::stringstream ss;
  ss << "time from start:   " << time_from_start_ << std::endl
     << "base position:     " << transform_W_B_.position.transpose() << std::endl
     << "base quaternion:   " << transform_W_B_.orientation.coeffs().transpose() << std::endl
     << "sensor position:   " << transform_W_S_.position.transpose() << std::endl
     << "sensor quaternion: " << transform_W_S_.orientation.coeffs().transpose() << std::endl
     << "joint position:    " << joint_position_.transpose() << std::endl;
  return ss.str();
}

// Timestamps are remapped so the reversed trajectory starts at t=0.
RobotTrajectory reverseTrajectory(const RobotTrajectory& trajectory) {
  auto final_time_from_start = trajectory.back().getTimeFromStart();

  RobotTrajectory reversed_trajectory;
  for (int i = trajectory.size() - 1; i >= 0; --i) {
    RobotState state = trajectory.at(i);
    state.setTimeFromStart(final_time_from_start - state.getTimeFromStart());
    reversed_trajectory.push_back(state);
  }

  return reversed_trajectory;
}

Eigen::Vector3d trajectoryDepartureDelta(const RobotTrajectory& trajectory) {
  if (trajectory.empty()) {
    throw std::invalid_argument("trajectoryDepartureDelta: trajectory is empty.");
  }

  for (const auto& state : trajectory) {
    if (!state.isSameSensorPosition(trajectory.front())) {
      return state.getSensorPosition() - trajectory.front().getSensorPosition();
    }
  }

  throw std::invalid_argument("trajectoryDepartureDelta: trajectory never leaves its start position.");
}

Eigen::Vector3d trajectoryArrivalDelta(const RobotTrajectory& trajectory) {
  if (trajectory.empty()) {
    throw std::invalid_argument("trajectoryArrivalDelta: trajectory is empty.");
  }

  for (auto it = trajectory.rbegin(); it != trajectory.rend(); ++it) {
    if (!it->isSameSensorPosition(trajectory.back())) {
      return trajectory.back().getSensorPosition() - it->getSensorPosition();
    }
  }

  throw std::invalid_argument("trajectoryArrivalDelta: trajectory never leaves its end position.");
}

int bestAlignedState(const std::vector<RobotState>& states, double yaw, double pitch) {
  int best_index = 0;
  double best_cost = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(states.size()); ++i) {
    const Eigen::Vector2d yaw_pitch = yawPitchFromQuaternion(states[i].getSensorOrientation());
    const double cost = std::abs(angleDifference(yaw_pitch(0), yaw)) + std::abs(yaw_pitch(1) - pitch);
    if (cost < best_cost) {
      best_cost = cost;
      best_index = i;
    }
  }
  return best_index;
}

RobotTrajectory concatenateTrajectory(const RobotTrajectory& lhs, const RobotTrajectory& rhs) {
  if (lhs.empty()) {
    return rhs;
  }

  if (rhs.empty()) {
    return lhs;
  }

  // Continuity check
  if (!lhs.back().isSameSensorPose(rhs.front())) {
    const RobotState& previous = lhs.back();
    const RobotState& next = rhs.front();
    const double position_error = (previous.getSensorPosition() - next.getSensorPosition()).norm();
    const double angle_error = Eigen::AngleAxisd(previous.getSensorOrientation().inverse() * next.getSensorOrientation()).angle();

    std::ostringstream ss;
    ss << "[concatenateTrajectory] Non-continuous trajectory junction before append: position_error=" << position_error
       << " m, angle_error=" << angle_error << " rad, previous_position=[" << previous.getSensorPosition().transpose()
       << "], next_start_position=[" << next.getSensorPosition().transpose() << "], previous_yaw_pitch=["
       << yawPitchFromQuaternion(previous.getSensorOrientation()).transpose() << "], next_start_yaw_pitch=["
       << yawPitchFromQuaternion(next.getSensorOrientation()).transpose() << "].";
    logging::error(ss.str());
    throw std::runtime_error("Trajectories cannot be appended because they are not continuous.");
  }

  // Concatenate the two trajectories
  RobotTrajectory result = lhs;
  result.reserve(lhs.size() + rhs.size() - 1);

  const double time_offset = result.back().getTimeFromStart();
  for (size_t i = 1; i < rhs.size(); ++i) {
    auto state = rhs[i];
    state.setTimeFromStart(state.getTimeFromStart() + time_offset);
    result.push_back(state);
  }

  return result;
}

double getTrajectoryLength(const RobotTrajectory& trajectory) {
  double length = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const auto& prev = trajectory[i - 1];
    const auto& curr = trajectory[i];
    length += (curr.getSensorPosition() - prev.getSensorPosition()).norm();
  }
  return length;
}

void setUniformTrajectoryTimes(RobotTrajectory& trajectory, double dt) {
  for (size_t i = 0; i < trajectory.size(); ++i) {
    trajectory[i].setTimeFromStart(static_cast<double>(i) * dt);
  }
}

void appendMotionState(RobotTrajectory* trajectory, const RobotState& state) {
  if (trajectory->empty() || !state.isSameSensorPose(trajectory->back())) {
    trajectory->push_back(state);
  }
}

void closeTrajectoryAtState(RobotTrajectory* trajectory, const RobotState& final_state) {
  if (trajectory->empty()) {
    throw std::invalid_argument("closeTrajectoryAtState: trajectory is empty.");
  }

  // Overwrite a sub-tolerance residual last state; the first state (the start) is never overwritten.
  if (trajectory->size() > 1 && trajectory->back().isSameSensorPose(final_state)) {
    trajectory->back() = final_state;
  } else {
    trajectory->push_back(final_state);
  }

  for (size_t i = 1; i < trajectory->size(); ++i) {
    if ((*trajectory)[i].isSameSensorPose((*trajectory)[i - 1])) {
      throw std::logic_error(
          "closeTrajectoryAtState: consecutive states at the same pose; splitting there would yield a zero-motion edge.");
    }
  }
}

}  // namespace open_apero
