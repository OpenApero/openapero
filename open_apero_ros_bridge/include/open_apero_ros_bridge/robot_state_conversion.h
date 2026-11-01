#pragma once

/**
 * @file robot_state_conversion.h
 * @brief Inline conversions between open_apero's RobotState and its ROS message
 *        representation, plus construction from separate TF transforms.
 */

#include <cassert>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Transform.h>
#include <open_apero_core/common/utils.h>
#include <open_apero_core/state/robot_state.h>
#include <open_apero_msgs/RobotState.h>

#include "open_apero_ros_bridge/eigen_conversion.h"

namespace open_apero {
namespace ros {

/**
 * @brief Convert an internal RobotState to its ROS message representation.
 * @param robot_state Input robot state.
 * @param msg Output ROS RobotState message.
 */
inline void robotStateToMsg(const RobotState& robot_state, open_apero_msgs::RobotState* msg) {
  assert(msg != nullptr);

  msg->time_from_start.fromSec(robot_state.getTimeFromStart());

  vectorEigenToMsg(robot_state.getBasePosition(), &(msg->base_transform.translation));
  quaternionEigenToMsg(robot_state.getBaseOrientation(), &(msg->base_transform.rotation));

  vectorEigenToMsg(robot_state.getSensorPosition(), &(msg->sensor_transform.translation));
  quaternionEigenToMsg(robot_state.getSensorOrientation(), &(msg->sensor_transform.rotation));

  const Eigen::VectorXd& joint_position = robot_state.getJointPosition();
  msg->joint_position.assign(joint_position.data(), joint_position.data() + joint_position.size());
}

/**
 * @brief Convert a ROS RobotState message to an internal RobotState.
 *
 * Orientations are validated on the way in: every correct publisher sends unit
 * quaternions, so a non-unit one is that publisher's bug — rejected here with
 * the message, never silently normalized into something plausible.
 *
 * @param msg Input ROS RobotState message.
 * @param robot_state Output robot state; unspecified contents on failure.
 * @return False if either quaternion is non-finite or not unit-norm.
 */
inline bool robotStateFromMsg(const open_apero_msgs::RobotState& msg, RobotState* robot_state) {
  assert(robot_state != nullptr);

  robot_state->setTimeFromStart(msg.time_from_start.toSec());

  Eigen::Vector3d temp_pos;
  Eigen::Quaterniond temp_quat;

  vectorEigenFromMsg(msg.base_transform.translation, &temp_pos);
  quaternionEigenFromMsg(msg.base_transform.rotation, &temp_quat);
  if (!isUnitQuaternion(temp_quat)) {
    return false;
  }
  robot_state->setBasePosition(temp_pos);
  robot_state->setBaseOrientation(temp_quat);

  vectorEigenFromMsg(msg.sensor_transform.translation, &temp_pos);
  quaternionEigenFromMsg(msg.sensor_transform.rotation, &temp_quat);
  if (!isUnitQuaternion(temp_quat)) {
    return false;
  }
  robot_state->setSensorPosition(temp_pos);
  robot_state->setSensorOrientation(temp_quat);

  robot_state->setJointPosition(Eigen::Map<const Eigen::VectorXd>(msg.joint_position.data(), msg.joint_position.size()));
  return true;
}

/**
 * @brief Construct an internal RobotState from separate base and sensor TF transforms.
 *
 * Same orientation validation contract as the message overload above.
 *
 * @param base_transform Base link transform in map frame.
 * @param sensor_transform Sensor (camera) transform in map frame.
 * @param robot_state Output robot state; unspecified contents on failure.
 * @return False if either quaternion is non-finite or not unit-norm.
 */
inline bool robotStateFromMsg(const geometry_msgs::Transform& base_transform, const geometry_msgs::Transform& sensor_transform,
                              RobotState* robot_state) {
  assert(robot_state != nullptr);

  Eigen::Vector3d temp_pos;
  Eigen::Quaterniond temp_quat;

  vectorEigenFromMsg(base_transform.translation, &temp_pos);
  quaternionEigenFromMsg(base_transform.rotation, &temp_quat);
  if (!isUnitQuaternion(temp_quat)) {
    return false;
  }
  robot_state->setBasePosition(temp_pos);
  robot_state->setBaseOrientation(temp_quat);

  vectorEigenFromMsg(sensor_transform.translation, &temp_pos);
  quaternionEigenFromMsg(sensor_transform.rotation, &temp_quat);
  if (!isUnitQuaternion(temp_quat)) {
    return false;
  }
  robot_state->setSensorPosition(temp_pos);
  robot_state->setSensorOrientation(temp_quat);
  return true;
}

}  // namespace ros
}  // namespace open_apero
