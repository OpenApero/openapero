#pragma once

/**
 * @file eigen_conversion.h
 * @brief Inline conversions between Eigen vectors/quaternions and their
 *        geometry_msgs equivalents.
 */

#include <cassert>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace open_apero {
namespace ros {

/**
 * @brief Convert an Eigen 3D vector to a geometry_msgs::Vector3.
 * @param eigen Input Eigen vector.
 * @param msg Output ROS Vector3 message.
 */
inline void vectorEigenToMsg(const Eigen::Vector3d& eigen, geometry_msgs::Vector3* msg) {
  assert(msg != nullptr);
  msg->x = eigen.x();
  msg->y = eigen.y();
  msg->z = eigen.z();
}

/**
 * @brief Convert an Eigen 3D vector to a geometry_msgs::Point.
 * @param eigen Input Eigen vector.
 * @param msg Output ROS Point message.
 */
inline void vectorEigenToMsg(const Eigen::Vector3d& eigen, geometry_msgs::Point* msg) {
  assert(msg != nullptr);
  msg->x = eigen.x();
  msg->y = eigen.y();
  msg->z = eigen.z();
}

/**
 * @brief Convert a geometry_msgs::Vector3 to an Eigen 3D vector.
 * @param msg Input ROS Vector3 message.
 * @param eigen Output Eigen vector.
 */
inline void vectorEigenFromMsg(const geometry_msgs::Vector3& msg, Eigen::Vector3d* eigen) {
  assert(eigen != nullptr);
  eigen->x() = msg.x;
  eigen->y() = msg.y;
  eigen->z() = msg.z;
}

/**
 * @brief Convert a geometry_msgs::Point to an Eigen 3D vector.
 * @param msg Input ROS Point message.
 * @param eigen Output Eigen vector.
 */
inline void vectorEigenFromMsg(const geometry_msgs::Point& msg, Eigen::Vector3d* eigen) {
  assert(eigen != nullptr);
  eigen->x() = msg.x;
  eigen->y() = msg.y;
  eigen->z() = msg.z;
}

/**
 * @brief Convert an Eigen quaternion to a geometry_msgs::Quaternion.
 * @param eigen Input Eigen quaternion.
 * @param msg Output ROS Quaternion message.
 */
inline void quaternionEigenToMsg(const Eigen::Quaterniond& eigen, geometry_msgs::Quaternion* msg) {
  assert(msg != nullptr);
  msg->x = eigen.x();
  msg->y = eigen.y();
  msg->z = eigen.z();
  msg->w = eigen.w();
}

/**
 * @brief Convert a geometry_msgs::Quaternion to an Eigen quaternion.
 * @param msg Input ROS Quaternion message.
 * @param eigen Output Eigen quaternion.
 */
inline void quaternionEigenFromMsg(const geometry_msgs::Quaternion& msg, Eigen::Quaterniond* eigen) {
  assert(eigen != nullptr);
  eigen->x() = msg.x;
  eigen->y() = msg.y;
  eigen->z() = msg.z;
  eigen->w() = msg.w;
}

}  // namespace ros
}  // namespace open_apero
