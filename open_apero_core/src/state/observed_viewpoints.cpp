#include "open_apero_core/state/observed_viewpoints.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/kdtree.h"

namespace open_apero {

// The dynamic adaptor indexes whatever the store holds when it is constructed, so it is only ever built over an
// empty one.
ObservedViewpoints::ObservedViewpoints() : kdtree_(std::make_unique<ViewpointKdTree>(3, data_)) {}

bool ObservedViewpoints::isValidThresholdPair(double position_threshold, double angle_threshold, std::string* why) {
  if (position_threshold == 0.0 && angle_threshold == 0.0) {
    return true;
  }
  // A radius of at most 0.01 reads as unbounded in Generator::findNearbyNodesInIndex.
  if (!(position_threshold > 1e-2)) {
    if (why) *why = "observed_viewpoint_position_threshold must be > 0.01, or 0.0 with observed_viewpoint_angle_threshold 0.0";
    return false;
  }
  if (!(angle_threshold > 0.0)) {
    if (why) *why = "observed_viewpoint_angle_threshold must be > 0, or 0.0 with observed_viewpoint_position_threshold 0.0";
    return false;
  }
  return true;
}

void ObservedViewpoints::setThresholds(double position_threshold, double angle_threshold) {
  std::string why;
  if (!isValidThresholdPair(position_threshold, angle_threshold, &why)) {
    throw std::invalid_argument("ObservedViewpoints: " + why);
  }
  position_threshold_ = position_threshold;
  angle_threshold_ = angle_threshold;
}

void ObservedViewpoints::add(const Transform3d& sensor_pose) {
  data_.points.push_back(sensor_pose.position);
  data_.orientations.push_back(sensor_pose.orientation);
  const std::size_t index = data_.points.size() - 1;
  kdtree_->addPoints(index, index);
}

bool ObservedViewpoints::isObserved(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) const {
  return isObserved(position, orientation, angle_threshold_);
}

bool ObservedViewpoints::isObserved(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double angle_threshold) const {
  if (!isEnabled() || data_.points.empty()) {
    return false;
  }
  const std::vector<std::size_t> indices = findPointsWithin(*kdtree_, position, position_threshold_);
  return std::any_of(indices.begin(), indices.end(), [this, &orientation, angle_threshold](std::size_t index) {
    return geodesicAngle(orientation, data_.orientations[index]) < angle_threshold;
  });
}

void ObservedViewpoints::getViewpointIndicesNear(const Eigen::Vector3d& position, double radius, std::vector<std::size_t>* indices) const {
  indices->clear();
  if (!isEnabled() || data_.points.empty() || !(radius > 0.0)) {
    return;
  }
  *indices = findPointsWithin(*kdtree_, position, radius);
  std::sort(indices->begin(), indices->end());
}

Transform3d ObservedViewpoints::getViewpoint(std::size_t index) const {
  return Transform3d{data_.points.at(index), data_.orientations.at(index)};
}

void ObservedViewpoints::clear() {
  data_.points.clear();
  data_.orientations.clear();
  kdtree_ = std::make_unique<ViewpointKdTree>(3, data_);
}

}  // namespace open_apero
