#include "open_apero_core/plugin/sensor_model/camera_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "open_apero_core/geometry/transform3d.h"

namespace open_apero {
namespace sensor_model {

CameraModel::CameraModel(AgentInterface& agent) : SensorModel(agent) {}

// Visualize the camera frustum as a wireframe pyramid in RViz.
// Four lines from the sensor origin to the frustum corners, plus the top/bottom/left/right edges of the
// far boundary: arcs on a sphere, straight lines on a plane.
void CameraModel::visualizeSensorView(VisualizationMarkers* markers, const Node* node) {
  VisualizationMarker marker;
  marker.ns = "sensor_view";
  marker.type = VisualizationMarker::LINE_LIST;
  marker.scale.x() = 0.02;
  marker.color.r = 0.4;
  marker.color.g = 0.4;
  marker.color.b = 0.4;
  marker.color.a = 1.0;

  // Get boundary lines
  const bool planar = p_observation_boundary_ == ObservationBoundary::kPlanar;
  int res_x = planar ? 1 : ceil(p_ray_length_ * c_field_of_view_x_ / 0.25);
  int res_y = planar ? 1 : ceil(p_ray_length_ * c_field_of_view_y_ / 0.25);

  std::vector<Eigen::Vector3d> left(res_y + 1);
  std::vector<Eigen::Vector3d> right(res_y + 1);
  std::vector<Eigen::Vector3d> top(res_x + 1);
  std::vector<Eigen::Vector3d> bottom(res_x + 1);
  Eigen::Vector3d direction;

  const Eigen::Vector3d& sensor_position = node->robot_state.getSensorPosition();
  const Eigen::Quaterniond& sensor_orientation = node->robot_state.getSensorOrientation();

  for (int i = 0; i <= res_y; ++i) {
    getDirectionVector(&direction, 0.0, static_cast<double>(i) / static_cast<double>(res_y));
    left[i] = sensor_position + getRayLength(direction) * (sensor_orientation * direction);
    getDirectionVector(&direction, 1.0, static_cast<double>(i) / static_cast<double>(res_y));
    right[i] = sensor_position + getRayLength(direction) * (sensor_orientation * direction);
  }
  for (int i = 0; i <= res_x; ++i) {
    getDirectionVector(&direction, static_cast<double>(i) / static_cast<double>(res_x), 0.0);
    top[i] = sensor_position + getRayLength(direction) * (sensor_orientation * direction);
    getDirectionVector(&direction, static_cast<double>(i) / static_cast<double>(res_x), 1.0);
    bottom[i] = sensor_position + getRayLength(direction) * (sensor_orientation * direction);
  }

  marker.points.push_back(sensor_position);
  marker.points.push_back(left[0]);
  marker.points.push_back(sensor_position);
  marker.points.push_back(left[res_y]);
  marker.points.push_back(sensor_position);
  marker.points.push_back(right[0]);
  marker.points.push_back(sensor_position);
  marker.points.push_back(right[res_y]);
  for (int i = 1; i <= res_x; ++i) {
    marker.points.push_back(top[i - 1]);
    marker.points.push_back(top[i]);
    marker.points.push_back(bottom[i - 1]);
    marker.points.push_back(bottom[i]);
  }
  for (int i = 1; i <= res_y; ++i) {
    marker.points.push_back(left[i - 1]);
    marker.points.push_back(left[i]);
    marker.points.push_back(right[i - 1]);
    marker.points.push_back(right[i]);
  }
  markers->addMarker(marker);
}

// The planar view is a pyramid, the convex hull of the sensor origin and the four far-face corners at
// depth ray_length, so the AABB of those five points is exact. The spherical view is every ray out to
// ray_length, a cone over a spherical cap. Its AABB is exact for any orientation: it holds the sensor
// origin, the far points of the corners, edge midpoints and optical axis, the far point along each
// world axis direction inside the field of view, and each world-axis extreme on the four far-edge arcs.
// Used for fast overlap checks between sensor views and updated map regions.
void CameraModel::getSensorViewBoundary(Eigen::Vector3d& min, Eigen::Vector3d& max, const Transform3d& sensor_transform) {
  Eigen::Vector3d direction;
  if (p_observation_boundary_ == ObservationBoundary::kPlanar) {
    min = sensor_transform.position;
    max = sensor_transform.position;
    for (const double relative_x : {0.0, 1.0}) {
      for (const double relative_y : {0.0, 1.0}) {
        getDirectionVector(&direction, relative_x, relative_y);
        const Eigen::Vector3d corner = sensor_transform.position + getRayLength(direction) * (sensor_transform.orientation * direction);
        min = min.cwiseMin(corner);
        max = max.cwiseMax(corner);
      }
    }
    return;
  }

  std::vector<Eigen::Vector3d> boundary_points{sensor_transform.position};
  for (const double relative_x : {0.0, 0.5, 1.0}) {
    for (const double relative_y : {0.0, 0.5, 1.0}) {
      getDirectionVector(&direction, relative_x, relative_y);
      boundary_points.emplace_back(sensor_transform.position + p_ray_length_ * (sensor_transform.orientation * direction));
    }
  }

  constexpr double kRoundingMargin = 1e-9;
  for (int axis = 0; axis < 3; ++axis) {
    for (const double sign : {-1.0, 1.0}) {
      const Eigen::Vector3d far_point = sensor_transform.position + sign * p_ray_length_ * Eigen::Vector3d::Unit(axis);
      if (pointInSensorView(far_point, sensor_transform, kRoundingMargin)) {
        boundary_points.push_back(far_point);
      }
    }
  }

  // A world-axis extreme can also sit strictly inside one of the four far-edge arcs (the great-circle
  // arcs between adjacent corner directions): where the axis, projected into the arc's plane, falls
  // between the arc's ends.
  std::array<Eigen::Vector3d, 4> corner_directions;
  const std::array<std::pair<double, double>, 4> corner_pixels{{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}};
  for (std::size_t i = 0; i < corner_pixels.size(); ++i) {
    getDirectionVector(&direction, corner_pixels[i].first, corner_pixels[i].second);
    corner_directions[i] = (sensor_transform.orientation * direction).normalized();
  }
  for (std::size_t i = 0; i < corner_directions.size(); ++i) {
    const Eigen::Vector3d& arc_start = corner_directions[i];
    const Eigen::Vector3d& arc_end = corner_directions[(i + 1) % corner_directions.size()];
    const Eigen::Vector3d arc_normal = arc_start.cross(arc_end).normalized();
    for (int axis = 0; axis < 3; ++axis) {
      for (const double sign : {-1.0, 1.0}) {
        const Eigen::Vector3d unit = sign * Eigen::Vector3d::Unit(axis);
        Eigen::Vector3d in_plane = unit - unit.dot(arc_normal) * arc_normal;
        if (in_plane.norm() < kRoundingMargin) {
          continue;
        }
        in_plane.normalize();
        if (arc_start.cross(in_plane).dot(arc_normal) >= 0.0 && in_plane.cross(arc_end).dot(arc_normal) >= 0.0) {
          boundary_points.emplace_back(sensor_transform.position + p_ray_length_ * in_plane);
        }
      }
    }
  }

  min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

  for (const auto& point : boundary_points) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }
}

bool CameraModel::pointInSensorView(const Eigen::Vector3d& point, const Transform3d& sensor_transform, double margin) const {
  // Camera frame per getDirectionVector: +x forward, y and z spanning the image. With a zero margin,
  // points exactly on the frustum edge may be rejected by rounding.
  const Eigen::Vector3d point_camera = sensor_transform.orientation.inverse() * (point - sensor_transform.position);

  const double range = p_observation_boundary_ == ObservationBoundary::kPlanar ? point_camera.x() : point_camera.norm();
  if (point_camera.x() <= -margin || range > p_ray_length_ + margin) {
    return false;
  }
  // The margin is measured perpendicular to each tilted side plane: margin / cos(fov / 2) laterally.
  return std::abs(point_camera.y()) <= point_camera.x() * c_tan_half_field_of_view_x_ + margin * c_sec_half_field_of_view_x_ &&
         std::abs(point_camera.z()) <= point_camera.x() * c_tan_half_field_of_view_y_ + margin * c_sec_half_field_of_view_y_;
}

// Unit pinhole ray for normalized image coordinates; +y left, +z up, (0,0) top-left, (1,1) bottom-right.
void CameraModel::getDirectionVector(Eigen::Vector3d* result, double relative_x, double relative_y) {
  *result = Eigen::Vector3d(p_focal_length_, (0.5 - relative_x) * static_cast<double>(p_resolution_x_),
                            (0.5 - relative_y) * static_cast<double>(p_resolution_y_))
                .normalized();
}

// camera_direction.x() > 0 for every pixel ray, as focal_length > 0.
double CameraModel::getRayLength(const Eigen::Vector3d& camera_direction) const {
  return p_observation_boundary_ == ObservationBoundary::kPlanar ? p_ray_length_ / camera_direction.x() : p_ray_length_;
}

double CameraModel::getObservationBoundaryWidth(double field_of_view, int resolution) const {
  return p_observation_boundary_ == ObservationBoundary::kPlanar ? p_ray_length_ * resolution / p_focal_length_
                                                                 : p_ray_length_ * field_of_view;
}

void CameraModel::setupFromParamMap(Configurable::ParamMap* param_map) {
  SensorModel::setupFromParamMap(param_map);

  setParam<int>(param_map, "resolution_x", &p_resolution_x_, 640);
  setParam<int>(param_map, "resolution_y", &p_resolution_y_, 480);
  setParam<double>(param_map, "focal_length", &p_focal_length_, 320.0);

  std::string observation_boundary_name;
  setParam<std::string>(param_map, "observation_boundary", &observation_boundary_name, "spherical");
  if (observation_boundary_name == "spherical") {
    p_observation_boundary_ = ObservationBoundary::kSpherical;
  } else if (observation_boundary_name == "planar") {
    p_observation_boundary_ = ObservationBoundary::kPlanar;
  } else {
    throw std::runtime_error("CameraModel: unsupported observation_boundary \"" + observation_boundary_name +
                             "\" (expected \"spherical\" or \"planar\")");
  }

  // cache param dependent constants
  c_field_of_view_x_ = 2.0 * atan2(p_resolution_x_, p_focal_length_ * 2.0);
  c_field_of_view_y_ = 2.0 * atan2(p_resolution_y_, p_focal_length_ * 2.0);
  c_tan_half_field_of_view_x_ = std::tan(c_field_of_view_x_ / 2.0);
  c_tan_half_field_of_view_y_ = std::tan(c_field_of_view_y_ / 2.0);
  c_sec_half_field_of_view_x_ = 1.0 / std::cos(c_field_of_view_x_ / 2.0);
  c_sec_half_field_of_view_y_ = 1.0 / std::cos(c_field_of_view_y_ / 2.0);
}

bool CameraModel::checkParamsValid(std::string* error_message) {
  if (p_ray_length_ <= 0.0) {
    *error_message = "ray_length expected > 0.0";
    return false;
  } else if (p_focal_length_ <= 0.0) {
    *error_message = "focal_length expected > 0.0";
    return false;
  } else if (p_resolution_x_ <= 0) {
    *error_message = "resolution_x expected > 0";
    return false;
  } else if (p_resolution_y_ <= 0.0) {
    *error_message = "resolution_y expected > 0";
    return false;
  }
  return true;
}

}  // namespace sensor_model
}  // namespace open_apero
