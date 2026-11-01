#include "open_apero_core/plugin/sensor_model/lidar_model.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "open_apero_core/geometry/transform3d.h"

namespace open_apero {
namespace sensor_model {

LidarModel::LidarModel(AgentInterface& agent) : SensorModel(agent) {}

void LidarModel::visualizeSensorView(VisualizationMarkers* markers, const Node* node) {
  VisualizationMarker marker;
  marker.ns = "sensor_view";
  marker.type = VisualizationMarker::LINE_LIST;
  marker.scale.x() = 0.02;
  marker.color.r = 0.4;
  marker.color.g = 0.4;
  marker.color.b = 0.4;
  marker.color.a = 1.0;
  // Get boundary lines
  int res_x = ceil(p_ray_length_ * p_field_of_view_x_ / 0.25);
  int res_y = ceil(p_ray_length_ * p_field_of_view_y_ / 0.25);
  std::vector<Eigen::Vector3d> left(res_y + 1);
  std::vector<Eigen::Vector3d> right(res_y + 1);
  std::vector<Eigen::Vector3d> top(res_x + 1);
  std::vector<Eigen::Vector3d> bottom(res_x + 1);
  Eigen::Vector3d direction;

  const Eigen::Vector3d& sensor_position = node->robot_state.getSensorPosition();
  const Eigen::Quaterniond& sensor_orientation = node->robot_state.getSensorOrientation();

  for (int i = 0; i <= res_y; ++i) {
    getDirectionVector(&direction, 0.0, static_cast<double>(i) / static_cast<double>(res_y));
    left[i] = sensor_position + p_ray_length_ * (sensor_orientation * direction);
    getDirectionVector(&direction, 1.0, static_cast<double>(i) / static_cast<double>(res_y));
    right[i] = sensor_position + p_ray_length_ * (sensor_orientation * direction);
  }
  for (int i = 0; i <= res_x; ++i) {
    getDirectionVector(&direction, static_cast<double>(i) / static_cast<double>(res_x), 0.0);
    top[i] = sensor_position + p_ray_length_ * (sensor_orientation * direction);
    getDirectionVector(&direction, static_cast<double>(i) / static_cast<double>(res_x), 1.0);
    bottom[i] = sensor_position + p_ray_length_ * (sensor_orientation * direction);
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

// A scan of at least half a turn, taken as level, reaches ray_length horizontally around the sensor and
// the vertical cone's height above and below it. A narrower scan is bounded by the origin and its far
// boundary, sampled as visualizeSensorView draws it, plus the far middle row.
void LidarModel::getSensorViewBoundary(Eigen::Vector3d& min, Eigen::Vector3d& max, const Transform3d& sensor_transform) {
  if (p_field_of_view_x_ >= M_PI) {
    const Eigen::Vector3d extent(p_ray_length_, p_ray_length_, p_ray_length_ * std::sin(p_field_of_view_y_ / 2.0));
    min = sensor_transform.position - extent;
    max = sensor_transform.position + extent;
    return;
  }

  std::vector<Eigen::Vector3d> boundary_points{sensor_transform.position};
  Eigen::Vector3d direction;
  const auto add_far_point = [&](double relative_x, double relative_y) {
    getDirectionVector(&direction, relative_x, relative_y);
    boundary_points.emplace_back(sensor_transform.position + p_ray_length_ * (sensor_transform.orientation * direction));
  };
  const int res_x = static_cast<int>(std::ceil(p_ray_length_ * p_field_of_view_x_ / 0.25));
  const int res_y = static_cast<int>(std::ceil(p_ray_length_ * p_field_of_view_y_ / 0.25));
  for (int i = 0; i <= res_x; ++i) {
    const double relative_x = static_cast<double>(i) / static_cast<double>(res_x);
    for (const double relative_y : {0.0, 0.5, 1.0}) {
      add_far_point(relative_x, relative_y);
    }
  }
  for (int i = 0; i <= res_y; ++i) {
    const double relative_y = static_cast<double>(i) / static_cast<double>(res_y);
    add_far_point(0.0, relative_y);
    add_far_point(1.0, relative_y);
  }

  min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

  for (const auto& point : boundary_points) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }
}

void LidarModel::getDirectionVector(Eigen::Vector3d* result, double relative_x, double relative_y) {
  double phi = (0.5 - relative_x) * p_field_of_view_x_;
  double theta = M_PI / 2.0 + (relative_y - 0.5) * p_field_of_view_y_;
  *result = Eigen::Vector3d(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

void LidarModel::setupFromParamMap(Configurable::ParamMap* param_map) {
  SensorModel::setupFromParamMap(param_map);
  setParam<int>(param_map, "resolution_x", &p_resolution_x_, 1024);
  setParam<int>(param_map, "resolution_y", &p_resolution_y_, 128);
  setParam<double>(param_map, "field_of_view_x", &p_field_of_view_x_, 360.0);
  setParam<double>(param_map, "field_of_view_y", &p_field_of_view_y_, 30.0);

  p_field_of_view_x_ *= M_PI / 180.0;
  p_field_of_view_y_ *= M_PI / 180.0;
}

bool LidarModel::checkParamsValid(std::string* error_message) {
  if (p_ray_length_ <= 0.0) {
    *error_message = "ray_length expected > 0.0";
    return false;
  } else if (p_field_of_view_x_ <= 0.0) {
    *error_message = "field_of_view_x expected > 0.0";
    return false;
  } else if (p_field_of_view_y_ <= 0.0) {
    *error_message = "field_of_view_y expected > 0.0";
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
