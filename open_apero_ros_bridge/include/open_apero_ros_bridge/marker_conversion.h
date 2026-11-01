#pragma once

/**
 * @file marker_conversion.h
 * @brief Inline conversions from open_apero visualization markers to their
 *        visualization_msgs equivalents.
 */

#include <cassert>

#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <open_apero_core/visualization/visualization_markers.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace open_apero {
namespace ros {
namespace marker_conversion_detail {

// The marker types that draw their points.
inline bool isPointListType(int type) {
  switch (type) {
    case VisualizationMarker::POINTS:
    case VisualizationMarker::LINE_STRIP:
    case VisualizationMarker::LINE_LIST:
    case VisualizationMarker::CUBE_LIST:
    case VisualizationMarker::SPHERE_LIST:
    case VisualizationMarker::TRIANGLE_LIST:
      return true;
    default:
      return false;
  }
}

}  // namespace marker_conversion_detail

/**
 * @brief Convert a single internal VisualizationMarker to a ROS Marker message.
 * @param marker Input visualization marker.
 * @param msg Output ROS Marker message.
 */
inline void visualizationMarkerToMsg(const VisualizationMarker& marker, visualization_msgs::Marker* msg) {
  assert(msg != nullptr);
  msg->type = marker.type;
  msg->action = marker.action;
  msg->id = marker.id;
  msg->ns = marker.ns;
  msg->text = marker.text;
  msg->pose.position.x = marker.position.x();
  msg->pose.position.y = marker.position.y();
  msg->pose.position.z = marker.position.z();
  msg->pose.orientation.x = marker.orientation.x();
  msg->pose.orientation.y = marker.orientation.y();
  msg->pose.orientation.z = marker.orientation.z();
  msg->pose.orientation.w = marker.orientation.w();
  msg->scale.x = marker.scale.x();
  msg->scale.y = marker.scale.y();
  msg->scale.z = marker.scale.z();
  msg->color.r = marker.color.r;
  msg->color.g = marker.color.g;
  msg->color.b = marker.color.b;
  msg->color.a = marker.color.a;
  msg->mesh_resource = marker.mesh_resource;
  msg->mesh_use_embedded_materials = marker.mesh_use_embedded_materials;

  msg->points.clear();
  geometry_msgs::Point point;
  for (const Eigen::Vector3d& p : marker.points) {
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    msg->points.push_back(point);
  }
  msg->colors.clear();
  std_msgs::ColorRGBA color;
  for (const Color& c : marker.colors) {
    color.r = c.r;
    color.g = c.g;
    color.b = c.b;
    color.a = c.a;
    msg->colors.push_back(color);
  }
}

/**
 * @brief Append a collection of internal VisualizationMarkers to msg->markers.
 *
 * An ADD of a point-list marker without points is left out: RViz rejects it, and the error marks the whole display.
 * @param markers Input visualization markers collection.
 * @param msg ROS MarkerArray message whose existing markers are kept.
 */
inline void visualizationMarkersToMsg(const VisualizationMarkers& markers, visualization_msgs::MarkerArray* msg) {
  assert(msg != nullptr);
  visualization_msgs::Marker marker;
  for (const VisualizationMarker& m : markers.getMarkers()) {
    if (m.action == VisualizationMarker::ADD && marker_conversion_detail::isPointListType(m.type) && m.points.empty()) {
      continue;
    }
    visualizationMarkerToMsg(m, &marker);
    msg->markers.push_back(marker);
  }
}

}  // namespace ros
}  // namespace open_apero
