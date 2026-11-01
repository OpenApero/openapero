#include "open_apero_core/plugin/map/map.h"

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace open_apero {

Map::Map(AgentInterface& agent) : Plugin(agent) {}

bool Map::getUpdatedMapRegion(Eigen::Vector3d* /*updated_map_min*/, Eigen::Vector3d* /*updated_map_max*/) {
  return false;
}

bool Map::getTotalMapRegion(Eigen::Vector3d* /*total_map_min*/, Eigen::Vector3d* /*total_map_max*/) {
  return false;
}

bool Map::getMappedOpticalFrameTransforms(std::vector<Transform3d>* /*transforms*/) {
  return false;
}

void Map::setHitRegions(const GeometryPrimitiveCollection& /*hit_regions*/) {}

void Map::getHitRegions(GeometryPrimitiveCollection& /*hit_regions*/) const {}

void Map::registerHit(const RobotState& /*stopped*/, const RobotState& /*target*/) {}

bool Map::isStateReachable(const RobotState& /*state*/) const {
  return true;
}

bool Map::getGroundHeight(const RobotState& /*state*/, double* /*ground_z*/) const {
  return false;
}

bool Map::saveMap(const std::string& /*filename*/) {
  return false;
}

bool Map::hasMappedFrameNear(const Transform3d& optical_frame, double position_tolerance, double angle_tolerance) {
  std::vector<Transform3d> mapped_optical_frames;
  if (!getMappedOpticalFrameTransforms(&mapped_optical_frames)) {
    return false;
  }
  for (const Transform3d& mapped_optical_frame : mapped_optical_frames) {
    if (mapped_optical_frame.isPoseReached(optical_frame, position_tolerance, angle_tolerance)) {
      return true;
    }
  }
  return false;
}

// Namespaces: hit_region_edges, hit_cylinder_edges, mapped_optical_frames, updated_map_region, total_map_region.
void Map::getVisualization(VisualizationMarkers* markers) {
  GeometryPrimitiveCollection hit_places;
  getHitRegions(hit_places);
  if (!hit_places.triangles.empty()) {
    VisualizationMarker hit_edge_marker;
    hit_edge_marker.ns = "hit_region_edges";
    hit_edge_marker.type = VisualizationMarker::LINE_LIST;
    hit_edge_marker.action = VisualizationMarker::ADD;
    hit_edge_marker.color.r = 0.0;
    hit_edge_marker.color.g = 0.0;
    hit_edge_marker.color.b = 0.0;
    hit_edge_marker.color.a = 1.0;
    hit_edge_marker.scale.x() = 0.03;

    auto add_prism_edge = [&hit_edge_marker](const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
      hit_edge_marker.points.push_back(start);
      hit_edge_marker.points.push_back(end);
    };

    for (auto& prism : hit_places.triangles) {
      const Eigen::Vector3d& A = prism.vertices[0];
      const Eigen::Vector3d& B = prism.vertices[1];
      const Eigen::Vector3d& C = prism.vertices[2];
      const Eigen::Vector3d offset(0.0, 0.0, prism.height);
      const Eigen::Vector3d Ap = A + offset;
      const Eigen::Vector3d Bp = B + offset;
      const Eigen::Vector3d Cp = C + offset;

      add_prism_edge(A, B);
      add_prism_edge(B, C);
      add_prism_edge(C, A);
      add_prism_edge(Ap, Bp);
      add_prism_edge(Bp, Cp);
      add_prism_edge(Cp, Ap);
      add_prism_edge(A, Ap);
      add_prism_edge(B, Bp);
      add_prism_edge(C, Cp);
    }
    markers->addMarker(hit_edge_marker);
  }

  if (!hit_places.cylinders.empty()) {
    constexpr int kCylinderSegments = 24;
    constexpr int kCylinderRibStride = 3;

    VisualizationMarker cylinder_edge_marker;
    cylinder_edge_marker.ns = "hit_cylinder_edges";
    cylinder_edge_marker.type = VisualizationMarker::LINE_LIST;
    cylinder_edge_marker.action = VisualizationMarker::ADD;
    cylinder_edge_marker.color.r = 0.0;
    cylinder_edge_marker.color.g = 0.0;
    cylinder_edge_marker.color.b = 0.0;
    cylinder_edge_marker.color.a = 1.0;
    cylinder_edge_marker.scale.x() = 0.03;

    auto add_cylinder_edge = [&cylinder_edge_marker](const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
      cylinder_edge_marker.points.push_back(start);
      cylinder_edge_marker.points.push_back(end);
    };

    for (auto& cylinder : hit_places.cylinders) {
      const Eigen::Vector3d& bottom_center = cylinder.bottom_center;
      const double radius = cylinder.radius;
      const double height = cylinder.height;

      for (int segment_idx = 0; segment_idx < kCylinderSegments; ++segment_idx) {
        const double angle = 2.0 * M_PI * static_cast<double>(segment_idx) / static_cast<double>(kCylinderSegments);
        const double next_angle = 2.0 * M_PI * static_cast<double>(segment_idx + 1) / static_cast<double>(kCylinderSegments);
        const Eigen::Vector3d radial(radius * std::cos(angle), radius * std::sin(angle), 0.0);
        const Eigen::Vector3d next_radial(radius * std::cos(next_angle), radius * std::sin(next_angle), 0.0);
        const Eigen::Vector3d bottom = bottom_center + radial;
        const Eigen::Vector3d next_bottom = bottom_center + next_radial;
        const Eigen::Vector3d top = bottom + Eigen::Vector3d(0.0, 0.0, height);
        const Eigen::Vector3d next_top = next_bottom + Eigen::Vector3d(0.0, 0.0, height);

        add_cylinder_edge(bottom, next_bottom);
        add_cylinder_edge(top, next_top);
        if (segment_idx % kCylinderRibStride == 0) {
          add_cylinder_edge(bottom, top);
        }
      }
    }
    markers->addMarker(cylinder_edge_marker);
  }

  std::vector<Transform3d> optical_frame_transforms;
  if (getMappedOpticalFrameTransforms(&optical_frame_transforms)) {
    for (size_t i = 0; i < optical_frame_transforms.size(); ++i) {
      VisualizationMarker sensor_arrow_msg;
      sensor_arrow_msg.type = VisualizationMarker::ARROW;
      sensor_arrow_msg.ns = "mapped_optical_frames";
      sensor_arrow_msg.action = VisualizationMarker::ADD;

      sensor_arrow_msg.position = optical_frame_transforms[i].position;
      sensor_arrow_msg.orientation = optical_frame_transforms[i].orientation;
      sensor_arrow_msg.scale.x() = 0.5;
      sensor_arrow_msg.scale.y() = 0.05;
      sensor_arrow_msg.scale.z() = 0.05;
      sensor_arrow_msg.color.r = 0.8;
      sensor_arrow_msg.color.g = 0.8;
      sensor_arrow_msg.color.b = 0.8;
      sensor_arrow_msg.color.a = 0.8;
      markers->addMarker(sensor_arrow_msg);
    }
  }

  {
    Eigen::Vector3d update_map_min, update_map_max;
    if (getUpdatedMapRegion(&update_map_min, &update_map_max)) {
      VisualizationMarker map_update_msg;
      map_update_msg.type = VisualizationMarker::CUBE;
      map_update_msg.ns = "updated_map_region";
      map_update_msg.action = VisualizationMarker::ADD;

      map_update_msg.position = (update_map_min + update_map_max) / 2.0;
      map_update_msg.scale = update_map_max - update_map_min;
      map_update_msg.orientation.x() = 0.0;
      map_update_msg.orientation.y() = 0.0;
      map_update_msg.orientation.z() = 0.0;
      map_update_msg.orientation.w() = 1.0;
      map_update_msg.color.r = 0.1;
      map_update_msg.color.g = 0.6;
      map_update_msg.color.b = 0.8;
      map_update_msg.color.a = 0.5;
      markers->addMarker(map_update_msg);
    }

    Eigen::Vector3d total_map_min, total_map_max;
    if (getTotalMapRegion(&total_map_min, &total_map_max)) {
      VisualizationMarker total_map_msg;
      total_map_msg.type = VisualizationMarker::CUBE;
      total_map_msg.ns = "total_map_region";
      total_map_msg.action = VisualizationMarker::ADD;

      total_map_msg.position = (total_map_min + total_map_max) / 2.0;
      total_map_msg.scale = total_map_max - total_map_min;
      total_map_msg.orientation.x() = 0.0;
      total_map_msg.orientation.y() = 0.0;
      total_map_msg.orientation.z() = 0.0;
      total_map_msg.orientation.w() = 1.0;
      total_map_msg.color.r = 0.1;
      total_map_msg.color.g = 0.6;
      total_map_msg.color.b = 0.8;
      total_map_msg.color.a = 0.2;
      markers->addMarker(total_map_msg);
    }
  }
}

}  // namespace open_apero
