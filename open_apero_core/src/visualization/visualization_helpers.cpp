#include "open_apero_core/visualization/visualization_helpers.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/visualization/colormap.h"

namespace open_apero {

namespace {

/// @brief The gain a node is drawn with: what a path can take from it, so 0 when suppressed.
double drawnGain(const Node* node) {
  return node->is_gain_suppressed ? 0.0 : node->getSingleNodeGain();
}

}  // namespace

void getNodeVisualizationMarkers(VisualizationMarkers* markers, const std::vector<Node*>& all_nodes, bool visualize_orientation) {
  if (all_nodes.empty()) {
    return;
  }

  double min_gain = std::numeric_limits<double>::max();
  double max_gain = std::numeric_limits<double>::lowest();

  for (auto node : all_nodes) {
    const double gain = drawnGain(node);
    min_gain = std::min(min_gain, gain);
    max_gain = std::max(max_gain, gain);
  }

  getNodeVisualizationMarkers(markers, all_nodes, visualize_orientation, min_gain, max_gain);
}

void getNodeVisualizationMarkers(VisualizationMarkers* markers, const std::vector<Node*>& all_nodes, bool visualize_orientation,
                                 double min_gain, double max_gain) {
  if (all_nodes.empty()) {
    return;
  }

  constexpr double kPoseArrowLength = 0.35;

  for (int i = 0; i < static_cast<int>(all_nodes.size()); ++i) {
    auto node = all_nodes[i];
    const double gain = drawnGain(node);

    VisualizationMarker node_msg;
    node_msg.ns = "nodes";
    node_msg.position = node->robot_state.getSensorPosition();
    node_msg.orientation = node->robot_state.getSensorOrientation();
    node_msg.action = VisualizationMarker::ADD;

    // Equal gains have no direction to normalize along and take the colormap's low end.
    node_msg.color = getColorFromColormap(gain, min_gain, max_gain > min_gain ? max_gain : min_gain + 1.0, 1.0);

    if (!node->is_traversable) {
      node_msg.color.a = 0.0;
    } else if (gain < 1e-2) {
      node_msg.color.a = 0.1;
    } else {
      node_msg.color.a = 1.0;
    }

    if (node->is_intermediate) {
      node_msg.type = VisualizationMarker::CUBE;

      node_msg.scale.x() = 0.15;
      node_msg.scale.y() = 0.15;
      node_msg.scale.z() = 0.15;
    } else if (visualize_orientation) {
      node_msg.type = VisualizationMarker::ARROW;
      node_msg.scale.x() = kPoseArrowLength;

      if (node->is_at_frontier) {
        node_msg.scale.y() = 0.10;
        node_msg.scale.z() = 0.10;
      } else {
        node_msg.scale.y() = 0.05;
        node_msg.scale.z() = 0.05;
      }
    } else {
      node_msg.type = VisualizationMarker::SPHERE;

      if (node->is_at_frontier) {
        node_msg.scale.x() = 0.40;
        node_msg.scale.y() = 0.40;
        node_msg.scale.z() = 0.40;
      } else {
        node_msg.scale.x() = 0.25;
        node_msg.scale.y() = 0.25;
        node_msg.scale.z() = 0.25;
      }
    }

    markers->addMarker(node_msg);
  }
}

// Rotation-only cluster edges are omitted to keep the travel graph legible.
void getEdgeVisualizationMarkers(VisualizationMarkers* markers, const std::vector<Edge*>& all_edges, bool is_edge_bidirectional) {
  // One dark achromatic tone for light backgrounds; hue stays reserved for value displays.
  const Color edge_tone(0.28, 0.30, 0.36);
  constexpr double kConsideredLineWidth = 0.022;
  constexpr double kUnconsideredLineWidth = 0.009;
  constexpr double kConsideredAlpha = 1.0;
  constexpr double kUnconsideredAlpha = 0.30;
  constexpr double kEdgeArrowLength = 0.20;
  constexpr double kEdgeArrowWidth = 0.045;
  // Arrowhead position along the arc; a bidirectional pair lands at 1/3 and 2/3, never overlapping.
  constexpr double kArrowArcFraction = 1.0 / 3.0;
  // Below this length the arrowhead would be larger than the edge carrying it.
  constexpr double kMinArcLengthForArrow = 0.30;
  constexpr double kEdgeOffsetDistance = 0.05;
  // z lifts above the sensor positions: completed path 0.010 < graph edges < planned path 0.020.
  constexpr double kEdgeZLift = 0.012;
  constexpr double kEdgeArrowZLift = 0.014;

  VisualizationMarker considered_lines;
  considered_lines.orientation.w() = 1.0;
  considered_lines.type = VisualizationMarker::LINE_LIST;
  considered_lines.ns = "edges";
  considered_lines.scale.x() = kConsideredLineWidth;
  considered_lines.color = edge_tone;
  considered_lines.color.a = kConsideredAlpha;
  considered_lines.action = VisualizationMarker::ADD;

  VisualizationMarker unconsidered_lines = considered_lines;
  unconsidered_lines.ns = "edges_unconsidered";
  unconsidered_lines.scale.x() = kUnconsideredLineWidth;
  unconsidered_lines.color.a = kUnconsideredAlpha;

  VisualizationMarker considered_arrows;
  considered_arrows.orientation.w() = 1.0;
  considered_arrows.type = VisualizationMarker::TRIANGLE_LIST;
  considered_arrows.ns = "edge_directions";
  considered_arrows.scale = Eigen::Vector3d(1.0, 1.0, 1.0);
  considered_arrows.color = edge_tone;
  considered_arrows.color.a = kConsideredAlpha;
  considered_arrows.action = VisualizationMarker::ADD;

  VisualizationMarker unconsidered_arrows = considered_arrows;
  unconsidered_arrows.ns = "edge_directions_unconsidered";
  unconsidered_arrows.color.a = kUnconsideredAlpha;

  for (int i = 0; i < static_cast<int>(all_edges.size()); ++i) {
    auto edge = all_edges[i];
    const auto& trajectory = edge->trajectory;
    if (trajectory.size() < 2 || trajectory.front().isSameSensorPosition(trajectory.back())) {
      continue;
    }

    std::vector<Eigen::Vector3d> plot_path;
    plot_path.reserve(trajectory.size());
    for (int j = 0; j < static_cast<int>(trajectory.size()); ++j) {
      plot_path.push_back(trajectory[j].getSensorPosition());
    }

    Eigen::Vector3d offset(0.0, 0.0, kEdgeZLift);
    if (is_edge_bidirectional) {
      Eigen::Vector3d direction = plot_path.back() - plot_path.front();
      direction.normalize();

      // Choose a stable nonparallel axis for the perpendicular offset.
      Eigen::Vector3d reference(0.0, 0.0, 1.0);
      if (std::fabs(direction.z()) > 0.9) {
        reference = Eigen::Vector3d(0.0, 1.0, 0.0);
      }

      Eigen::Vector3d lateral = reference.cross(direction);
      lateral.normalize();
      offset += lateral * kEdgeOffsetDistance;
    }

    for (auto& point : plot_path) {
      point += offset;
    }

    // Every polyline is packed into one shared LINE_LIST per class, so the marker
    // count stays constant instead of growing with the graph.
    VisualizationMarker& lines = edge->is_considered ? considered_lines : unconsidered_lines;
    double arc_length = 0.0;
    for (int j = 1; j < static_cast<int>(plot_path.size()); ++j) {
      lines.points.push_back(plot_path[j - 1]);
      lines.points.push_back(plot_path[j]);
      arc_length += (plot_path[j] - plot_path[j - 1]).norm();
    }

    if (arc_length < kMinArcLengthForArrow) {
      continue;
    }

    // Locate the segment containing the arrowhead, then interpolate inside it: an
    // edge whose trajectory is a single straight segment must still place its
    // arrowhead at the requested fraction rather than at the segment midpoint.
    const double target_arc_length = kArrowArcFraction * arc_length;
    double traversed = 0.0;
    int segment_index = 1;
    for (int j = 1; j < static_cast<int>(plot_path.size()); ++j) {
      const double segment_length = (plot_path[j] - plot_path[j - 1]).norm();
      segment_index = j;
      if (traversed + segment_length >= target_arc_length) {
        break;
      }
      traversed += segment_length;
    }

    const Eigen::Vector3d& segment_start = plot_path[segment_index - 1];
    const Eigen::Vector3d& segment_end = plot_path[segment_index];
    const double segment_length = (segment_end - segment_start).norm();
    const double interpolation = segment_length > 1e-9 ? std::min(1.0, (target_arc_length - traversed) / segment_length) : 0.0;

    // Taking the local tangent rather than the endpoint chord keeps arrowheads on
    // curved edges pointing along the curve.
    Eigen::Vector3d direction = segment_end - segment_start;
    if (direction.norm() < 1e-6) {
      direction = plot_path.back() - plot_path.front();
    }
    direction.normalize();

    Eigen::Vector3d reference(0.0, 0.0, 1.0);
    if (std::fabs(direction.z()) > 0.9) {
      reference = Eigen::Vector3d(0.0, 1.0, 0.0);
    }
    const Eigen::Vector3d perpendicular = direction.cross(reference).normalized() * kEdgeArrowWidth;

    Eigen::Vector3d center = segment_start + (segment_end - segment_start) * interpolation;
    center.z() += kEdgeArrowZLift - kEdgeZLift;
    const Eigen::Vector3d arrow_half_vector = direction * kEdgeArrowLength / 2.0;

    // Wound counter-clockwise seen from above so the face normal points up: the
    // opposite winding renders as an unlit back face, i.e. black whatever the
    // marker color says.
    VisualizationMarker& arrows = edge->is_considered ? considered_arrows : unconsidered_arrows;
    arrows.points.push_back(center + arrow_half_vector);                  // Tip of the arrow
    arrows.points.push_back(center - arrow_half_vector - perpendicular);  // Base corner 1
    arrows.points.push_back(center - arrow_half_vector + perpendicular);  // Base corner 2
  }

  for (const auto* marker : {&considered_lines, &unconsidered_lines, &considered_arrows, &unconsidered_arrows}) {
    if (!marker->points.empty()) {
      markers->addMarker(*marker);
    }
  }
}

// The planned path is raised slightly to avoid z-fighting with graph edges.
void getPlannedPathVisualizationMarkers(const std::vector<Edge*>& path_edges, VisualizationMarkers* markers, const Map* map) {
  if (path_edges.empty()) {
    return;
  }

  VisualizationMarker path_msg;
  path_msg.orientation.w() = 1.0;
  path_msg.type = VisualizationMarker::LINE_LIST;
  path_msg.ns = "planned_path";
  path_msg.scale.x() = 0.06;
  path_msg.scale.y() = 0.06;
  path_msg.color.r = 0.1;
  path_msg.color.g = 1.0;
  path_msg.color.b = 0.0;
  path_msg.color.a = 1.0;
  path_msg.action = VisualizationMarker::ADD;

  VisualizationMarker arrow_msg;
  arrow_msg.type = VisualizationMarker::ARROW;
  arrow_msg.ns = "planned_path_orientations";
  arrow_msg.scale.x() = 0.25;  // Arrow shaft length
  arrow_msg.scale.y() = 0.05;  // Arrow head width
  arrow_msg.scale.z() = 0.05;  // Arrow head height
  arrow_msg.color.r = 0.1;
  arrow_msg.color.g = 1.0;
  arrow_msg.color.b = 0.0;
  arrow_msg.color.a = 1.0;
  arrow_msg.action = VisualizationMarker::ADD;

  for (auto edge : path_edges) {
    auto& trajectory = edge->trajectory;

    for (int j = 1; j < static_cast<int>(trajectory.size()); ++j) {
      Eigen::Vector3d pos_prev = trajectory[j - 1].getSensorPosition();
      Eigen::Vector3d pos_curr = trajectory[j].getSensorPosition();

      pos_prev.z() += 0.02;
      pos_curr.z() += 0.02;

      path_msg.points.push_back(pos_prev);
      path_msg.points.push_back(pos_curr);

      VisualizationMarker arrow = arrow_msg;
      arrow.position = pos_curr;
      arrow.orientation = trajectory[j].getSensorOrientation();
      markers->addMarker(arrow);
    }
  }

  markers->addMarker(path_msg);

  if (map == nullptr) {
    return;
  }

  // The same path draped onto the ground, in a darker green that stays legible on light floors.
  VisualizationMarker ground_msg;
  ground_msg.orientation.w() = 1.0;
  ground_msg.type = VisualizationMarker::LINE_LIST;
  ground_msg.ns = "planned_path_ground";
  ground_msg.scale.x() = 0.06;
  ground_msg.scale.y() = 0.06;
  ground_msg.color.r = 11.0 / 255.0;
  ground_msg.color.g = 94.0 / 255.0;
  ground_msg.color.b = 30.0 / 255.0;
  ground_msg.color.a = 1.0;
  ground_msg.action = VisualizationMarker::ADD;
  constexpr double kGroundLift = 0.03;

  for (auto edge : path_edges) {
    auto& trajectory = edge->trajectory;

    // Consecutive states share a lookup: state j is the segment end at j and the
    // segment start at j+1.
    bool prev_resolved = false;
    double prev_ground_z = 0.0;
    for (int j = 0; j < static_cast<int>(trajectory.size()); ++j) {
      double ground_z = 0.0;
      const bool resolved = map->getGroundHeight(trajectory[j], &ground_z);
      if (resolved && prev_resolved) {
        Eigen::Vector3d pos_prev = trajectory[j - 1].getSensorPosition();
        Eigen::Vector3d pos_curr = trajectory[j].getSensorPosition();
        pos_prev.z() = prev_ground_z + kGroundLift;
        pos_curr.z() = ground_z + kGroundLift;
        ground_msg.points.push_back(pos_prev);
        ground_msg.points.push_back(pos_curr);
      }
      prev_resolved = resolved;
      prev_ground_z = ground_z;
    }
  }

  if (!ground_msg.points.empty()) {
    markers->addMarker(ground_msg);
  }
}

}  // namespace open_apero
