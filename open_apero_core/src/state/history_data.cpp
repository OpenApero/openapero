#include "open_apero_core/state/history_data.h"

#include <cmath>
#include <cstddef>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/colormap.h"

namespace open_apero {

void HistoryData::resetEpisode() {
  past_nodes_.clear();
  past_trajectory_.clear();
  past_costs_.clear();
  observed_viewpoints_.clear();
}

// past_nodes_ keeps segment endpoints, past_trajectory_ the dense path, past_costs_ the cumulative cost per state.
void HistoryData::addVisitedEdge(const Edge& edge) {
  const RobotTrajectory& trajectory = edge.trajectory;

  past_nodes_ = concatenateTrajectory(past_nodes_, {trajectory.front(), trajectory.back()});
  past_trajectory_ = concatenateTrajectory(past_trajectory_, trajectory);

  if (past_costs_.empty()) {
    past_costs_.push_back(0.0);
  }

  double last_cost = past_costs_.back();
  for (int i = 1; i < static_cast<int>(trajectory.size()); ++i) {
    past_costs_.push_back(last_cost + edge.cost * i / (trajectory.size() - 1));
  }
}

// Consecutive duplicates are dropped: the map may report the same frame on two harvests.
void HistoryData::addObservedPose(const Transform3d& sensor_pose) {
  const std::size_t num_stored = observed_viewpoints_.size();
  if (num_stored > 0 && observed_viewpoints_.getViewpoint(num_stored - 1).isSamePose(sensor_pose)) {
    return;
  }
  observed_viewpoints_.add(sensor_pose);
}

// Visualize the completed trajectory as a colored line strip.
// If total_cost_budget is finite, color transitions from start to end based on accumulated cost
// relative to the budget. If total_cost_budget is infinite, renders in a flat gray.
void HistoryData::getVisualization(VisualizationMarkers* markers, double total_cost_budget) {
  if (past_trajectory_.size() < 2) {
    return;
  }

  // Continuously increment the already traveled path
  VisualizationMarker msg;
  msg.orientation.w() = 1.0;
  msg.type = VisualizationMarker::LINE_LIST;
  msg.ns = "completed_path";
  msg.action = VisualizationMarker::ADD;
  msg.scale.x() = 0.12;
  msg.scale.y() = 0.12;

  const bool use_flat_color = std::isinf(total_cost_budget);

  for (int i = 0; i + 1 < static_cast<int>(past_trajectory_.size()); ++i) {
    auto pos_curr = past_trajectory_[i].getSensorPosition();
    auto pos_next = past_trajectory_[i + 1].getSensorPosition();

    // Add a small offset to the path to avoid z-fighting
    pos_curr.z() += 0.01;
    pos_next.z() += 0.01;

    Color color_curr = use_flat_color ? Color::gray() : getTrajectoryColor(past_costs_[i], 0.0, total_cost_budget, 1.0);
    Color color_next = use_flat_color ? Color::gray() : getTrajectoryColor(past_costs_[i + 1], 0.0, total_cost_budget, 1.0);

    msg.points.push_back(pos_curr);
    msg.colors.push_back(color_curr);

    msg.points.push_back(pos_next);
    msg.colors.push_back(color_next);
  }

  markers->addMarker(msg);
}

}  // namespace open_apero
