#include "open_apero_core/plugin/evaluator/simple_yaw_planning_evaluator.h"

#include <algorithm>
#include <vector>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/visualization/colormap.h"

namespace open_apero {
namespace evaluator {

PluginRegistry::Registration<SimpleYawPlanningEvaluator> SimpleYawPlanningEvaluator::registration("SimpleYawPlanningEvaluator");

SimpleYawPlanningEvaluator::SimpleYawPlanningEvaluator(AgentInterface& agent) : YawPlanningEvaluator(agent) {}

void SimpleYawPlanningEvaluator::setupFromParamMap(Configurable::ParamMap* param_map) {
  setParam<bool>(param_map, "visualize_followup", &p_visualize_followup_, true);
  YawPlanningEvaluator::setupFromParamMap(param_map);
}

void SimpleYawPlanningEvaluator::getVisualization(VisualizationMarkers* markers, const Node* node) {
  const YawPlanningInfo* info = node->info ? node->info->as<YawPlanningInfo>() : nullptr;
  if (info == nullptr) {
    if (p_visualize_followup_) {
      following_evaluator_->getVisualization(markers, node);
    }
    return;
  }

  // Simple version: visualize facing of endpoints of trajectories, colored with
  // value (highest green to lowest red)
  double max_value = info->orientations[0]->getSingleNodeGain();
  double min_value = info->orientations[0]->getSingleNodeGain();

  for (int i = 1; i < static_cast<int>(info->orientations.size()); ++i) {
    auto gain = info->orientations[i]->getSingleNodeGain();
    max_value = std::max(max_value, gain);
    min_value = std::min(min_value, gain);
  }

  for (int i = 0; i < static_cast<int>(info->orientations.size()); ++i) {
    // If this orientation has been selected, remove from the visualization
    if (info->visited_orientation[i]) {
      // the orientation has been selected
      continue;
    }

    // Setup marker message
    VisualizationMarker marker;
    marker.ns = "yaw_candidate_gains";
    marker.position = info->orientations[i]->robot_state.getSensorPosition();
    marker.orientation = info->orientations[i]->robot_state.getSensorOrientation();
    marker.type = VisualizationMarker::ARROW;
    marker.scale.x() = 0.6;
    marker.scale.y() = 0.07;
    marker.scale.z() = 0.07;
    marker.action = VisualizationMarker::ADD;

    // Color according to relative value (blue when indifferent)
    if (max_value != min_value) {
      marker.color = getColorFromColormap(info->orientations[i]->getSingleNodeGain(), min_value, max_value, 0.4);
    } else {
      marker.color.r = 0.3;
      marker.color.g = 0.3;
      marker.color.b = 1.0;
      marker.color.a = 0.4;
    }

    // If this orientation is active, use large scale
    if (i == info->active_orientation) {
      marker.scale.y() = 0.14;
      marker.scale.z() = 0.14;
      marker.color.a = 1.0;
    }
    markers->addMarker(marker);
  }

  // Followup
  if (p_visualize_followup_) {
    following_evaluator_->getVisualization(markers, info->orientations[info->active_orientation].get());
  }
}

double SimpleYawPlanningEvaluator::getRealAccumulatedGain() const {
  return following_evaluator_->getRealAccumulatedGain();
}

}  // namespace evaluator
}  // namespace open_apero
