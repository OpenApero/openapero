#include "open_apero_core/plugin/evaluator/naive_evaluator.h"

#include <memory>

namespace open_apero {
namespace evaluator {

PluginRegistry::Registration<NaiveEvaluator> NaiveEvaluator::registration("NaiveEvaluator");

NaiveEvaluator::NaiveEvaluator(AgentInterface& agent) : SimulatedSensorEvaluator(agent) {}

// The map's explored volume, not the summed planning gain.
double NaiveEvaluator::getRealAccumulatedGain() const {
  return map_->getExploredVolume();
}

bool NaiveEvaluator::evaluateFromVisibleVoxels(Node* node) {
  node->gain_anchors.clear();
  if (!node->info) {
    return false;
  }

  SimulatedSensorInfo* info = node->info->as<SimulatedSensorInfo>();
  if (info == nullptr) {
    return false;
  }

  double total_gain = 0.0;
  for (auto& [voxel_center, voxel_info] : info->visible_voxels) {
    if (voxel_info.occupancy == Occupancy::kUnknown) {
      voxel_info.gain = 1.0;
      total_gain += 1.0;
    }
  }

  // Unknown voxels contribute one unit each; known voxels retain zero gain.
  node->gain_anchors = {std::make_shared<GainAnchor>(total_gain)};
  return true;
}

}  // namespace evaluator
}  // namespace open_apero
