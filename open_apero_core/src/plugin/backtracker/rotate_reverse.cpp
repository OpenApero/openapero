#include "open_apero_core/plugin/backtracker/rotate_reverse.h"

#include <string>

#include "open_apero_core/plugin/robot_model/robot_model.h"

namespace open_apero {
namespace backtracker {

PluginRegistry::Registration<RotateReverse> RotateReverse::registration("RotateReverse");

RotateReverse::RotateReverse(AgentInterface& agent) : RotateInPlace(agent) {}

bool RotateReverse::recordExecutedEdge(const Edge& edge) {
  std::string error;
  if (!queue_.record(edge, p_queue_size_, &error)) {
    agent_.printError("RotateReverse: " + error);
    return false;
  }
  return true;
}

bool RotateReverse::buildTrajectory(const RobotState& state, RobotTrajectory* out) {
  if (remaining_init_rotate_angle_ > 0.0 || queue_.empty()) {
    if (!rotate(state, out)) {
      return false;
    }

    debitInitialRotation(*out);
    return true;
  }

  return queue_.popReversed(out);
}

void RotateReverse::setupFromParamMap(Configurable::ParamMap* param_map) {
  RotateInPlace::setupFromParamMap(param_map);
  setParam<int>(param_map, "queue_size", &p_queue_size_, 100);
}

bool RotateReverse::checkParamsValid(std::string* error_message) {
  if (p_queue_size_ <= 0) {
    *error_message = "queue_size expected > 0";
    return false;
  }
  return RotateInPlace::checkParamsValid(error_message);
}

}  // namespace backtracker
}  // namespace open_apero
