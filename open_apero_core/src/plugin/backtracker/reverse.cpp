#include "open_apero_core/plugin/backtracker/reverse.h"

#include <string>

#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace backtracker {

bool ExecutedTrajectoryQueue::record(const Edge& edge, int capacity, std::string* error) {
  if (edge.is_backtrack) {
    return true;
  }

  if (edge.trajectory.size() < 2) {
    *error = "the executed trajectory has fewer than 2 states";
    return false;
  }

  if (static_cast<int>(trajectories_.size()) >= capacity) {
    trajectories_.pop_front();
  }
  trajectories_.push_back(edge.trajectory);
  return true;
}

bool ExecutedTrajectoryQueue::popReversed(RobotTrajectory* out) {
  if (trajectories_.empty()) {
    return false;
  }
  *out = reverseTrajectory(trajectories_.back());
  trajectories_.pop_back();
  return true;
}

PluginRegistry::Registration<Reverse> Reverse::registration("Reverse");

Reverse::Reverse(AgentInterface& agent) : Backtracker(agent) {}

bool Reverse::recordExecutedEdge(const Edge& edge) {
  std::string error;
  if (!queue_.record(edge, p_queue_size_, &error)) {
    agent_.printError("Reverse: " + error);
    return false;
  }
  return true;
}

bool Reverse::buildTrajectory(const RobotState& /*state*/, RobotTrajectory* out) {
  if (!queue_.popReversed(out)) {
    agent_.printError("Reverse: no executed trajectory left to reverse; consider a backtracker that can also rotate in place.");
    return false;
  }
  return true;
}

void Reverse::setupFromParamMap(Configurable::ParamMap* param_map) {
  setParam<int>(param_map, "queue_size", &p_queue_size_, 100);
}

bool Reverse::checkParamsValid(std::string* error_message) {
  if (p_queue_size_ <= 0) {
    *error_message = "queue_size expected > 0";
    return false;
  }
  return Backtracker::checkParamsValid(error_message);
}

}  // namespace backtracker
}  // namespace open_apero
