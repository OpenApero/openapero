#pragma once

/**
 * @file rotate_reverse.h
 * @brief Backtracker that combines rotation-in-place with trajectory reversal.
 */

#include <deque>
#include <string>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/plugin/backtracker/reverse.h"
#include "open_apero_core/plugin/backtracker/rotate_in_place.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace backtracker {

/**
 * @brief Two-strategy backtracker: first does the initial rotation sweep (and
 *        further rotations whenever the reverse queue is empty), otherwise
 *        reverses the most recent forward trajectory.
 */
class RotateReverse : public RotateInPlace {
 public:
  explicit RotateReverse(AgentInterface& agent);

  void resetEpisode() override {
    RotateInPlace::resetEpisode();
    queue_.clear();
  }

  bool recordExecutedEdge(const Edge& edge) override;

  bool buildTrajectory(const RobotState& state, RobotTrajectory* out) override;

  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  static PluginRegistry::Registration<RotateReverse> registration;

  int p_queue_size_ = 100;
  ExecutedTrajectoryQueue queue_;
};

}  // namespace backtracker
}  // namespace open_apero
