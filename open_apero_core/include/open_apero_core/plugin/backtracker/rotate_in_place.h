#pragma once

/**
 * @file rotate_in_place.h
 * @brief Backtracker that produces a rotation-in-place recovery trajectory.
 */

#include <cmath>
#include <string>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/plugin/backtracker/backtracker.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"

namespace open_apero {
namespace backtracker {

/**
 * @brief Rotates the robot in place by a configurable step angle. Tracks an
 *        initial-rotation budget so isInitialExplorationDone() can flag when
 *        the start-of-episode sweep is finished.
 */
class RotateInPlace : public Backtracker {
 public:
  explicit RotateInPlace(AgentInterface& agent);  // NOLINT(google-runtime-references)

  void resetEpisode() override { remaining_init_rotate_angle_ = p_initial_rotate_angle_; }

  bool buildTrajectory(const RobotState& state, RobotTrajectory* out) override;

  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Validates rotation_step > 0 and initial_rotate_angle >= 0. */
  bool checkParamsValid(std::string* error_message) override;

  bool isInitialExplorationDone() override;

 protected:
  static PluginRegistry::Registration<RotateInPlace> registration;

  /** @brief Build a single rotation-step trajectory starting from @p state. */
  virtual bool rotate(const RobotState& state, RobotTrajectory* out);

  /**
   * @brief Debit the initial-rotation budget by the yaw swept along @p trajectory.
   *
   * Sums per-step yaw differences, so a sweep beyond pi is debited in full.
   * Precondition: consecutive states are less than pi apart in yaw.
   */
  void debitInitialRotation(const RobotTrajectory& trajectory);

  double p_rotation_step_ = M_PI;
  double p_initial_rotate_angle_ = 2 * M_PI;

  double remaining_init_rotate_angle_ = 2 * M_PI;
};

}  // namespace backtracker
}  // namespace open_apero
