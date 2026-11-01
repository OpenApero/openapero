#pragma once

/**
 * @file simple_yaw_planning_evaluator.h
 * @brief Yaw planning evaluator that scores the robot's candidate orientations and keeps the best one.
 */

#include "open_apero_core/plugin/evaluator/yaw_planning_evaluator.h"

namespace open_apero {
namespace evaluator {

/**
 * @brief Scores the robot's candidate orientations (RobotModel::sampleOrientations) through the
 *        YawPlanningEvaluator base; adds the visualization and getRealAccumulatedGain().
 */
class SimpleYawPlanningEvaluator : public YawPlanningEvaluator {
 public:
  /** @brief Constructs a SimpleYawPlanningEvaluator and binds it to the agent. */
  explicit SimpleYawPlanningEvaluator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /** @brief Visualizes the gain of the best yaw orientation found for the node. */
  void getVisualization(VisualizationMarkers* markers, const Node* node) override;

  /**
   * @brief Returns the total accumulated gain from the following evaluator.
   * @return Accumulated gain value.
   */
  double getRealAccumulatedGain() const override;

  /** @brief Loads visualize_followup, then the base keys; the following evaluator comes from following_evaluator_args. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

 protected:
  static PluginRegistry::Registration<SimpleYawPlanningEvaluator> registration;

  // params
  bool p_visualize_followup_ = true;  // true: also visualize the gain of the best orientation
};

}  // namespace evaluator
}  // namespace open_apero
