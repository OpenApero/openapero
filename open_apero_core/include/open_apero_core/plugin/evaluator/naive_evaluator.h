#pragma once

/**
 * @file naive_evaluator.h
 * @brief Simple evaluator that counts unobserved visible voxels as gain.
 */

#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/evaluator/simulated_sensor_evaluator.h"

namespace open_apero {
namespace evaluator {

/**
 * @brief Evaluates nodes by counting the number of yet unobserved visible voxels.
 */
class NaiveEvaluator : public SimulatedSensorEvaluator {
 public:
  /** @brief Constructs a NaiveEvaluator and binds it to the agent. */
  explicit NaiveEvaluator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /**
   * @brief Returns the map's explored volume.
   * @return Explored volume [m^3].
   */
  double getRealAccumulatedGain() const override;

 protected:
  static PluginRegistry::Registration<NaiveEvaluator> registration;

  /**
   * @brief Computes gain by counting unobserved visible voxels at the node.
   * @param node The node to evaluate.
   * @return true on success.
   */
  bool evaluateFromVisibleVoxels(Node* node) override;
};

}  // namespace evaluator
}  // namespace open_apero
