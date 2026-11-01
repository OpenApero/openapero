#pragma once

/**
 * @file points_evaluator.h
 * @brief Evaluator that computes gain from discrete collectible point
 *        spheres in the environment (e.g., rings to collect).
 */

#include <string>

#include <open_apero_core/gain/gain_anchor.h>
#include <open_apero_core/graph/tree_node.h>
#include <open_apero_core/plugin/evaluator/simulated_sensor_evaluator.h>
#include <ros/ros.h>

namespace open_apero {
namespace evaluator {

/**
 * @brief Evaluator that assigns gain to nodes based on proximity to discrete
 *        collectible point spheres scattered in the environment.
 *
 * Points go from kUnseen to kObserved as the robot maps them, and from kUnseen or
 * kObserved to kCollected as it passes within their radius. Gain is awarded only for kObserved points
 * within collection radius.
 */
class PointsEvaluator : public SimulatedSensorEvaluator {
 public:
  /// @brief Lifecycle status of a point sphere.
  enum class PointStatus { kUnseen, kObserved, kCollected };

  /// Spherical collectible with gain and lifecycle state.
  struct PointSphere : public GainAnchor {
    Eigen::Vector3d position;
    double radius;
    PointStatus status = PointStatus::kUnseen;
    int id;

    /**
     * @brief Construct a PointSphere.
     * @param gain Gain value awarded upon collection.
     * @param x X position.
     * @param y Y position.
     * @param z Z position.
     * @param radius Collection radius.
     * @param status Initial status.
     * @param id Unique identifier.
     */
    PointSphere(double gain, double x, double y, double z, double radius, PointStatus status, int id)
        : GainAnchor(gain), position(x, y, z), radius(radius), status(status), id(id) {}
  };

  /**
   * @brief Construct the evaluator.
   * @param agent Reference to the parent agent interface.
   */
  explicit PointsEvaluator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  void resetEpisode() override {
    SimulatedSensorEvaluator::resetEpisode();
    for (auto& point : points_) {
      point->as<PointSphere>()->status = PointStatus::kUnseen;
    }
  }

  /// @brief Return the total gain from all kCollected points.
  double getRealAccumulatedGain() const override;

  /**
   * @brief Read evaluator parameters and load point definitions from file.
   * @param param_map Pointer to the parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Update point statuses (kObserved/kCollected) based on the traversed edge.
   * @param reached_node   The node the robot just navigated to.
   * @param traversed_edge Edge used to reach the node, or nullptr.
   */
  void onNodeReached(Node* reached_node, const Edge* traversed_edge) override;

  /**
   * @brief Publish visualization markers and collectible point status for a given node.
   * @param markers Output visualization markers.
   * @param node The node to visualize.
   */
  void getVisualization(VisualizationMarkers* markers, const Node* node) override;

 protected:
  static PluginRegistry::Registration<PointsEvaluator> registration;

  bool evaluateFromVisibleVoxels(Node* node) override;

  void loadPointsFromFile(const std::string& filename);

  std::string p_points_file_;

  GainAnchorPtrVector points_;

  ::ros::Publisher collectible_point_pub_;
};

}  // namespace evaluator
}  // namespace open_apero
