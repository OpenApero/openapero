#pragma once

/**
 * @file ros_agent.h
 * @brief ROS-integrated agent for active-perception exploration.
 */

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <open_apero_core/agent/agent.h>
#include <open_apero_core/graph/edge.h>
#include <open_apero_msgs/CommandResult.h>
#include <open_apero_msgs/RobotState.h>
#include <open_apero_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "open_apero_ros/agent/ros_agent_metrics.h"

namespace open_apero {
namespace ros {

/**
 * @brief ROS-based online exploration agent.
 *
 * Wraps the core Agent with ROS subscribers, publishers, services, and
 * TF-based pose tracking so that exploration can run on a live robot or in
 * simulation via ROS topics.
 */
class RosAgent : public Agent {
 public:
  /**
   * @brief Factory method to create a fully initialized ROS agent.
   *
   * Avoids calling virtual methods in the constructor by separating
   * construction from parameter setup.
   *
   * @param nh Public ROS node handle.
   * @param nh_private Private ROS node handle for parameters.
   * @param factory Plugin factory used to create the agent's plugins.
   * @param param_map Parameter map populated by the factory.
   * @return A fully initialized RosAgent.
   */
  static std::unique_ptr<RosAgent> create(const ::ros::NodeHandle& nh, const ::ros::NodeHandle& nh_private, PluginFactory* factory,
                                          Configurable::ParamMap* param_map);

  virtual ~RosAgent() = default;

  /**
   * @brief Resets all agent state for a new episode, including ROS-specific state.
   *
   * Extends Agent::resetEpisode() by also resetting ROS timing counters and clearing the
   * named visualization topics. With a pose held it resets fully; without one it clears
   * the episode state immediately and roots the graph once the first pose arrives.
   * Can be triggered via the ~/reset ROS service.
   */
  void resetEpisode() override;

  /**
   * @brief ROS callback for incoming robot state messages.
   * @param msg The robot state message containing base and sensor transforms.
   */
  void robotStateCallback(const open_apero_msgs::RobotState& msg);

  /**
   * @brief Accept the explicitly completed state for the pending TF-mode command.
   * @param msg Stamped final state carrying the matching command sequence.
   */
  void commandResultCallback(const open_apero_msgs::CommandResult& msg);

  /**
   * @brief Service callback to toggle planning on or off.
   * @param req Request with a boolean data field (true = start, or resume an episode a stop
   *            interrupted, refused once the episode has ended; false = stop, which finalizes
   *            the run output of a started episode).
   * @param res Response indicating success.
   * @return True on success.
   */
  bool runSrvCallback(std_srvs::SetBool::Request& req,    // NOLINT(google-runtime-references) required by ROS service callback signature
                      std_srvs::SetBool::Response& res);  // NOLINT(google-runtime-references)

  /**
   * @brief Service callback to reset the agent for a new episode.
   * @param req Empty request.
   * @param res Empty response.
   * @return True on success.
   */
  bool resetSrvCallback(std_srvs::Empty::Request& req,    // NOLINT(google-runtime-references) required by ROS service callback signature
                        std_srvs::Empty::Response& res);  // NOLINT(google-runtime-references)

  /**
   * @brief Service callback that returns the wall-clock seconds elapsed since the last call.
   * @param req Request (unused).
   * @param res Response whose message field holds the elapsed seconds.
   * @return True on success.
   */
  bool wallTimeSrvCallback(std_srvs::SetBool::Request& req,    // NOLINT(google-runtime-references) ROS service callback signature
                           std_srvs::SetBool::Response& res);  // NOLINT(google-runtime-references)

  /**
   * @brief Publish visualization markers and status overlay to RViz.
   * @param markers The set of visualization markers to publish.
   */
  void publishVisualization(const VisualizationMarkers& markers) override;

  /**
   * @brief Publish one named group of markers to its own RViz MarkerArray topic,
   *        creating the publisher on first use. Publishes nothing without subscribers;
   *        otherwise a leading DELETEALL (even for an empty set) clears the group's
   *        previous markers.
   * @param name    Topic name relative to the node's namespace, not the private one: `/name`
   *                for open_apero_node (e.g. `map_visualization`, `layer_visualization/planning`).
   * @param markers The markers for this group.
   */
  void publishNamedVisualization(const std::string& name, const VisualizationMarkers& markers) override;

  /**
   * @brief Clear every known named visualization topic (publishes a lone
   *        DELETEALL to each). resetEpisode() calls it so the previous episode's
   *        markers do not linger during the idle gap.
   */
  void clearNamedVisualizations();

  /**
   * @brief Main planning loop; spins ROS and calls loopIteration().
   *
   * Returns when ROS shuts down, or when an episode terminates if
   * exit_on_episode_end is set. With that parameter false the loop keeps spinning
   * after termination so the reset service can start the next episode. A shutdown
   * during a started episode finalizes its run output as EpisodeEnd::kStopped.
   */
  void planningLoop() override;

  /**
   * @brief Static helper to initialize a PluginFactory and param map from a ROS namespace.
   * @param factory Factory to read the parameter namespace through.
   * @param param_map Output parameter map.
   * @param nh_private Private node handle whose namespace is queried.
   */
  static void setupFactoryAndParams(PluginFactory& factory, Configurable::ParamMap* param_map, const ::ros::NodeHandle& nh_private);

  // printInfo/printWarning/printError are final in Agent and forward to the
  // logging::* sink; open_apero_node's main() installs the rosconsole sink
  // (installRosconsoleSink) rather than RosAgent overriding them.

  /// @brief Save an RViz screenshot to the log directory; warns when RViz does not answer.
  void saveScreenshot() override;

  /**
   * @brief Timer callback that updates the robot pose from TF.
   * @param event The timer event (unused beyond triggering the callback).
   */
  void updatePoseFromTF(const ::ros::TimerEvent& event);

 protected:
  /**
   * @brief Construct the ROS agent, set up subscribers/publishers.
   *
   * Does not call setupFromParamMap; use the static create() factory method
   * or call setupFromParamMap() after construction.
   */
  RosAgent(const ::ros::NodeHandle& nh, const ::ros::NodeHandle& nh_private, PluginFactory* factory, Configurable::ParamMap* param_map);

  // ros
  ::ros::NodeHandle nh_;
  ::ros::NodeHandle nh_private_;

  ::ros::Publisher target_pub_;
  ::ros::Publisher trajectory_vis_pub_;
  ::ros::Publisher nav_path_pub_;
  ::ros::Publisher status_overlay_pub_;

  /// @brief Named visualization publishers, keyed by their topic-relative name
  /// (e.g. "map_visualization", "layer_visualization/planning"). Created lazily
  /// on first publishNamedVisualization() for that name.
  std::map<std::string, ::ros::Publisher> named_vis_pubs_;

  /// @brief Build a MarkerArray with a leading DELETEALL followed by the
  /// converted markers, all stamped with the map frame and current time.
  visualization_msgs::MarkerArray buildMarkerArrayMsg(const VisualizationMarkers& markers) const;

  /// @brief Lines of the RViz status overlay, top to bottom.
  ///
  /// The base set is the active layer, the traversed distance and the elapsed
  /// time. Derived agents may add lines of their own. buildStatusOverlay sizes
  /// the box from the returned lines and keeps their spacing, so an override
  /// only decides the text.
  virtual std::vector<std::string> statusOverlayLines() const;

  ::ros::Subscriber robot_state_sub_;
  ::ros::Subscriber command_result_sub_;

  ::ros::ServiceServer get_wall_time_srv_;
  ::ros::ServiceServer run_srv_;
  ::ros::ServiceServer reset_srv_;

  ::ros::ServiceClient save_screenshot_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  ::ros::Timer pose_update_timer_;

  std::string p_map_frame_;
  std::string p_base_frame_;
  std::string p_camera_frame_;
  bool p_use_tf_;               ///< Read poses from TF instead of RobotState messages.
  bool p_exit_on_episode_end_;  ///< Leave planningLoop() once an episode terminates.

  /// Correlation ID of the most recently dispatched command. CommandPhase
  /// already records whether it is pending, so no second pending flag is needed.
  /// Deliberately survives resets so a delayed old result cannot match a new command.
  uint32_t command_sequence_ = 0u;

  // Whether toggle_running(true) has already begun an episode. A later true is
  // then a resume, not a restart: see runSrvCallback.
  bool episode_started_ = false;

  // The state (kPlanning or kAwaitingInitialState) a toggle_running(false) interrupted, which
  // toggle_running(true) resumes; kIdle when there is none: never stopped, resumed, ended or reset.
  State interrupted_state_ = State::kIdle;

  ::ros::Time replan_start_ros_;
  std::chrono::high_resolution_clock::time_point wall_time_srv_timer_;

  // Roots the graph in the state that just arrived when a clear is waiting on one
  // (i.e. state_ == State::kAwaitingInitialState). Called from the pose callbacks.
  virtual void onStateAvailable();

  /** @brief Allocate and arm the sequence attached to a movement command. */
  uint32_t beginCommandDispatch();

  /** @brief Validate and canonicalize an externally supplied RobotState message. */
  bool prepareRobotState(const open_apero_msgs::RobotState& msg, const char* source, RobotState* robot_state);

  void onPlanningStart() override;

  bool requestNextTrajectory() override;

  /**
   * @brief Sends a movement command by publishing the trajectory to the platform via ROS topics.
   *        Automatically calls finish() on the cursor, marking the movement as handled.
   * @param cursor The state cursor pointing to the current executing edge.
   * @return true if successful, false if the trajectory inside cursor was empty.
   */
  bool sendMovement(EdgeCursor* cursor) override;

  void setupFromParamMap(Configurable::ParamMap* param_map) override;
};

}  // namespace ros
}  // namespace open_apero
