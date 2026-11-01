#include "open_apero_ros/agent/ros_agent.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_rviz_plugins/Screenshot.h>
#include <nav_msgs/Path.h>
#include <open_apero_core/common/utils.h>
#include <open_apero_core/logging/logging.h>
#include <open_apero_core/logging/scoped_timer.h>
#include <open_apero_ros_bridge/eigen_conversion.h>
#include <open_apero_ros_bridge/marker_conversion.h>
#include <open_apero_ros_bridge/robot_state_conversion.h>
#include <open_apero_ros_bridge/status_overlay.h>
#include <tf2/exceptions.h>

#include "open_apero_ros/plugin/plugin_factory_ros.h"

namespace open_apero {
namespace ros {

std::unique_ptr<RosAgent> RosAgent::create(const ::ros::NodeHandle& nh, const ::ros::NodeHandle& nh_private, PluginFactory* factory,
                                           Configurable::ParamMap* param_map) {
  std::unique_ptr<RosAgent> agent(new RosAgent(nh, nh_private, factory, param_map));
  agent->setupFromParamMap(param_map);
  agent->assureParamsValid();
  // Read through nh_private, not setParam: by the map backend (agentParamsReadByBackend(), e.g. an
  // embedded voxblox server), the logging bridge (log_level), open_apero_node (plugin_libraries) and
  // runSrvCallback (run_dir, which the coordinator writes).
  for (const std::string& key : agent->getMap().agentParamsReadByBackend()) {
    markParamConsumed(param_map, key);
  }
  markParamConsumed(param_map, "log_level");
  markParamConsumed(param_map, "plugin_libraries");
  markParamConsumed(param_map, "run_dir");
  agent->reportUnreadParams(*param_map);
  return agent;
}

RosAgent::RosAgent(const ::ros::NodeHandle& nh, const ::ros::NodeHandle& nh_private, PluginFactory* factory,
                   Configurable::ParamMap* param_map)
    : Agent(factory, param_map), nh_(nh), nh_private_(nh_private) {
  metrics_ = std::make_unique<RosAgentMetrics>();
  // The rosconsole sink is installed once in main() (installRosconsoleSink),
  // before construction, so it is not a per-agent constructor side effect.
}

void RosAgent::setupFromParamMap(Configurable::ParamMap* param_map) {
  Agent::setupFromParamMap(param_map);

  setParam<std::string>(param_map, "map_frame", &p_map_frame_, "map");
  setParam<std::string>(param_map, "base_frame", &p_base_frame_, "base");
  setParam<std::string>(param_map, "camera_frame", &p_camera_frame_, "camera_frame");
  setParam<bool>(param_map, "use_tf", &p_use_tf_, true);
  // A standalone run is one episode long, so ending it ends the process. Drivers
  // that replay many episodes through the reset service set this false to keep the
  // node alive between them (see planningLoop).
  setParam<bool>(param_map, "exit_on_episode_end", &p_exit_on_episode_end_, true);

  target_pub_ = nh_.advertise<open_apero_msgs::RobotTrajectory>("command/trajectory", 10);
  trajectory_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_visualization", 100);
  get_wall_time_srv_ = nh_private_.advertiseService("get_wall_time", &RosAgent::wallTimeSrvCallback, this);
  reset_srv_ = nh_private_.advertiseService("reset", &RosAgent::resetSrvCallback, this);
  run_srv_ = nh_private_.advertiseService("toggle_running", &RosAgent::runSrvCallback, this);
  nav_path_pub_ = nh_.advertise<nav_msgs::Path>("next_movement", 10);
  status_overlay_pub_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("status_overlay", 10);

  save_screenshot_client_ = nh_.serviceClient<jsk_rviz_plugins::Screenshot>("/rviz/screenshot");

  // Pose updates either poll TF at 20 Hz or follow robot_state messages.
  if (p_use_tf_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pose_update_timer_ = nh_.createTimer(::ros::Duration(0.05), &RosAgent::updatePoseFromTF, this);
    command_result_sub_ = nh_.subscribe("command/result", 1, &RosAgent::commandResultCallback, this);
  } else {
    robot_state_sub_ = nh_.subscribe("robot_state", 1, &RosAgent::robotStateCallback, this);
  }

  ROS_INFO_STREAM(
      "\n******************** Initialized Agent ********************\n"
      "Initialized 'RosAgent' from namespace '"
      << param_map->at("param_namespace") << "' with parameters:" << param_map->at("verbose_text"));
}

void RosAgent::setupFactoryAndParams(PluginFactory& factory, Configurable::ParamMap* param_map, const ::ros::NodeHandle& nh_private) {
  *param_map = Configurable::ParamMap();
  std::string type;
  const std::string& args = nh_private.getNamespace();
  factory.getParamMapAndType(param_map, &type, args);
  param_map->at("verbose_text") = "";
}

void RosAgent::onPlanningStart() {
  if (p_use_tf_) {
    robot_->setBaseInSensorTransform(current_state_->getSensorTransform().inverse() * current_state_->getBaseTransform());
  }

  Agent::onPlanningStart();

  wall_time_srv_timer_ = std::chrono::high_resolution_clock::now();
  replan_start_ros_ = ::ros::Time::now();
}

void RosAgent::resetEpisode() {
  interrupted_state_ = State::kIdle;
  if (current_state_) {
    Agent::resetEpisode();
  } else {
    // The clear is what carries the episode boundary to the map's backend, so it runs
    // now rather than when a pose arrives; only the rerooting is owed one.
    clearEpisodeState();
    state_ = State::kAwaitingInitialState;
    ROS_INFO("Episode state cleared; the graph will be rooted at the first robot state.");
  }
  clearNamedVisualizations();
}

void RosAgent::onStateAvailable() {
  if (state_ == State::kAwaitingInitialState) {
    // The clear already ran at the episode boundary; only the rerooting was owed a pose.
    onPlanningStart();
    ROS_INFO("Reset agent (state became available).");
  }
}

bool RosAgent::resetSrvCallback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/) {
  resetEpisode();
  return true;
}

void RosAgent::planningLoop() {
  ROS_INFO("\n******************** Agent is now Running ********************\n");

  while (::ros::ok()) {
    // Terminated means "this episode is over", not necessarily "this process is
    // over": a multi-episode driver ends an episode and then arms the next one
    // through the reset service, which can only be served while this loop keeps
    // spinning. Leaving here unconditionally would unadvertise those services the
    // moment the first episode finished. Breaking at the top of the iteration (and
    // not in the condition) also lets the terminating iteration spin once more, so
    // the stop command published from onGoalReached() leaves the publisher queue
    // before the process exits.
    if (state_ == State::kTerminated && p_exit_on_episode_end_) {
      break;
    }

    if (state_ == State::kPlanning) {
      loopIteration();
      // The awaited state or map frame arrives via a callback, so iterating
      // faster than spinOnce cannot make it arrive sooner — without this the
      // wait is a 100% CPU spin over executeTrajectory. Same wall-time backoff
      // as the idle branch below; it bounds added latency at 2 ms per arrival,
      // noise against a lockstep sim step.
      if (isAwaitingExternalInput(getExecutionStatus())) {
        ::ros::WallDuration(0.002).sleep();
      }
    } else {
      // Wall time prevents a lockstep simulation without /clock from busy-spinning.
      ::ros::WallDuration(0.002).sleep();
    }

    {
      ScopedTimer t(metrics_->as<RosAgentMetrics>()->ros_callback_s, p_log_performance_);
      ::ros::spinOnce();
    }
  }

  // A shutdown (Ctrl+C) ends a started episode as toggle_running(false) does, without the screenshot. rosconsole is
  // already shut down here, so nothing is logged.
  if (episode_started_ && !episode_output_finalized_) {
    finalizeEpisodeOutput(EpisodeEnd::kStopped);
  }
}

bool RosAgent::requestNextTrajectory() {
  if (p_log_performance_) {
    auto* ros_metrics = metrics_->as<RosAgentMetrics>();
    ros_metrics->replan_time_ros_s = (::ros::Time::now() - replan_start_ros_).toSec();
    ros_metrics->ros_time_s = ::ros::Time::now().toSec();
  }

  if (!Agent::requestNextTrajectory()) {
    return false;
  }

  if (p_log_performance_) {
    replan_start_ros_ = ::ros::Time::now();
  }
  return true;
}

bool RosAgent::sendMovement(EdgeCursor* cursor) {
  if (cursor == nullptr || cursor->edge == nullptr || cursor->edge->trajectory.empty()) {
    ROS_ERROR("Tried to publish an empty trajectory");
    return false;
  }

  const RobotTrajectory* trajectory = &(cursor->edge->trajectory);
  const uint32_t command_sequence = beginCommandDispatch();

  open_apero_msgs::RobotTrajectoryPtr msg(new open_apero_msgs::RobotTrajectory);
  msg->header.stamp = ::ros::Time::now();
  msg->header.frame_id = p_map_frame_;
  msg->command_sequence = command_sequence;
  int n_points = trajectory->size();
  msg->points.resize(n_points);
  for (int i = 0; i < n_points; ++i) {
    robotStateToMsg(trajectory->at(i), &msg->points[i]);
  }
  target_pub_.publish(msg);

  nav_msgs::Path path_msg;
  path_msg.header.stamp = ::ros::Time::now();
  path_msg.header.frame_id = p_map_frame_;
  for (int i = 0; i < n_points; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ::ros::Time::now();
    pose.header.frame_id = p_map_frame_;
    ros::vectorEigenToMsg(trajectory->at(i).getSensorPosition(), &pose.pose.position);
    ros::quaternionEigenToMsg(trajectory->at(i).getSensorOrientation(), &pose.pose.orientation);
    path_msg.poses.push_back(pose);
  }
  nav_path_pub_.publish(path_msg);

  cursor->finish();

  return true;
}

visualization_msgs::MarkerArray RosAgent::buildMarkerArrayMsg(const VisualizationMarkers& markers) const {
  visualization_msgs::MarkerArray msg;

  // Clear stale marker IDs before publishing the current set.
  VisualizationMarker delete_marker;
  delete_marker.id = 0;
  delete_marker.ns = "deleteall";
  delete_marker.action = VisualizationMarker::DELETEALL;

  visualization_msgs::Marker delete_msg;
  visualizationMarkerToMsg(delete_marker, &delete_msg);
  msg.markers.push_back(delete_msg);

  visualizationMarkersToMsg(markers, &msg);

  for (visualization_msgs::Marker& m : msg.markers) {
    m.header.frame_id = p_map_frame_;
    m.header.stamp = ::ros::Time::now();
  }

  return msg;
}

void RosAgent::publishNamedVisualization(const std::string& name, const VisualizationMarkers& markers) {
  auto it = named_vis_pubs_.find(name);
  if (it == named_vis_pubs_.end()) {
    it = named_vis_pubs_.emplace(name, nh_.advertise<visualization_msgs::MarkerArray>(name, 10)).first;
  }
  // Disabled RViz displays unsubscribe, so avoid serialization without listeners.
  if (it->second.getNumSubscribers() == 0) {
    return;
  }
  // Empty updates still clear geometry left by the previous tick.
  it->second.publish(buildMarkerArrayMsg(markers));
}

void RosAgent::clearNamedVisualizations() {
  // An empty array is a no-op in RViz; only DELETEALL removes the stale markers.
  visualization_msgs::Marker delete_msg;
  delete_msg.action = visualization_msgs::Marker::DELETEALL;

  visualization_msgs::MarkerArray msg;
  msg.markers.push_back(delete_msg);

  for (auto& entry : named_vis_pubs_) {
    entry.second.publish(msg);
  }
}

std::vector<std::string> RosAgent::statusOverlayLines() const {
  std::stringstream traversed;
  traversed << "Traversed " << std::fixed << std::setprecision(1) << metrics_->total_path_length << " m";
  std::stringstream elapsed;
  elapsed << "Elapsed   " << std::fixed << std::setprecision(1) << metrics_->total_path_duration << " s";

  return {"On " + (active_layer_ ? active_layer_->layerName() : std::string("idle")), traversed.str(), elapsed.str()};
}

void RosAgent::publishVisualization(const VisualizationMarkers& markers) {
  if (markers.getMarkers().empty()) {
    return;
  }

  trajectory_vis_pub_.publish(buildMarkerArrayMsg(markers));
  status_overlay_pub_.publish(buildStatusOverlay(statusOverlayLines()));
}

bool RosAgent::prepareRobotState(const open_apero_msgs::RobotState& msg, const char* source, RobotState* robot_state) {
  if (!robotStateFromMsg(msg, robot_state)) {
    // An orientation that is not a rotation is rejected at the boundary and
    // the message dropped, never repaired: publishers must send unit
    // quaternions. In lockstep the loop then waits on the state source.
    ROS_ERROR_STREAM_THROTTLE(1.0, source << " carries a non-finite or non-unit quaternion; dropping it. "
                                          << "Fix the publisher -- the planner does not repair orientations.");
    return false;
  }

  // Ground sensors should be roll-free; report any normalization correction. Zeroing
  // rotates about the forward axis only, so the correction magnitude is the roll.
  const Eigen::Quaterniond received_orientation = robot_state->getSensorOrientation();
  const Eigen::Vector2d received_yaw_pitch = yawPitchFromQuaternion(received_orientation);

  zeroSensorRoll(robot_state);

  const double introduced_change =
      std::abs(Eigen::AngleAxisd(received_orientation.inverse() * robot_state->getSensorOrientation()).angle());
  if (introduced_change > 1e-4) {
    ROS_WARN_THROTTLE(1.0,
                      "[zeroSensorRoll] received yaw=%.5f pitch=%.5f -> zeroing roll introduced %.5f rad "
                      "orientation change",
                      received_yaw_pitch[0], received_yaw_pitch[1], introduced_change);
  }

  // Re-derive the base from the roll-free sensor pose and fixed mount.
  robot_->placeRobotAtSensorPose(robot_state, robot_state->getSensorTransform(), map_.get());

  return true;
}

void RosAgent::robotStateCallback(const open_apero_msgs::RobotState& msg) {
  RobotState robot_state;
  if (!prepareRobotState(msg, "robot_state message", &robot_state)) {
    return;
  }

  setCurrentState(robot_state, StateUpdateKind::kCommandResult);

  onStateAvailable();
}

void RosAgent::commandResultCallback(const open_apero_msgs::CommandResult& msg) {
  if (command_phase_ != CommandPhase::kPending) {
    ROS_WARN_STREAM_THROTTLE(1.0, "Ignoring command/result sequence " << msg.command_sequence << ": no command is awaiting a result.");
    return;
  }
  if (msg.command_sequence != command_sequence_) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Ignoring command/result sequence " << msg.command_sequence << ": awaiting sequence " << command_sequence_ << ".");
    return;
  }
  if (msg.header.frame_id != p_map_frame_) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "command/result sequence " << msg.command_sequence << " is in frame '" << msg.header.frame_id
                                                              << "', expected '" << p_map_frame_ << "'; dropping it.");
    return;
  }

  RobotState robot_state;
  if (!prepareRobotState(msg.state, "command/result state", &robot_state)) {
    return;
  }

  setCurrentState(robot_state, StateUpdateKind::kCommandResult);
  // The phase is kResultHeld only during this call: a subclass's onStateAvailable() override
  // judges the command against this result here, before a later TF observation replaces
  // current_state_ with a newer live pose.
  onStateAvailable();
}

uint32_t RosAgent::beginCommandDispatch() {
  ++command_sequence_;
  if (command_sequence_ == 0u) {
    ++command_sequence_;  // Reserve zero for commands (such as stop) that expect no result.
  }
  return command_sequence_;
}

bool RosAgent::runSrvCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  if (req.data) {
    if (episode_started_) {
      // Resuming after a stop. The map, graph, history and run outputs all
      // survive; only the timing anchors move, so the idle gap is not charged
      // to the next replan, and the real end finalizes the run output again.
      // Wiping state is what the ~reset service is for.
      if (state_ == State::kTerminated || (state_ == State::kIdle && interrupted_state_ == State::kIdle)) {
        res.success = false;
        res.message = "The episode has ended (budget spent, goal reached or no step offered); call ~reset to start a new one.";
        ROS_WARN_STREAM(res.message);
        return true;
      }
      if (state_ != State::kIdle) {
        res.success = false;
        res.message = "Already running; toggle_running(false) first, or call ~reset to start a new episode.";
        ROS_WARN_STREAM(res.message);
        return true;
      }
      state_ = interrupted_state_;
      interrupted_state_ = State::kIdle;
      resumeEpisode();
      wall_time_srv_timer_ = std::chrono::high_resolution_clock::now();
      replan_start_ros_ = ::ros::Time::now();
      ROS_INFO("Resumed planning; map and graph preserved.");
      // The graph is still unrooted when the stop came before the first pose; one that arrived since roots it now.
      if (state_ == State::kAwaitingInitialState && current_state_) {
        onStateAvailable();
      }
      res.success = true;
      return true;
    }

    // ~run_dir is read at toggle_running(true); the coordinator sets it first.
    std::string run_dir;
    nh_private_.param<std::string>("run_dir", run_dir, "");

    RunOutputConfig run_config;
    run_config.run_dir = run_dir;

    // Fail loudly instead of silently running unlogged: logging was requested
    // but no directory was provided.
    if (loggingRequested() && run_dir.empty()) {
      res.success = false;
      res.message = "log_performance/num_maps_to_save set but no run_dir provided; refusing to start.";
      ROS_ERROR_STREAM(res.message);
      return true;
    }
    if (!run_dir.empty() && !configureRunOutput(run_config)) {
      res.success = false;
      res.message = "Failed to configure run output (see planner log); refusing to start.";
      ROS_ERROR_STREAM(res.message);
      return true;
    }

    resetEpisode();
    episode_started_ = true;
    res.success = true;
  } else {
    // A repeated stop keeps what the first one interrupted; a stop after the episode ended leaves nothing to resume.
    if (episode_started_ && (state_ == State::kPlanning || state_ == State::kAwaitingInitialState)) {
      interrupted_state_ = state_;
    }
    state_ = State::kIdle;
    ROS_INFO("Stopped planning.");
    if (episode_started_) {
      finalizeEpisodeOutput(EpisodeEnd::kStopped);
      saveScreenshot();
    }
    res.success = true;
  }
  return true;
}

bool RosAgent::wallTimeSrvCallback(std_srvs::SetBool::Request& /*req*/, std_srvs::SetBool::Response& res) {
  double time =
      std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - wall_time_srv_timer_).count();
  wall_time_srv_timer_ = std::chrono::high_resolution_clock::now();

  res.message = std::to_string(time);
  res.success = true;
  return true;
}

void RosAgent::saveScreenshot() {
  if (!p_visualize_) {
    return;  // no RViz to take it
  }
  jsk_rviz_plugins::Screenshot srv;
  srv.request.file_name = run_dir_ + "/rviz_screenshot.png";
  if (!save_screenshot_client_.call(srv)) {
    ROS_WARN_STREAM("No RViz screenshot: failed to call " << save_screenshot_client_.getService() << " (is RViz running?).");
  } else {
    ROS_INFO_STREAM("Saved screenshot to: " << srv.request.file_name);
  }
}

void RosAgent::updatePoseFromTF(const ::ros::TimerEvent& /*event*/) {
  RobotState robot_state;
  robot_state.setTimeFromStart(0.0);

  geometry_msgs::TransformStamped base_transform;
  try {
    base_transform = tf_buffer_->lookupTransform(p_map_frame_, p_base_frame_, ::ros::Time(0), ::ros::Duration(1.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  geometry_msgs::TransformStamped sensor_transform;
  try {
    sensor_transform = tf_buffer_->lookupTransform(p_map_frame_, p_camera_frame_, ::ros::Time(0), ::ros::Duration(1.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  if (!robotStateFromMsg(base_transform.transform, sensor_transform.transform, &robot_state)) {
    ROS_ERROR_THROTTLE(1.0, "TF transform carries a non-finite or non-unit quaternion; skipping this sample.");
    return;
  }

  // Agent zeroes sensor roll while preserving the independently
  // measured base from the base_frame lookup.
  // TF is an asynchronous observation stream. Even a transform newer than the
  // dispatch can be mid-motion, so it updates pose but never completes the
  // command. command/result carries the authoritative final state.
  setCurrentState(robot_state, StateUpdateKind::kPoseObservation);

  onStateAvailable();
}

}  // namespace ros
}  // namespace open_apero
