// Each cycle advances layer state, updates and expands candidates, selects by
// layer priority, and executes the resulting trajectory.

#include "open_apero_core/agent/agent.h"

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/common/frames.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/logging/episode_summary.h"
#include "open_apero_core/logging/logging.h"
#include "open_apero_core/logging/run_manifest.h"
#include "open_apero_core/logging/scoped_timer.h"
#include "open_apero_core/state/observed_viewpoints.h"
#include "open_apero_core/visualization/visualization_helpers.h"

namespace open_apero {

namespace {

const char* toString(Agent::CommandPhase phase) {
  switch (phase) {
    case Agent::CommandPhase::kNone:
      return "None";
    case Agent::CommandPhase::kPending:
      return "Pending";
    case Agent::CommandPhase::kResultHeld:
      return "ResultHeld";
    case Agent::CommandPhase::kResolved:
      return "Resolved";
  }
  return "<unknown>";
}

const char* toString(Agent::EpisodeEnd end) {
  switch (end) {
    case Agent::EpisodeEnd::kBudgetExhausted:
      return "budget_exhausted";
    case Agent::EpisodeEnd::kGoalReached:
      return "goal_reached";
    case Agent::EpisodeEnd::kNoStep:
      return "no_step";
    case Agent::EpisodeEnd::kStopped:
      return "stopped";
  }
  return "<unknown>";
}

// A [settle] warning reports a wait once it has lasted kSettleWarningDelay, then at most once per kSettleWarningPeriod [s].
constexpr double kSettleWarningDelay = 1.0;
constexpr double kSettleWarningPeriod = 1.0;

}  // namespace

Agent::~Agent() {
  performance_log_.close();
  map_log_.close();
}

Agent::Agent(PluginFactory* factory, Configurable::ParamMap* /*param_map*/) : factory_(factory) {
  metrics_ = std::make_unique<AgentMetrics>();
  history_data_ = std::make_unique<HistoryData>();
}

void Agent::setupFromParamMap(Configurable::ParamMap* param_map) {
  if (param_map == nullptr) {
    return;
  }
  supplied_param_keys_ = PluginFactory::paramKeys(*param_map);

  bool verbose_plugins = true;
  bool build_plugins_on_init = false;
  bool strict_params = false;

  setParam<bool>(param_map, "verbose", &p_verbose_, true);
  setParam<bool>(param_map, "verbose_plugins", &verbose_plugins, true);
  setParam<bool>(param_map, "debug_diagnostics", &p_debug_diagnostics_, false);
  setVerbose(verbose_plugins);

  // Set before any plugin is created, so the factory checks every plugin's parameters.
  setParam<bool>(param_map, "strict_params", &strict_params, false);
  getFactory().setStrictParams(strict_params);

  setParam<bool>(param_map, "build_plugins_on_init", &build_plugins_on_init, false);
  setParam<bool>(param_map, "log_performance", &p_log_performance_, false);
  setParam<int>(param_map, "num_maps_to_save", &p_num_maps_to_save_, 0);

  setParam<bool>(param_map, "visualize", &p_visualize_, true);
  setParam<bool>(param_map, "color_past_trajectory_by_budget", &p_color_past_trajectory_by_budget_, false);
  setParam<bool>(param_map, "wait_for_map_updated", &p_wait_for_map_updated_, false);

  setParam<double>(param_map, "target_position_threshold", &p_target_position_threshold_, 0.1);
  setParam<double>(param_map, "target_angle_threshold", &p_target_angle_threshold_, 0.1);

  // Both 0.0 disables the observed viewpoints. An invalid pair leaves them as they were;
  // checkParamsValid() reports it.
  setParam<double>(param_map, "observed_viewpoint_position_threshold", &p_observed_viewpoint_position_threshold_, 0.0);
  setParam<double>(param_map, "observed_viewpoint_angle_threshold", &p_observed_viewpoint_angle_threshold_, 0.0);
  if (ObservedViewpoints::isValidThresholdPair(p_observed_viewpoint_position_threshold_, p_observed_viewpoint_angle_threshold_)) {
    history_data_->getObservedViewpoints().setThresholds(p_observed_viewpoint_position_threshold_, p_observed_viewpoint_angle_threshold_);
  }

  // -1 is nondeterministic; >= 0 makes stochastic plugins reproducible (see AgentInterface::getRandomSeed).
  setParam<int>(param_map, "random_seed", &p_random_seed_, -1);

  // Values below 1e-4 mean an unbounded budget.
  setParam<double>(param_map, "cost_budget", &p_cost_budget_, std::numeric_limits<double>::infinity());
  if (p_cost_budget_ < 1e-4) {
    p_cost_budget_ = std::numeric_limits<double>::infinity();
  }
  current_cost_budget_ = p_cost_budget_;

  std::string args;
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "map_args", &args, param_ns + "/map");
  map_ = getFactory().createPlugin<Map>(args, *this, verbose_plugins);

  setParam<std::string>(param_map, "robot_args", &args, param_ns + "/robot");
  robot_ = getFactory().createPlugin<RobotModel>(args, *this, verbose_plugins);

  setParam<std::string>(param_map, "cost_computer_args", &args, param_ns + "/cost_computer");
  cost_computer_ = getFactory().createPlugin<CostComputer>(args, *this, verbose_plugins);

  // "layers" is a required, non-empty dictionary: each key becomes a layer's instanceName(). No layer type
  // is privileged; arbitration order comes only from runtime priority (see requestNextTrajectory).
  const std::string layers_ns = param_ns + "/layers";
  const std::vector<std::string> layer_names = getFactory().listChildNamespaces(layers_ns);
  if (layer_names.empty()) {
    throw std::runtime_error("Agent: 'layers' must be a non-empty dictionary of layer configurations under '" + layers_ns + "'.");
  }
  for (const std::string& instance : layer_names) {
    // createPlugin throws rather than returning null; dictionary keys are unique by construction.
    auto layer = getFactory().createPlugin<Layer>(layers_ns + "/" + instance, *this, verbose_plugins);
    layer->setInstanceName(instance);
    layers_.push_back(std::move(layer));
  }

  // Every priority a layer can report, promotions included, must be distinct across layers.
  // requestNextTrajectory re-checks per tick in case a layer's declared set is incomplete.
  for (size_t i = 0; i < layers_.size(); ++i) {
    for (size_t j = i + 1; j < layers_.size(); ++j) {
      for (const int a : layers_[i]->possiblePriorities()) {
        for (const int b : layers_[j]->possiblePriorities()) {
          if (a == b) {
            throw std::runtime_error("Agent: layers '" + layers_[i]->instanceName() + "' and '" + layers_[j]->instanceName() +
                                     "' can both report priority " + std::to_string(a) +
                                     " (including state-dependent promotions); every possible layer priority must be distinct.");
          }
        }
      }
    }
  }

  if (p_verbose_) {
    // layers_ follows the dictionary's (alphabetical) key order, not priority.
    std::string stack_summary = "Agent configured layer stack (alphabetical):";
    for (const auto& layer : layers_) {
      stack_summary +=
          "\n  - " + layer->instanceName() + " [" + layer->layerName() + "], priority=" + std::to_string(layer->configuredPriority());
    }
    printInfo(stack_summary);
  }

  // Registered unconditionally so the CSV header and run manifest are complete; opening the file is
  // deferred to configureRunOutput(), which may be called after construction.
  performance_log_.addMetrics(*metrics_);
  for (auto& layer : layers_) {
    performance_log_.addMetrics(layer->getLayerMetrics(), layer->instanceName());
  }

  if (build_plugins_on_init) {
    setCurrentState(RobotState(), StateUpdateKind::kPoseObservation);

    printInfo("Resetting graph from current robot state.");
    initial_state_ = std::make_unique<RobotState>(*current_state_);
    active_node_ = nullptr;
    active_layer_ = nullptr;
    active_cursor_.reset();

    for (auto& layer : layers_) {
      layer->onEpisodeStart(*current_state_);
    }
    for (auto& layer : layers_) {
      layer->onStateReached(*current_state_, nullptr, nullptr);
    }
    robot_->update();
    for (auto& layer : layers_) {
      layer->update();
    }
    for (auto& layer : layers_) {
      layer->warmUp();
    }

    current_state_.reset();
  }
}

bool Agent::checkParamsValid(std::string* error_message) {
  const double min_target_angle_threshold = orientationDotEpsilonToAngleThreshold(kEpsilon);

  // The settle gate cannot be tighter than pose deduplication, or a discarded
  // endpoint rotation could leave an unreachable target.
  if (p_target_position_threshold_ <= kEpsilon) {
    std::ostringstream ss;
    ss << "Invalid target_position_threshold=" << p_target_position_threshold_
       << ": isSensorPoseReached must be more relaxed than default isSameSensorPose; expected > " << kEpsilon << ".";
    *error_message = ss.str();
    return false;
  }
  if (p_target_angle_threshold_ < min_target_angle_threshold) {
    std::ostringstream ss;
    ss << "Invalid target_angle_threshold=" << p_target_angle_threshold_
       << ": isSensorPoseReached must be more relaxed than default isSameSensorPose; expected >= " << min_target_angle_threshold
       << " rad for orientation_dot_epsilon=" << kEpsilon << ".";
    *error_message = ss.str();
    return false;
  }

  if (!ObservedViewpoints::isValidThresholdPair(p_observed_viewpoint_position_threshold_, p_observed_viewpoint_angle_threshold_,
                                                error_message)) {
    return false;
  }

  return true;
}

HistoryData& Agent::getHistoryData() {
  if (!history_data_) {
    throw std::runtime_error("getHistoryData() called before history_data_ was set");
  }
  return *history_data_;
}

PluginFactory& Agent::getFactory() {
  if (!factory_) {
    throw std::runtime_error("getFactory() called before factory_ was set");
  }
  return *factory_;
}

Map& Agent::getMap() {
  if (!map_) {
    throw std::runtime_error("getMap() called before map_ was set");
  }
  return *map_;
}

RobotModel& Agent::getRobot() {
  if (!robot_) {
    throw std::runtime_error("getRobot() called before robot_ was set");
  }
  return *robot_;
}

CostComputer* Agent::getCostComputer() {
  return cost_computer_.get();
}

const Eigen::Vector3d& Agent::getCurrentBasePosition() const {
  if (!current_state_) {
    throw std::runtime_error("getCurrentBasePosition() called before current_state_ was set");
  }
  return current_state_->getBasePosition();
}

const Eigen::Vector3d& Agent::getCurrentSensorPosition() const {
  if (!current_state_) {
    throw std::runtime_error("getCurrentSensorPosition() called before current_state_ was set");
  }
  return current_state_->getSensorPosition();
}

const Eigen::Vector3d& Agent::getInitialSensorPosition() const {
  if (!initial_state_) {
    throw std::runtime_error("getInitialSensorPosition() called before initial_state_ was set");
  }
  return initial_state_->getSensorPosition();
}

void Agent::printInfo(const std::string& text) {
  logging::info(text);
}

void Agent::printWarning(const std::string& text) {
  logging::warning(text);
}

void Agent::printError(const std::string& text) {
  logging::error(text);
}

void Agent::clearEpisodeState() {
  printInfo("Resetting agent for new episode.");
  state_ = State::kIdle;
  command_phase_ = CommandPhase::kNone;
  last_execution_status_ = ExecutionStatus::kAwaitingCommandResult;
  episode_output_finalized_ = false;

  // The layers free their graphs in the resets below, so a cursor or node from the
  // finished cycle would dangle until the graph is rerooted.
  active_cursor_.reset();
  active_node_ = nullptr;
  active_layer_ = nullptr;

  map_->resetEpisode();
  robot_->resetEpisode();
  cost_computer_->resetEpisode();
  history_data_->resetEpisode();
  metrics_->resetEpisode();
  for (auto& layer : layers_) {
    layer->resetEpisode();
  }
}

void Agent::resetEpisode() {
  clearEpisodeState();
  onPlanningStart();

  printInfo("Planner reset complete. Ready for new episode.");
}

bool Agent::configureRunOutput(const RunOutputConfig& config) {
  if (run_output_configured_) {
    if (config.run_dir == run_dir_) {
      return true;  // Idempotent: already configured for exactly this directory.
    }
    printError("configureRunOutput: already configured for '" + run_dir_ + "'; refusing to reconfigure into '" + config.run_dir + "'.");
    return false;
  }

  if (config.run_dir.empty()) {
    printError("configureRunOutput: empty run directory.");
    return false;
  }

  const bool want_perf = p_log_performance_;
  const bool want_maps = p_num_maps_to_save_ > 0;

  const std::string manifest_path = config.run_dir + "/run_manifest.yaml";
  const std::string perf_path = config.run_dir + "/performance_log.csv";
  const std::string maplog_path = config.run_dir + "/map_log.csv";
  const std::string maps_dir = config.run_dir + "/maps";

  // Pre-existing targets are rejected, never truncated.
  const auto path_exists = [](const std::string& path) { return ::access(path.c_str(), F_OK) == 0; };
  std::vector<std::string> targets = {manifest_path};
  if (want_perf) targets.push_back(perf_path);
  if (want_maps) targets.push_back(maplog_path);
  for (const auto& target : targets) {
    if (path_exists(target)) {
      printError("configureRunOutput: refusing to overwrite existing '" + target + "'.");
      return false;
    }
  }

  // All-or-nothing: on any failure remove only what THIS call created.
  std::vector<std::string> created_files;
  bool created_maps_dir = false;
  const auto rollback = [&]() {
    performance_log_.close();
    map_log_.close();
    for (const auto& file : created_files) ::unlink(file.c_str());
    if (created_maps_dir) ::rmdir(maps_dir.c_str());
  };

  if (want_maps) {
    if (::mkdir(maps_dir.c_str(), 0775) == 0) {
      created_maps_dir = true;
    } else if (errno != EEXIST) {
      printError("configureRunOutput: failed to create maps directory '" + maps_dir + "'.");
      return false;
    }
  }

  if (want_perf) {
    if (!performance_log_.open(perf_path)) {
      printError("configureRunOutput: failed to open '" + perf_path + "'.");
      rollback();
      return false;
    }
    created_files.push_back(perf_path);
  }

  if (want_maps) {
    if (!map_log_.open(maplog_path)) {
      printError("configureRunOutput: failed to open '" + maplog_path + "'.");
      rollback();
      return false;
    }
    created_files.push_back(maplog_path);
    map_log_.writeRow({"MapName", "TotalPathLength", "TotalPathDuration"});
  }

  RunManifest manifest;
  manifest.columns = performance_log_.columns();
  for (const auto& layer : layers_) {
    manifest.layers.emplace_back(layer->instanceName(), layer->layerName());
  }
  if (std::isfinite(p_cost_budget_)) {
    manifest.cost_budget = p_cost_budget_;
  }
  manifest.random_seed = p_random_seed_;
  std::string manifest_error;
  if (!manifest.write(manifest_path, &manifest_error)) {
    printError("configureRunOutput: failed to write manifest '" + manifest_path + "': " + manifest_error + ".");
    rollback();
    return false;
  }
  created_files.push_back(manifest_path);

  run_dir_ = config.run_dir;
  run_output_configured_ = true;
  printInfo("Run output configured at '" + config.run_dir + "'.");
  return true;
}

void Agent::setCurrentState(const RobotState& robot_state, StateUpdateKind update_kind) {
  RobotState normalized_state = robot_state;
  zeroSensorRoll(&normalized_state);
  current_state_ = std::make_unique<RobotState>(normalized_state);

  // The update after a command result is an ordinary observation; an intermediate observation while
  // the command is pending leaves the phase waiting for the classified result.
  if (command_phase_ == CommandPhase::kResultHeld) {
    command_phase_ = CommandPhase::kResolved;
  }
  if (update_kind == StateUpdateKind::kCommandResult && command_phase_ == CommandPhase::kPending) {
    command_phase_ = CommandPhase::kResultHeld;
  }
}

// Ground planning assumes a level sensor so yaw representations agree. Roll rotates
// about the forward axis, so the extracted angles are the roll-free ZYX yaw and pitch.
void Agent::zeroSensorRoll(RobotState* robot_state) {
  const Eigen::Vector2d yaw_pitch = yawPitchFromQuaternion(robot_state->getSensorOrientation());
  robot_state->setSensorOrientation(quaternionFromEulerAnglesZyx(Eigen::Vector3d(yaw_pitch(0), yaw_pitch(1), 0.0)));
}

void Agent::onPlanningStart() {
  printInfo("Resetting graph from current robot state.");
  initial_state_ = std::make_unique<RobotState>(*current_state_);
  active_node_ = nullptr;
  active_layer_ = nullptr;
  active_cursor_.reset();
  current_map_index_ = 0;
  current_cost_budget_ = p_cost_budget_;
  state_ = State::kPlanning;

  for (auto& layer : layers_) {
    layer->onEpisodeStart(*current_state_);
  }

  replan_start_wall_ = std::chrono::high_resolution_clock::now();
  episode_start_steady_ = std::chrono::steady_clock::now();
}

void Agent::planningLoop() {
  while (state_ != State::kTerminated) {
    if (state_ == State::kPlanning) {
      loopIteration();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void Agent::terminateEpisode(EpisodeEnd end) {
  const bool first_termination = state_ != State::kTerminated;
  state_ = State::kTerminated;
  command_phase_ = CommandPhase::kNone;
  last_execution_status_ = ExecutionStatus::kTerminated;
  active_cursor_.reset();
  active_node_ = nullptr;
  active_layer_ = nullptr;
  if (first_termination) {
    finalizeEpisodeOutput(end);
  }
}

void Agent::resumeEpisode() {
  episode_output_finalized_ = false;
}

void Agent::finalizeEpisodeOutput(EpisodeEnd end) {
  if (episode_output_finalized_) {
    return;
  }
  episode_output_finalized_ = true;

  // Every executed edge is charged before this point, so these are the episode's final totals.
  if (run_output_configured_) {
    EpisodeSummary summary;
    summary.end_reason = toString(end);
    summary.cycles = metrics_->cycle_index;
    summary.total_cost = metrics_->total_cost;
    summary.cost_budget = getOriginalCostBudget();
    summary.total_path_length = metrics_->total_path_length;
    summary.total_path_duration = metrics_->total_path_duration;
    summary.num_visited_nodes = metrics_->num_visited_nodes;
    summary.elapsed_wall_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - episode_start_steady_).count();
    for (const auto& layer : layers_) {
      const std::optional<double> gain = layer->realAccumulatedGain();
      if (gain.has_value()) {
        summary.real_accumulated_gain.emplace_back(layer->instanceName(), *gain);
      }
    }
    const std::string summary_path = run_dir_ + "/episode_summary.yaml";
    std::string summary_error;
    if (!summary.write(summary_path, &summary_error)) {
      printWarning("Failed to write '" + summary_path + "': " + summary_error + ".");
    }
  }

  // An unbounded budget gives no cost spacing for the snapshots, so the final map is the one saved.
  if (p_num_maps_to_save_ > 0 && !std::isfinite(getOriginalCostBudget()) && run_output_configured_) {
    saveMapSnapshot();
  }
}

void Agent::reportUnreadParams(const Configurable::ParamMap& param_map) const {
  factory_->reportIgnoredParams("Agent", supplied_param_keys_, param_map);
}

void Agent::loopIteration() {
  if (!executeTrajectory()) {
    return;
  }

  // The completed edge's node decides whether this was the final approach; read before active_node_ is
  // cleared below. onGoalReached() stops the robot first; the episode ends after the edge is accounted for.
  const bool goal_reached = active_node_ != nullptr && active_node_->is_terminating;
  if (goal_reached) {
    onGoalReached();
  }

  // A layer may free its graph edge during onStateReached(), so snapshot it.
  Edge traversed_edge_snapshot;
  Edge* traversed_edge = nullptr;
  if (active_cursor_ && active_cursor_->edge) {
    traversed_edge_snapshot = *active_cursor_->edge;
    traversed_edge = &traversed_edge_snapshot;
    {
      std::stringstream ss;
      ss << "[loopIteration] recording traversed edge: size=" << traversed_edge->trajectory.size()
         << ", is_backtrack=" << traversed_edge->is_backtrack;
      logging::debug(ss.str());
    }
    history_data_->addVisitedEdge(*traversed_edge);
  }

  recordObservedPoses();

  // The cursor does not own its edge, and layers may free it in the callbacks below (e.g.
  // BacktrackingLayer::planPath replaces recovery_edge_), so drop the non-owning pointers now.
  active_cursor_.reset();
  settle_wait_edge_ = nullptr;
  settle_wait_idx_ = -1;

  for (auto& layer : layers_) {
    layer->onStateReached(*current_state_, traversed_edge, active_node_);
  }

  // onStateReached is active_node_'s only consumer in this cycle; update(),
  // expand() and planPath() below may prune it from the graph, so it must not
  // survive past this point either.
  active_node_ = nullptr;

  if (traversed_edge) {
    // Account for the edge before judging the budget: the edge that exhausts it was executed too.
    metrics_->total_path_length += getTrajectoryLength(traversed_edge->trajectory);
    metrics_->total_path_duration += traversed_edge->trajectory.back().getTimeFromStart();
    metrics_->total_cost += traversed_edge->cost;
    metrics_->num_visited_nodes += 1;

    reduceCostBudget(traversed_edge->cost);
    if (!isCostBudgetPositive()) {
      printInfo("Cost budget reached.");
      updateFrameLedger();
      terminateEpisode(EpisodeEnd::kBudgetExhausted);
      saveScreenshot();
      return;
    }
  }

  // The final approach is now accounted for exactly once, so end the episode
  // before any update/expand/replan work that no longer has a purpose.
  if (goal_reached) {
    updateFrameLedger();
    terminateEpisode(EpisodeEnd::kGoalReached);
    return;
  }

  // The robot updates before the layers: a layer's update or expansion samples
  // through it, so its map-derived state has to be current first.
  robot_->update();

  // Update before expansion so new candidates use the current state.
  for (auto& layer : layers_) {
    layer->update();
  }

  if (readyToExpand()) {
    for (auto& layer : layers_) {
      layer->expand();
    }
  }

  requestNextTrajectory();
}

bool Agent::requestNextTrajectory() {
  if (p_verbose_ || p_log_performance_) {
    metrics_->replan_time_wall_s = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - replan_start_wall_).count();
    replan_start_wall_ = std::chrono::high_resolution_clock::now();

    if (p_verbose_) {
      std::stringstream ss;
      ss << "Replanning: " << std::setprecision(3) << metrics_->replan_time_wall_s << "s elapsed.";
      printInfo(ss.str());
    }
  }

  // Read the ledger here, not in recordObservedPoses, so every caller of this
  // function is covered. resetCycle() and resetMapUpdateStatus() both run below,
  // after the row is written, so the two counts describe the cycle that just ended.
  updateFrameLedger();

  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  // Snapshot each layer's runtime priority exactly once this tick: getPriority()
  // is state-dependent (e.g. BacktrackingLayer does traversability work and may
  // promote itself), so it must not be re-evaluated inside a sort comparator.
  struct LayerCandidate {
    Layer* layer;
    int priority;
  };
  std::vector<LayerCandidate> candidates;
  candidates.reserve(layers_.size());
  for (auto& layer : layers_) {
    candidates.push_back({layer.get(), layer->getPriority(*current_state_)});
  }
  // Runtime priorities must be distinct so arbitration order is unambiguous.
  // Configured priorities are distinct (checked at startup), but promotion can
  // still collide two layers at the same runtime value.
  for (size_t i = 0; i < candidates.size(); ++i) {
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (candidates[i].priority == candidates[j].priority) {
        throw std::runtime_error("Agent: layers '" + candidates[i].layer->instanceName() + "' and '" + candidates[j].layer->instanceName() +
                                 "' report the same runtime priority " + std::to_string(candidates[i].priority) +
                                 "; layer priorities must be distinct.");
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const LayerCandidate& a, const LayerCandidate& b) { return a.priority > b.priority; });

  Layer* winning_layer = nullptr;
  {
    ScopedTimer t(metrics_->plan_path_total_s, p_log_performance_);
    for (const LayerCandidate& candidate : candidates) {
      Layer* layer = candidate.layer;
      if (layer->planPath(*current_state_, map_.get(), robot_.get(), &planned_path, &planned_edges)) {
        winning_layer = layer;
        break;
      }
    }
  }

  if (!winning_layer) {
    // Still a complete cycle: flush the map's cycle-local state so it does not leak into the next episode.
    map_->resetMapUpdateStatus();
    printError("No layer offered a step. Terminating.");
    terminateEpisode(EpisodeEnd::kNoStep);
    return false;
  }

  if (planned_path.size() < 2) {
    throw std::runtime_error("No planned path found.");
  }
  if (planned_edges.empty()) {
    throw std::runtime_error("No planned edges found.");
  }

  winning_layer->onSelectedAsWinner();
  active_layer_ = winning_layer;

  Node* next_node = planned_path[1];
  Edge* next_edge = planned_edges.front();

  RobotTrajectory trajectory = next_edge->trajectory;
  if (trajectory.size() < 2) {
    throw std::runtime_error("Next trajectory has less than 2 points; a move needs the current and the goal state.");
  }
  if (std::abs(trajectory.front().getTimeFromStart()) > kEpsilon) {
    throw std::runtime_error("Next trajectory does not start from time 0.");
  }

  {
    ScopedTimer t(metrics_->visualization_s, p_log_performance_);
    if (p_visualize_) {
      // Separate topics let RViz toggle layers; active mirrors the current winner.
      for (auto& layer : layers_) {
        VisualizationMarkers layer_markers;
        const bool is_active = (layer.get() == active_layer_);
        Node* selected = is_active ? next_node : nullptr;
        layer->getVisualization(selected, &layer_markers);
        publishNamedVisualization("layer_visualization/" + layer->instanceName(), layer_markers);
        if (is_active) {
          publishNamedVisualization("layer_visualization/active", layer_markers);
        }

        // Its own topic: what the chosen path is worth is read against the graph, not
        // inside it, so it has to be switchable without taking the graph with it.
        VisualizationMarkers path_markers;
        layer->getPathVisualization(selected, &path_markers);
        publishNamedVisualization("path_visualization/" + layer->instanceName(), path_markers);
        if (is_active) {
          publishNamedVisualization("path_visualization/active", path_markers);
        }
      }

      VisualizationMarkers map_markers;
      map_->getVisualization(&map_markers);
      publishNamedVisualization("map_visualization", map_markers);

      VisualizationMarkers robot_markers;
      robot_->getVisualization(trajectory, &robot_markers);
      publishNamedVisualization("robot_visualization", robot_markers);

      VisualizationMarkers trajectory_markers;
      if (p_color_past_trajectory_by_budget_) {
        history_data_->getVisualization(&trajectory_markers, getOriginalCostBudget());
      } else {
        history_data_->getVisualization(&trajectory_markers);
      }
      getPlannedPathVisualizationMarkers(planned_edges, &trajectory_markers, map_.get());
      publishVisualization(trajectory_markers);
    }
  }

  // Snapshots are spaced in used budget; an unbounded budget saves its one snapshot at termination.
  if (p_num_maps_to_save_ > 0 && std::isfinite(getOriginalCostBudget()) && run_output_configured_) {
    const double target_cost = static_cast<double>(current_map_index_) / p_num_maps_to_save_ * getOriginalCostBudget();
    if (getUsedCostBudget() > target_cost) {
      saveMapSnapshot();
    }
  }

  if (p_verbose_) {
    for (auto& layer : layers_) {
      const std::string summary = layer->cycleSummary();
      if (!summary.empty()) {
        printInfo(summary);
      }
    }
  }

  if (p_log_performance_) {
    metrics_->elapsed_wall_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - episode_start_steady_).count();
    performance_log_.writeRow();
  }
  metrics_->cycle_index += 1;  // zero-based: a row just written carries the pre-increment index

  metrics_->resetCycle();
  for (auto& layer : layers_) {
    layer->getLayerMetrics().resetCycle();
  }
  map_->resetMapUpdateStatus();
  command_phase_ = CommandPhase::kNone;

  active_node_ = next_node;
  active_cursor_ = std::make_unique<EdgeCursor>(next_edge);
  // Clear the non-owning timer key before its edge can be freed.
  settle_wait_edge_ = nullptr;
  settle_wait_idx_ = -1;
  // A preflight veto produces no simulation frame: the phase stays kNone and settling skips the map gate.
  if (sendMovement(active_cursor_.get())) {
    ++metrics_->commands_dispatched;
    command_phase_ = CommandPhase::kPending;
    last_execution_status_ = ExecutionStatus::kPlanned;
  }
  return true;
}

bool Agent::executeTrajectory() {
  if (!active_cursor_) {
    return true;
  }

  // Measure waiting per waypoint so diagnostics expose settle deadlocks.
  const auto now = std::chrono::high_resolution_clock::now();
  if (active_cursor_->edge != settle_wait_edge_ || active_cursor_->waypoint_idx != settle_wait_idx_) {
    settle_wait_edge_ = active_cursor_->edge;
    settle_wait_idx_ = active_cursor_->waypoint_idx;
    settle_wait_start_ = now;
  }
  const double elapsed = std::chrono::duration<double>(now - settle_wait_start_).count();
  const bool log_settle =
      elapsed > kSettleWarningDelay && std::chrono::duration<double>(now - last_settle_log_time_).count() > kSettleWarningPeriod;

  // Do not settle against the state preceding the pending command.
  if (command_phase_ == CommandPhase::kPending) {
    last_execution_status_ = ExecutionStatus::kAwaitingCommandResult;
    if (log_settle) {
      last_settle_log_time_ = now;
      std::stringstream ss;
      ss << "[settle] AWAITING state for dispatched command: waypoint_idx=" << active_cursor_->waypoint_idx << "/"
         << active_cursor_->edge->trajectory.size() << " elapsed=" << std::fixed << std::setprecision(2) << elapsed << "s";
      printWarning(ss.str());
    }
    return false;
  }

  if (!hasSettled(active_cursor_->to(), log_settle)) {
    if (log_settle) {
      last_settle_log_time_ = now;
      std::stringstream ss;
      ss << "[settle] STALLED waiting to settle: waypoint_idx=" << active_cursor_->waypoint_idx << "/"
         << active_cursor_->edge->trajectory.size() << " finished=" << active_cursor_->isFinished()
         << " command_phase=" << toString(command_phase_) << " elapsed=" << std::fixed << std::setprecision(2) << elapsed << "s";
      printWarning(ss.str());
    }
    return false;
  }

  // Snap to the waypoint so adjacent history edges share an exact endpoint.
  // The next measured state replaces this canonical pose.
  setCurrentState(active_cursor_->to(), StateUpdateKind::kPoseObservation);

  if (!active_cursor_->isFinished()) {
    // A mid-edge veto follows a command, so its stop frame can still settle.
    if (sendMovement(active_cursor_.get())) {
      ++metrics_->commands_dispatched;
      command_phase_ = CommandPhase::kPending;
      last_execution_status_ = ExecutionStatus::kPlanned;
    }
    return false;
  }

  // The edge is complete; whether it was the final approach is loopIteration's decision.
  return true;
}

void Agent::updateFrameLedger() {
  std::vector<Transform3d> mapped_optical_frames;
  const bool map_reports_frames = map_->getMappedOpticalFrameTransforms(&mapped_optical_frames);
  metrics_->frames_mapped = map_reports_frames ? static_cast<int>(mapped_optical_frames.size()) : 0;
  if ((map_reports_frames || p_debug_diagnostics_) && metrics_->frames_mapped < metrics_->commands_dispatched) {
    std::stringstream ss;
    ss << "[frame ledger] cycle " << metrics_->cycle_index << ": dispatched " << metrics_->commands_dispatched
       << " commands, the map integrated " << metrics_->frames_mapped << " frames since the last replan";
    if (map_reports_frames) {
      printError(ss.str());
    } else {
      printWarning(ss.str() + " (the map reports no frame list)");
    }
  }
}

void Agent::saveMapSnapshot() {
  std::stringstream map_name;
  map_name << std::setw(5) << std::setfill('0') << current_map_index_;
  std::ostringstream length, duration;
  length << metrics_->total_path_length;
  duration << metrics_->total_path_duration;
  map_log_.writeRow({map_name.str(), length.str(), duration.str()});
  const std::string map_file_path = run_dir_ + "/maps/" + map_name.str();
  if (!map_->saveMap(map_file_path)) {
    printError("Failed to save map: " + map_file_path);
  }
  ++current_map_index_;
}

// Harvest the frames the map integrated since its list was last cleared, whatever the planner commanded.
void Agent::recordObservedPoses() {
  std::vector<Transform3d> optical_frames;
  if (!map_->getMappedOpticalFrameTransforms(&optical_frames)) {
    return;
  }
  for (const Transform3d& optical_frame : optical_frames) {
    history_data_->addObservedPose(opticalToSensorFrame(optical_frame));
  }
}

bool Agent::hasSettled(const RobotState& target, bool log_on_block) {
  if (!current_state_->isSensorPoseReached(target, p_target_position_threshold_, p_target_angle_threshold_)) {
    last_execution_status_ = ExecutionStatus::kAwaitingPose;
    if (log_on_block) {
      const double pos_err = (current_state_->getSensorPosition() - target.getSensorPosition()).norm();
      const double angle_err =
          std::abs(Eigen::AngleAxisd(current_state_->getSensorOrientation().inverse() * target.getSensorOrientation()).angle());
      std::stringstream ss;
      ss << "[settle] BLOCKED pose: pos_err=" << pos_err << " (thr=" << p_target_position_threshold_ << ") angle_err=" << angle_err
         << " (thr=" << p_target_angle_threshold_ << ")";
      printWarning(ss.str());
    }
    return false;
  }

  if (!p_wait_for_map_updated_) {
    return true;
  }

  // Only a published command can produce the fresh map frame required below.
  if (command_phase_ == CommandPhase::kNone) {
    if (log_on_block) {
      printInfo("[settle] map-update gate skipped: no command since the gate was armed, no frame can arrive; replanning immediately");
    }
    return true;
  }

  // The target is a sensor pose; compare in the optical frame the map reports.
  const Transform3d target_sensor_transform{target.getSensorPosition(), target.getSensorOrientation()};
  const Transform3d target_optical_frame_transform = sensorToOpticalFrame(target_sensor_transform);

  if (map_->hasMappedFrameNear(target_optical_frame_transform, p_target_position_threshold_, p_target_angle_threshold_)) {
    return true;
  }

  last_execution_status_ = ExecutionStatus::kAwaitingMapUpdate;
  if (log_on_block) {
    // The frame list only feeds the report; the gate has already answered, and on a
    // backend that pairs streams it asked more than this list can tell.
    std::vector<Transform3d> optical_frame_transforms;
    if (!map_->getMappedOpticalFrameTransforms(&optical_frame_transforms)) {
      std::stringstream ss;
      ss << "[settle] pose OK, BLOCKED map-update: no frame mapped since the last replan; "
         << "target pos=[" << target.getSensorPosition().transpose() << "]";
      printWarning(ss.str());
      return false;
    }

    std::stringstream ss;
    ss << "[settle] pose OK, BLOCKED map-update: " << optical_frame_transforms.size()
       << " frame(s) mapped since the last replan, none within (pos=" << p_target_position_threshold_
       << ", angle=" << p_target_angle_threshold_ << ") of target pos=[" << target.getSensorPosition().transpose() << "]";

    if (p_debug_diagnostics_) {
      double best_score = std::numeric_limits<double>::infinity();
      double best_position_error = std::numeric_limits<double>::infinity();
      double best_angle_error = std::numeric_limits<double>::infinity();
      size_t best_frame_index = 0u;
      for (size_t frame_index = 0u; frame_index < optical_frame_transforms.size(); ++frame_index) {
        const Transform3d& optical_frame_transform = optical_frame_transforms[frame_index];
        const double position_error = (optical_frame_transform.position - target_optical_frame_transform.position).norm();
        const double angle_error =
            std::abs(Eigen::AngleAxisd(optical_frame_transform.orientation.inverse() * target_optical_frame_transform.orientation).angle());
        const double score = std::max(position_error / p_target_position_threshold_, angle_error / p_target_angle_threshold_);
        if (score < best_score) {
          best_score = score;
          best_position_error = position_error;
          best_angle_error = angle_error;
          best_frame_index = frame_index;
        }
      }

      const Transform3d best_sensor_frame = opticalToSensorFrame(optical_frame_transforms[best_frame_index]);
      const Eigen::Vector2d target_yaw_pitch = yawPitchFromQuaternion(target_sensor_transform.orientation);
      const Eigen::Vector2d mapped_yaw_pitch = yawPitchFromQuaternion(best_sensor_frame.orientation);
      ss << "; nearest mapped frame " << best_frame_index << " has pos_err=" << best_position_error << " m angle_err=" << best_angle_error
         << " rad, mapped sensor pos=[" << best_sensor_frame.position.transpose() << "] yaw_pitch=[" << mapped_yaw_pitch.transpose()
         << "], target yaw_pitch=[" << target_yaw_pitch.transpose() << "]";
    }
    printWarning(ss.str());
  }

  return false;
}

}  // namespace open_apero
