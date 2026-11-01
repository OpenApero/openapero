#pragma once

/**
 * @file agent.h
 * @brief Core online planning loop for active perception.
 *
 * Agent runs the planning cycle: layer expansion, priority-sorted step selection and
 * trajectory execution. Each layer keeps its own current-state view; the agent passes RobotState to
 * all of them via Layer::onStateReached.
 * Subclasses (e.g. RosAgent) implement platform I/O.
 */

#include <chrono>
#include <ctime>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/agent/agent_metrics.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/edge_cursor.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/logging/csv_writer.h"
#include "open_apero_core/logging/performance_log.h"
#include "open_apero_core/plugin/cost_computer/cost_computer.h"
#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/generator/graph_generator.h"
#include "open_apero_core/plugin/layer/layer.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/history_data.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/**
 * @brief Where a run's output artifacts go.
 *
 * Analysis-only inputs (e.g. a plot's reference line) do not belong here; the manifest records only
 * what the run observes.
 */
struct RunOutputConfig {
  std::string run_dir;  ///< Directory to write the run's artifacts into.
};

/**
 * @brief Main agent that drives the active-perception planning loop.
 *
 * Owns the map, the robot model, the cost computer and the layer stack, which setupFromParamMap() creates through the
 * plugin factory, and runs the plan-execute cycle: loopIteration() returns early until the dispatched edge is complete,
 * then notifies, updates and expands the layers, and requestNextTrajectory() selects and dispatches the next step.
 *
 * Agent is abstract. A platform derives from it, implements sendMovement() and AgentInterface::publishVisualization(),
 * and feeds measured states in with setCurrentState(). The constructor and setupFromParamMap() are protected, so the
 * subclass constructs and sets itself up: RosAgent::create does both, BenchmarkAgent is a minimal driver without a robot.
 */
class Agent : public AgentInterface, public Configurable {
 public:
  /**
   * @brief Lifecycle of the agent's episode.
   *
   * kIdle: no episode (constructed, or reset before a state arrived). kAwaitingInitialState: set only by platforms
   * that wait for the first pose before planning (RosAgent). kPlanning: the episode runs and planningLoop()
   * calls loopIteration(). kTerminated: the episode ended (terminateEpisode()) and planningLoop() returns.
   */
  enum class State { kIdle, kAwaitingInitialState, kPlanning, kTerminated };

  /**
   * @brief Why the most recent execution step did or did not dispatch a trajectory.
   *        Set at each decision point in executeTrajectory/hasSettled, so a non-ROS
   *        driver can tell waiting from planning or termination.
   */
  enum class ExecutionStatus {
    kAwaitingCommandResult,  ///< A command is in flight; its resulting state has not arrived.
    kAwaitingPose,           ///< Reported pose is not yet within thresholds of the target waypoint.
    kAwaitingMapUpdate,      ///< Pose reached, but no matching optical map frame has arrived yet.
    kPlanned,                ///< A trajectory was dispatched this step.
    kTerminated,             ///< The episode ended (goal reached, budget exhausted, or no step offered).
  };

  /**
   * @brief Where the command sent this cycle stands. The map-update gate may wait for a mapped frame only
   *        once a command was published (any phase but kNone): a lockstep simulator produces frames only in
   *        response to commands.
   */
  enum class CommandPhase {
    kNone,        ///< No command sent since the map-update gate was armed (startup, veto, episode end); no frame can come.
    kPending,     ///< Command published; its resulting state has not arrived yet.
    kResultHeld,  ///< The state currently held is the command's own result. Lasts until the next state
                  ///< arrives: an asynchronous source (TF poll, pose stream) also delivers samples that are
                  ///< merely periodic, and only in this phase may a consumer judge what the command achieved.
    kResolved,    ///< Result consumed; a map frame for the command may still arrive.
  };

  /** @brief Whether an incoming state completes a command or merely observes motion. */
  enum class StateUpdateKind {
    kCommandResult,    ///< Authoritative state produced after the pending command completed.
    kPoseObservation,  ///< Intermediate/asynchronous pose; must not resolve the pending command.
  };

  /** @brief How an episode segment ended; episode_summary.yaml records it as end_reason. */
  enum class EpisodeEnd { kBudgetExhausted, kGoalReached, kNoStep, kStopped };

  virtual ~Agent();

  /**
   * @brief Starts a new episode: clearEpisodeState() to drop what the finished
   *        episode accumulated, then onPlanningStart() to root the graph in the
   *        state currently held.
   *
   * Precondition: a state is held (hasCurrentState()); onPlanningStart() dereferences it. RosAgent's override
   * also handles the case without one.
   */
  virtual void resetEpisode();

  /** @brief Runs loopIteration() while planning until the episode terminates; in any other
   *  state it sleeps 1 ms per pass. */
  virtual void planningLoop();

  /**
   * @brief Activates a run's output directory: run_manifest.yaml always, performance_log.csv
   *        with log_performance, map_log.csv and maps/ with num_maps_to_save > 0.
   *        All-or-nothing: on failure only files this call created are removed, and existing
   *        target files are rejected. A failed attempt may be retried. Idempotent for the same
   *        directory; a different one is rejected.
   * @return True if the run is configured (now or already, same directory).
   */
  bool configureRunOutput(const RunOutputConfig& config);

  /** @brief Warns about, or with strict_params throws for, top-level keys of the agent's map that
   *  nothing read. Platforms call it after their complete setup (RosAgent::create does). */
  void reportUnreadParams(const Configurable::ParamMap& param_map) const;

  /// @brief Whether this run produces any on-disk artifacts that need a directory.
  bool loggingRequested() const { return p_log_performance_ || p_num_maps_to_save_ > 0; }

  bool hasCurrentState() const override { return current_state_ != nullptr; }
  const Eigen::Vector3d& getCurrentBasePosition() const override;
  const Eigen::Vector3d& getCurrentSensorPosition() const override;
  const Eigen::Vector3d& getInitialSensorPosition() const override;

  HistoryData& getHistoryData() override;
  PluginFactory& getFactory() override;
  Map& getMap() override;
  RobotModel& getRobot() override;
  CostComputer* getCostComputer() override;

  // final: a single logging route. Every message — from any Agent subclass or a
  // free logging::* call — flows through the installed sink, so a platform wires
  // its backend once (via logging::setSink) instead of also overriding these.
  void printInfo(const std::string& text) final;
  void printWarning(const std::string& text) final;
  void printError(const std::string& text) final;
  bool checkParamsValid(std::string* error_message) override;

  /** @brief Saves a screenshot; called when the budget is exhausted and by RosAgent's stop. No-op by default. */
  virtual void saveScreenshot() {}

  /** @brief Re-arms finalizeEpisodeOutput() for an episode resumed after a stop, so its real end
   *  writes the run output again. */
  void resumeEpisode();

  AgentMetrics& getMetrics() { return *metrics_; }
  bool isLoggingPerformance() const override { return p_log_performance_; }
  bool isDebugDiagnosticsEnabled() const final { return p_debug_diagnostics_; }

  /// @brief Status of the most recent execution step (see ExecutionStatus).
  ExecutionStatus getExecutionStatus() const { return last_execution_status_; }

  /**
   * @brief True while execution waits on a command result, a pose within thresholds or a
   *        mapped frame. All arrive through callbacks, so a driver may back off meanwhile.
   */
  static bool isAwaitingExternalInput(ExecutionStatus status) {
    return status == ExecutionStatus::kAwaitingCommandResult || status == ExecutionStatus::kAwaitingPose ||
           status == ExecutionStatus::kAwaitingMapUpdate;
  }

  /// @brief Episode-wide random seed (-1 = nondeterministic). Plugins pull this
  /// to seed their own engines so a whole run is reproducible.
  int getRandomSeed() const override { return p_random_seed_; }

  /// @brief The configured episode budget (see AgentInterface::getOriginalCostBudget). The budget is shared by all
  /// layers: the agent charges every executed edge to it, whichever layer produced the edge, and terminates the
  /// episode once isCostBudgetPositive() is false.
  double getOriginalCostBudget() const override { return p_cost_budget_; }
  double getCurrentCostBudget() const override { return current_cost_budget_; }
  double getUsedCostBudget() const override { return p_cost_budget_ - current_cost_budget_; }
  bool isCostBudgetPositive() const override { return current_cost_budget_ > 0.0; }
  /// @brief Charges @p cost_to_reduce to the remaining budget; the agent does so for every executed edge.
  void reduceCostBudget(double cost_to_reduce) { current_cost_budget_ -= cost_to_reduce; }
  /// @brief Sets both the configured and the remaining budget to @p cost_budget, overriding the cost_budget parameter.
  void setCostBudget(double cost_budget) {
    p_cost_budget_ = cost_budget;
    current_cost_budget_ = cost_budget;
  }

 protected:
  /**
   * @param factory   Factory the agent creates its plugins with; must outlive the agent.
   * @param param_map Not read by the constructor. The creator passes the same map to
   *                  setupFromParamMap() once the object is complete (RosAgent::create);
   *                  a caller that never sets up (BenchmarkAgent) may pass nullptr.
   */
  Agent(PluginFactory* factory, Configurable::ParamMap* param_map);

  /**
   * @brief Reads the agent parameters and creates the map, robot, cost computer and layer stack.
   * @param param_map Agent parameters; nullptr leaves every parameter at its default and creates nothing.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Drops everything the finished episode accumulated -- map, robot, cost
   *        computer, history, metrics, layers, and the dispatch the finished cycle
   *        left behind -- without needing a pose. onPlanningStart() is the half that
   *        needs the pose; resetEpisode() is the two in sequence.
   */
  void clearEpisodeState();

  /**
   * @brief Roots a new episode in the state currently held: records it as the initial state, restores the full cost
   *        budget, enters State::kPlanning and calls Layer::onEpisodeStart() on every layer.
   *
   * Precondition: a state is held (hasCurrentState()).
   */
  virtual void onPlanningStart();

  /**
   * @brief One pass of planningLoop(): the first half of a cycle.
   *
   * Returns early until executeTrajectory() reports the dispatched edge complete. Then records the edge, calls
   * Layer::onStateReached() on every layer, charges the edge to the budget (terminating the episode when the budget
   * is spent or the goal is reached), updates the robot model and the layers, expands the layers when readyToExpand(),
   * and calls requestNextTrajectory().
   */
  virtual void loopIteration();

  /**
   * @brief Selects and dispatches the next step: the second half of a cycle, after loopIteration() has notified,
   *        updated and expanded the layers.
   *
   * Snapshots every layer's getPriority(), asks the layers in descending priority for planPath() until one offers a
   * step, publishes the visualization, writes the performance-log row (with log_performance), arms the map-update gate
   * and dispatches the step's first edge through sendMovement().
   *
   * @return False when no layer offers a step; the episode then terminates (EpisodeEnd::kNoStep).
   * @throws std::runtime_error if two layers report the same runtime priority, or the winning step has fewer than two
   *         nodes, no edge, or a first edge with fewer than two states or not starting at time 0.
   */
  virtual bool requestNextTrajectory();

  /** @brief Whether the agent should expand each layer this iteration.
   *  Default returns true; subclasses may gate (e.g. on auxiliary state). */
  virtual bool readyToExpand() { return true; }

  /**
   * @brief Sends the edge under @p cursor to the robot: the hook through which every platform moves.
   *
   * Called with the selected edge at waypoint_idx 0 and, until cursor->isFinished(), again each time the robot has
   * settled on cursor->to(). Advance the cursor past what was commanded: EdgeCursor::finish() when the whole
   * trajectory is sent (RosAgent, BenchmarkAgent), EdgeCursor::advance() for one waypoint. A cursor left at 0 makes
   * the agent settle on the start state and call this again.
   *
   * @param cursor The active edge and waypoint index; never null.
   * @return True if a command was published; execution then waits for its result, which the platform delivers with
   *         setCurrentState(state, StateUpdateKind::kCommandResult). False if nothing was published (e.g. a pre-flight
   *         veto): no result is awaited and, on an edge's first dispatch, the map-update gate expects no frame
   *         (CommandPhase::kNone).
   */
  virtual bool sendMovement(EdgeCursor* cursor) = 0;

  /**
   * @brief Hook for subclasses that detect a collision or a vetoed step; nothing in the core calls it.
   *        The default does nothing.
   * @param hit_really_happened True when the robot was observed failing to reach the commanded
   *                            waypoint, false when the remaining trajectory was vetoed before dispatch.
   */
  virtual void respondToHit(bool hit_really_happened);

  /**
   * @brief Store the latest robot state after applying agent-level state normalization.
   * @param robot_state State to store.
   * @param update_kind Whether this state completes the pending command.
   */
  void setCurrentState(const RobotState& robot_state, StateUpdateKind update_kind);

  /**
   * @brief Zeroes the roll of a state's sensor orientation in place (the sensor is assumed
   *        level). Afterwards getSensorYaw and getSensorEulerAnglesZyx()(0) agree exactly, unless the
   *        sensor points straight up or down (gimbal lock).
   */
  static void zeroSensorRoll(RobotState* robot_state);

  /**
   * @brief Advances execution of the dispatched edge: once the robot has settled on the commanded waypoint, sends the
   *        next one through sendMovement().
   * @return True once the whole edge is complete, or when nothing is dispatched; false while waiting for a command
   *         result, the pose or a map frame, or right after sending the next waypoint.
   */
  bool executeTrajectory();

  /** @brief Called once when a terminating edge has been executed to its end; the platform
   *  stops the robot and records the goal. */
  virtual void onGoalReached() {}

  /** @brief Ends the episode: nothing plans or is dispatched. Silent; each call site logs its own
   *  reason. The first call since the last reset or resume finalizes the run output with @p end. */
  void terminateEpisode(EpisodeEnd end);

  /** @brief Writes episode_summary.yaml (with the run output configured) and, with an unbounded
   *  budget, the final map snapshot. Once per episode segment: termination and a platform's stop
   *  both call it; resumeEpisode() re-arms it. */
  void finalizeEpisodeOutput(EpisodeEnd end);

  /**
   * @brief Whether the robot has settled on @p target: pose within the target thresholds and,
   *        with wait_for_map_updated, a mapped frame near it.
   * @param target       Waypoint the robot was commanded to.
   * @param log_on_block When true, emit a diagnostic naming the gate (pose vs map-update) that is
   *                     keeping the robot from settling; executeTrajectory() sets it for a wait over a
   *                     second, at most once a second.
   * @return True if the robot may proceed past @p target.
   */
  bool hasSettled(const RobotState& target, bool log_on_block = false);

  /**
   * @brief Copies the frames the map reports as integrated into the observation history.
   *        Call once per cycle; the map keeps them until the next dispatch clears them.
   */
  void recordObservedPoses();

  /**
   * @brief Sets AgentMetrics::frames_mapped to the number of frames the map reports integrating
   *        since the last dispatch, and logs an error when it is below commands_dispatched. A map
   *        that reports no frame list counts 0 and is reported, as a warning, only with debug_diagnostics.
   */
  void updateFrameLedger();

  /**
   * @brief Saves map snapshot current_map_index_ to maps/ and appends its map_log.csv row.
   *        Precondition: num_maps_to_save > 0 and the run output is configured.
   */
  void saveMapSnapshot();

  PluginFactory* factory_;

  std::vector<std::unique_ptr<Layer>> layers_;

  std::unique_ptr<Map> map_;
  std::unique_ptr<RobotModel> robot_;
  std::unique_ptr<CostComputer> cost_computer_;
  std::unique_ptr<HistoryData> history_data_;
  std::unique_ptr<AgentMetrics> metrics_;

  std::unique_ptr<EdgeCursor> active_cursor_;

  Node* active_node_ = nullptr;
  Layer* active_layer_ = nullptr;
  State state_ = State::kIdle;
  CommandPhase command_phase_ = CommandPhase::kNone;
  ExecutionStatus last_execution_status_ = ExecutionStatus::kAwaitingCommandResult;
  std::unique_ptr<RobotState> initial_state_;
  std::unique_ptr<RobotState> current_state_;
  double current_cost_budget_ = std::numeric_limits<double>::infinity();
  int current_map_index_ = 0;
  std::chrono::high_resolution_clock::time_point replan_start_wall_;
  /// Monotonic clock captured at episode start; drives AgentMetrics::elapsed_wall_time_s.
  std::chrono::steady_clock::time_point episode_start_steady_;

  // The current settle wait; its diagnostics start once it has lasted a second, then repeat at most once a second.
  std::chrono::high_resolution_clock::time_point last_settle_log_time_;
  std::chrono::high_resolution_clock::time_point settle_wait_start_;
  const Edge* settle_wait_edge_ = nullptr;
  int settle_wait_idx_ = -1;

  PerformanceLog performance_log_;
  CsvWriter map_log_;                             ///< Map-snapshot log; per-row flush like performance_log_.
  bool run_output_configured_ = false;            ///< Set once configureRunOutput() succeeds.
  bool episode_output_finalized_ = false;         ///< Set by finalizeEpisodeOutput(); cleared by clearEpisodeState() and resumeEpisode().
  std::vector<std::string> supplied_param_keys_;  ///< Keys of the setup map, for reportUnreadParams().

  double p_target_position_threshold_ = 0.10;
  double p_target_angle_threshold_ = 0.1;
  double p_observed_viewpoint_position_threshold_ = 0.0;  ///< Position threshold of HistoryData::getObservedViewpoints() [m], as read.
  double p_observed_viewpoint_angle_threshold_ = 0.0;     ///< Orientation threshold of HistoryData::getObservedViewpoints() [rad], as read.
  int p_random_seed_ = -1;
  double p_cost_budget_ = std::numeric_limits<double>::infinity();
  int p_num_maps_to_save_ = 0;
  std::string run_dir_;

  bool p_visualize_ = true;
  bool p_verbose_ = true;
  bool p_debug_diagnostics_ = false;
  bool p_log_performance_ = false;
  bool p_wait_for_map_updated_ = false;
  bool p_color_past_trajectory_by_budget_ = false;
};

inline void Agent::respondToHit(bool /*hit_really_happened*/) {}

}  // namespace open_apero
