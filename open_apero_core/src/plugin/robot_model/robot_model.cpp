#include "open_apero_core/plugin/robot_model/robot_model.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/plugin/map/map.h"

namespace open_apero {

RobotModel::RobotModel(AgentInterface& agent) : Plugin(agent) {}

void RobotModel::setupFromParamMap(Configurable::ParamMap* param_map) {
  setParam<double>(param_map, "max_linear_velocity", &p_max_linear_velocity_, 1.0);
  setParam<double>(param_map, "max_yaw_rate", &p_max_yaw_rate_, M_PI / 2.0);
  setParam<double>(param_map, "time_step", &p_time_step_, 0.05);

  setParam<bool>(param_map, "visualize", &p_visualize_, true);
  setParam<double>(param_map, "safe_radius", &p_safe_radius_, 0.0);
  setParam<double>(param_map, "clearance_margin_eps", &p_clearance_margin_eps_, 1e-4);

  setParam<int>(param_map, "num_orientation_candidates", &p_num_orientation_candidates_, 12);

  std::string sensor_mount_type_name;
  setParam<std::string>(param_map, "sensor_mount_type", &sensor_mount_type_name, "fixed");
  std::transform(sensor_mount_type_name.begin(), sensor_mount_type_name.end(), sensor_mount_type_name.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  if (sensor_mount_type_name == "fixed") {
    p_sensor_mount_type_ = SensorMountType::kFixed;
  } else if (sensor_mount_type_name == "free_pitch") {
    p_sensor_mount_type_ = SensorMountType::kFreePitch;
  } else if (sensor_mount_type_name == "unconstrained") {
    p_sensor_mount_type_ = SensorMountType::kUnconstrained;
  } else {
    throw std::runtime_error("Invalid sensor_mount_type: " + sensor_mount_type_name);
  }

  // Default: the full roll-free range.
  setParam<double>(param_map, "gaze_pitch_min", &p_gaze_pitch_min_, -M_PI_2);
  setParam<double>(param_map, "gaze_pitch_max", &p_gaze_pitch_max_, M_PI_2);

  // Base pose in the sensor frame.
  setParam<double>(param_map, "base_in_sensor_position_x", &p_base_in_sensor_transform_.position.x(), 0.0);
  setParam<double>(param_map, "base_in_sensor_position_y", &p_base_in_sensor_transform_.position.y(), 0.0);
  setParam<double>(param_map, "base_in_sensor_position_z", &p_base_in_sensor_transform_.position.z(), 0.0);

  setParam<double>(param_map, "base_in_sensor_orientation_x", &p_base_in_sensor_transform_.orientation.x(), 0.0);
  setParam<double>(param_map, "base_in_sensor_orientation_y", &p_base_in_sensor_transform_.orientation.y(), 0.0);
  setParam<double>(param_map, "base_in_sensor_orientation_z", &p_base_in_sensor_transform_.orientation.z(), 0.0);
  setParam<double>(param_map, "base_in_sensor_orientation_w", &p_base_in_sensor_transform_.orientation.w(), 1.0);
}

bool RobotModel::checkParamsValid(std::string* error_message) {
  if (p_max_linear_velocity_ <= 0.0) {
    *error_message = "max_linear_velocity expected > 0.0";
    return false;
  } else if (p_max_yaw_rate_ <= 0.0) {
    *error_message = "max_yaw_rate expected > 0.0";
    return false;
  } else if (p_time_step_ <= 0.0) {
    *error_message = "time_step expected > 0.0";
    return false;
  } else if (p_num_orientation_candidates_ <= 0) {
    *error_message = "num_orientation_candidates expected > 0";
    return false;
  } else if (p_clearance_margin_eps_ <= 0.0 || p_clearance_margin_eps_ > 1.0) {
    *error_message = "clearance_margin_eps expected in (0.0, 1.0]";
    return false;
  }

  if (!std::isfinite(p_gaze_pitch_min_) || !std::isfinite(p_gaze_pitch_max_) || p_gaze_pitch_min_ > p_gaze_pitch_max_) {
    *error_message = "gaze pitch band must be finite, with gaze_pitch_min <= gaze_pitch_max";
    return false;
  }
  if (p_gaze_pitch_min_ < -M_PI_2 || p_gaze_pitch_max_ > M_PI_2) {
    *error_message = "gaze pitch band must lie within [-pi/2, pi/2]";
    return false;
  }

  return true;
}

Map* RobotModel::getMap() const {
  return &agent_.getMap();
}

bool RobotModel::isWithinSafeRadius(const RobotState& state) const {
  if (p_safe_radius_ <= 0.0) {
    return false;
  }

  return (agent_.getInitialSensorPosition() - state.getSensorPosition()).norm() <= p_safe_radius_;
}

bool RobotModel::isStateTraversable(const RobotState& state, Map* map) const {
  switch (getStateTraversabilityRaw(state, map)) {
    case Traversability::kTraversable:
      return true;
    case Traversability::kUntraversable:
      return false;
    case Traversability::kPotentiallyTraversable:
      // Trust unobserved space only near the known-safe start.
      return isWithinSafeRadius(state);
    case Traversability::kUnset:
      throw std::logic_error(
          "RobotModel::isStateTraversable: getStateTraversabilityRaw returned UNSET (raw layer must commit to a verdict)");
  }

  throw std::logic_error("RobotModel::isStateTraversable: unhandled Traversability value");
}

bool RobotModel::isStateSafe(const RobotState& state, Map* map) const {
  if (!isStateTraversable(state, map)) {
    return false;
  }
  // Clearance saturates at exactly 1.0 when the margin is met, but it is reached
  // through a division, so compare with a tolerance rather than against 1.0.
  constexpr double kFullClearanceTolerance = 1e-4;
  return getStateClearance(state, map) > 1.0 - kFullClearanceTolerance;
}

bool RobotModel::planRotationTrajectory(const RobotState& start, const RobotState& goal, Map* map, RobotTrajectory* trajectory) {
  *trajectory = steer(start, goal, map);
  if (trajectory->empty()) {
    return false;
  }
  // Omnidirectional robots: rotation cannot affect collision, accept unconditionally.
  if (!isTraversabilityOrientationRelated()) {
    return true;
  }
  return isTrajectoryTraversable(*trajectory, map);
}

void RobotModel::updateHitRegions(const RobotState& /*target_state*/, const RobotState& /*stopped_state*/,
                                  GeometryPrimitiveCollection* /*hit_regions*/) const {}

bool RobotModel::isInsideHitRegions(const RobotState& /*robot_state*/, const GeometryPrimitiveCollection& /*hit_regions*/) const {
  return false;
}

bool RobotModel::getBodyBounds(const RobotState& /*state*/, Eigen::Vector3d* /*min*/, Eigen::Vector3d* /*max*/) const {
  return false;
}

double RobotModel::getStateClearance(const RobotState& /*state*/, Map* /*map*/) const {
  return 1.0;
}

bool RobotModel::isStationingPositionAdmissible(const Eigen::Vector3d& /*position*/, Map* /*map*/) const {
  return true;
}

bool RobotModel::samplePosition(Eigen::Vector3d* /*pos*/, double /*z_default*/) const {
  return false;
}

bool RobotModel::samplePositionNear(const Eigen::Vector3d& /*center*/, double /*max_distance_3d*/, double /*max_distance_xy*/,
                                    Eigen::Vector3d* /*pos*/, double /*z_default*/) const {
  return false;
}

void RobotModel::attachSharedGainAnchors(const Eigen::Vector3d& /*position*/, const NodeClusterPtrs& /*cluster*/) const {}

void RobotModel::refineBeforeExecute(RobotTrajectory& /*trajectory*/, int /*num_committed_states*/) {}

void RobotModel::sampleOrientations(const Eigen::Vector3d& /*position*/, std::vector<Eigen::Quaterniond>* candidates) const {
  candidates->resize(p_num_orientation_candidates_);
  for (int i = 0; i < p_num_orientation_candidates_; ++i) {
    const double yaw = angleScaled(static_cast<double>(i) * 2.0 * M_PI / p_num_orientation_candidates_);
    candidates->at(i) = quaternionFromYaw(yaw);
  }
}

EdgeList RobotModel::getInplaceAdjacency(const std::vector<RobotState>& states) const {
  if (static_cast<int>(states.size()) != p_num_orientation_candidates_) {
    throw std::invalid_argument("getInplaceAdjacency: states.size() must equal num_orientation_candidates");
  }
  const int n = static_cast<int>(states.size());
  EdgeList edges;
  edges.reserve(2 * n);
  for (int i = 0; i < n; ++i) {
    const int i_fwd = (i + 1) % n;
    const int i_bwd = (i - 1 + n) % n;
    if (i_fwd != i) {
      edges.emplace_back(i, i_fwd);
    }
    if (i_bwd != i && i_bwd != i_fwd) {
      edges.emplace_back(i, i_bwd);
    }
  }
  return edges;
}

EdgeList RobotModel::getTranslationAdjacency(const std::vector<RobotState>& from_states, const std::vector<RobotState>& to_states,
                                             const Eigen::Vector3d& departure_direction, const Eigen::Vector3d& arrival_direction,
                                             bool /*connect_enclosing_cell*/) const {
  if (from_states.empty() || to_states.empty()) {
    return {};
  }

  const Eigen::Vector2d departure_yaw_pitch = yawPitchFromDirection(departure_direction);
  const Eigen::Vector2d arrival_yaw_pitch = yawPitchFromDirection(arrival_direction);

  const int from_idx = bestAlignedState(from_states, departure_yaw_pitch(0), departure_yaw_pitch(1));
  const int to_idx = bestAlignedState(to_states, arrival_yaw_pitch(0), arrival_yaw_pitch(1));

  return {{from_idx, to_idx}};
}

void RobotModel::checkOrientations(const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& candidates,
                                   std::vector<RobotState>* states, std::vector<bool>* is_traversable, StationingCheck stationing) const {
  const int n = static_cast<int>(candidates.size());
  states->resize(n);
  is_traversable->resize(n);

  Map* map = getMap();
  const bool is_traversability_same = !isTraversabilityOrientationRelated();
  // Reused only when is_traversability_same; an orientation-independent body also has an
  // orientation-independent margin, so the cached verdict may include safety.
  std::optional<bool> shared_traversability;

  // One query for the whole cluster: stationing depends on the position alone.
  const bool may_station = stationing == StationingCheck::kExemptBootstrap || isStationingPositionAdmissible(position, map);

  // The clearance margin is part of the stationing rule, so the bootstrap seed is exempt from it too.
  const bool enforce_clearance = stationing == StationingCheck::kEnforce;

  // Placement still runs at a rejected position (callers read the states), but without a map.
  Map* placement_map = may_station ? map : nullptr;

  for (int i = 0; i < n; ++i) {
    Traversability traversability = placeRobotAtSensorPose(&states->at(i), {position, candidates.at(i)}, placement_map);

    // Checked before the stationing short-circuit so a contract breach is never hidden by the verdict.
    if (traversability == Traversability::kPotentiallyTraversable) {
      throw std::logic_error(
          "RobotModel::checkOrientations: placeRobotAtSensorPose returned POTENTIALLY_TRAVERSABLE; placement layer must commit or return "
          "UNSET");
    }

    if (!may_station) {
      is_traversable->at(i) = false;
      continue;
    }

    if (traversability == Traversability::kTraversable) {
      is_traversable->at(i) = true;
    } else if (traversability == Traversability::kUntraversable) {
      is_traversable->at(i) = false;
    } else {  // kUnset — the placement layer declined to commit, so ask the map.
      if (shared_traversability.has_value()) {
        is_traversable->at(i) = *shared_traversability;
      } else {
        is_traversable->at(i) = enforce_clearance ? isStateSafe(states->at(i), map) : isStateTraversable(states->at(i), map);
      }
    }

    if (is_traversability_same && !shared_traversability.has_value()) {
      shared_traversability = is_traversable->at(i);
    }
  }
}

bool RobotModel::retargetTrajectoryTail(RobotTrajectory* trajectory, const RobotState& goal_state) {
  if (trajectory == nullptr || trajectory->empty()) {
    throw std::invalid_argument("Trajectory is null or empty.");
  }

  // Sanity check: goal position must match trajectory end position
  if (!trajectory->back().isSameSensorPosition(goal_state)) {
    throw std::invalid_argument("goal_state position does not match trajectory end position.");
  }

  // Build into a copy so a refusal leaves the caller's trajectory untouched.
  RobotTrajectory result = *trajectory;

  // Find the first state where the trajectory reaches the final position
  auto it = result.begin();
  while (it != result.end()) {
    if (it->isSameSensorPosition(result.back())) {
      break;
    }
    ++it;
  }

  // Trim the trajectory, keeping the first state where the position matches
  if (it != result.end()) {
    result.erase(it + 1, result.end());
  }

  // Same default-kEpsilon tolerance as steer's coincidence check: a sub-kEpsilon residual is "at goal".
  if (!result.back().isSameSensorPose(goal_state)) {
    RobotTrajectory rotation = steer(result.back(), goal_state, getMap());
    result.insert(result.end(), rotation.begin() + 1, rotation.end());
  } else if (result.size() < 2) {
    // A whole-rotation trajectory trims to its first state; with nothing to append the
    // result would be a zero-motion edge nothing downstream may execute. Refuse.
    return false;
  } else {
    // Nothing to append: pin the endpoint to the exact goal so the caller's
    // node state and its edge endpoint match exactly, not just within tolerance.
    result.back() = goal_state;
  }

  setUniformTrajectoryTimes(result, p_time_step_);
  *trajectory = std::move(result);
  return true;
}

bool RobotModel::retargetTrajectoryHead(RobotTrajectory* trajectory, const RobotState& goal_state) {
  if (trajectory == nullptr || trajectory->empty()) {
    throw std::invalid_argument("Trajectory is null or empty.");
  }

  RobotTrajectory reversed = reverseTrajectory(*trajectory);
  if (!retargetTrajectoryTail(&reversed, goal_state)) {
    return false;
  }
  std::reverse(reversed.begin(), reversed.end());

  setUniformTrajectoryTimes(reversed, p_time_step_);
  *trajectory = std::move(reversed);
  return true;
}

bool RobotModel::retargetTrajectoryEndpoints(RobotTrajectory* trajectory, const RobotState& start_state, const RobotState& goal_state) {
  if (trajectory == nullptr || trajectory->empty()) {
    throw std::invalid_argument("Trajectory is null or empty.");
  }

  // Both ends are rebuilt on one copy: retargetTrajectoryHead commits before the tail is
  // attempted, so retargeting in place would leave a half-retargeted trajectory on refusal.
  RobotTrajectory result = *trajectory;
  if (!retargetTrajectoryHead(&result, start_state)) {
    return false;
  }
  if (!retargetTrajectoryTail(&result, goal_state)) {
    return false;
  }

  // Positions are preserved, so for an orientation-independent body the input's validation
  // already covers the result and no map query is needed.
  if (isTraversabilityOrientationRelated() && !isTrajectoryTraversable(result, getMap())) {
    return false;
  }

  *trajectory = std::move(result);
  return true;
}

bool RobotModel::isTrajectoryTraversable(const RobotTrajectory& trajectory, Map* map) const {
  if (trajectory.empty()) {
    return true;
  }
  if (!map) {
    return false;
  }

  for (const auto& state : trajectory) {
    if (!isStateTraversable(state, map)) {
      return false;
    }
  }
  return true;
}

}  // namespace open_apero
