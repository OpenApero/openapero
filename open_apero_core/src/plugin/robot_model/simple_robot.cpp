#include "open_apero_core/plugin/robot_model/simple_robot.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/bounding_volume.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/plugin/map/tsdf_map.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/third_party/tinycolormap.h"
#include "open_apero_core/visualization/colormap.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {
namespace robot_model {

PluginRegistry::Registration<SimpleRobot> SimpleRobot::registration("SimpleRobot");

SimpleRobot::SimpleRobot(AgentInterface& agent) : RobotModel(agent) {}

void SimpleRobot::setupFromParamMap(Configurable::ParamMap* param_map) {
  RobotModel::setupFromParamMap(param_map);

  {
    std::vector<double> collision_sphere_x, collision_sphere_y, collision_sphere_z, collision_sphere_r;
    setParam<double>(param_map, "collision_sphere_x", &collision_sphere_x, std::vector<double>());
    setParam<double>(param_map, "collision_sphere_y", &collision_sphere_y, std::vector<double>());
    setParam<double>(param_map, "collision_sphere_z", &collision_sphere_z, std::vector<double>());
    setParam<double>(param_map, "collision_sphere_r", &collision_sphere_r, std::vector<double>());

    std::vector<double> collision_sphere_safe_margins;
    setParam<double>(param_map, "collision_sphere_safe_margins", &collision_sphere_safe_margins, std::vector<double>());  // optional
    if (collision_sphere_safe_margins.empty()) {
      collision_sphere_safe_margins.assign(collision_sphere_x.size(), 0.0);
    }

    if (collision_sphere_x.size() != collision_sphere_y.size() || collision_sphere_x.size() != collision_sphere_z.size() ||
        collision_sphere_x.size() != collision_sphere_r.size() || collision_sphere_x.size() != collision_sphere_safe_margins.size()) {
      throw std::runtime_error("Mismatch in parameter array sizes for collision spheres.");
    }

    p_collision_spheres_.clear();
    for (size_t i = 0; i < collision_sphere_x.size(); ++i) {
      GeometricSphere sphere;
      sphere.center = Eigen::Vector3d(collision_sphere_x[i], collision_sphere_y[i], collision_sphere_z[i]);
      sphere.radius = collision_sphere_r[i];
      sphere.safe_margin = collision_sphere_safe_margins[i];
      p_collision_spheres_.push_back(sphere);
    }
  }

  {
    std::vector<double> contact_sphere_x, contact_sphere_y, contact_sphere_z, contact_sphere_r;
    setParam<double>(param_map, "contact_sphere_x", &contact_sphere_x, std::vector<double>());
    setParam<double>(param_map, "contact_sphere_y", &contact_sphere_y, std::vector<double>());
    setParam<double>(param_map, "contact_sphere_z", &contact_sphere_z, std::vector<double>());
    setParam<double>(param_map, "contact_sphere_r", &contact_sphere_r, std::vector<double>());

    if (contact_sphere_x.size() != contact_sphere_y.size() || contact_sphere_x.size() != contact_sphere_z.size() ||
        contact_sphere_x.size() != contact_sphere_r.size()) {
      throw std::runtime_error("Mismatch in parameter array sizes for contact spheres.");
    }

    p_contact_spheres_.clear();
    for (size_t i = 0; i < contact_sphere_x.size(); ++i) {
      GeometricSphere sphere;
      sphere.center = Eigen::Vector3d(contact_sphere_x[i], contact_sphere_y[i], contact_sphere_z[i]);
      sphere.radius = contact_sphere_r[i];
      sphere.safe_margin = 0.0;
      p_contact_spheres_.push_back(sphere);
    }
  }

  configureCollisionCheckYaw(param_map);

  setParam<int>(param_map, "min_observed_spheres", &p_min_observed_spheres_, -1);
  setParam<int>(param_map, "collision_sphere_visualization_stride", &p_collision_sphere_visualization_stride_, 1);

  setParam<bool>(param_map, "sample_z", &p_sample_z_, true);

  setParam<bool>(param_map, "restrict_sampling_to_mapped_region", &p_restrict_sampling_to_mapped_region_, false);

  std::string temp_args;
  std::string ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "bounding_volume_args", &temp_args, ns + "/bounding_volume");
  bounding_volume_ = std::make_unique<BoundingVolume>();
  bounding_volume_->setInfoLogger([this](const std::string& msg) { agent_.printInfo(msg); });
  agent_.getFactory().configure(*bounding_volume_, temp_args, verbose_plugins_);

  rng_.seed(agent_.getRandomSeed(), ns);
}

void SimpleRobot::resetEpisode() {
  RobotModel::resetEpisode();
  // BoundingVolume is a stateless geometry type: nothing to reset per episode.
  rng_.reseed(agent_.getRandomSeed());
  // The map starts empty again, so last episode's region says nothing about this one.
  clearMappedRegion();
}

void SimpleRobot::update() {
  RobotModel::update();
  if (!p_restrict_sampling_to_mapped_region_) {
    return;
  }
  // A map with nothing in it yet reports no region and leaves the extents
  // untouched, so they have to be cleared rather than left holding last cycle's.
  if (!getMap()->getTotalMapRegion(&mapped_region_min_, &mapped_region_max_)) {
    clearMappedRegion();
  }
}

void SimpleRobot::clearMappedRegion() {
  mapped_region_min_.setConstant(kDoubleNan);
  mapped_region_max_.setConstant(kDoubleNan);
}

bool SimpleRobot::getSamplingBounds(Eigen::Vector3d* sampling_min, Eigen::Vector3d* sampling_max) const {
  if (!bounding_volume_) {
    return false;
  }

  *sampling_min = Eigen::Vector3d(bounding_volume_->x_min, bounding_volume_->y_min, bounding_volume_->z_min);
  *sampling_max = Eigen::Vector3d(bounding_volume_->x_max, bounding_volume_->y_max, bounding_volume_->z_max);

  // The componentwise intersection is only frame-correct while the configured
  // volume is unrotated: its extents are expressed in its rotated frame whereas
  // the mapped region is axis-aligned in the world frame.
  if (!p_restrict_sampling_to_mapped_region_ || !hasMappedRegion() || bounding_volume_->rotation != 0.0) {
    return true;
  }

  Eigen::Vector3d mapped_min;
  Eigen::Vector3d mapped_max;
  mapped_min.x() = std::max(sampling_min->x(), mapped_region_min_.x());
  mapped_max.x() = std::min(sampling_max->x(), mapped_region_max_.x());
  mapped_min.y() = std::max(sampling_min->y(), mapped_region_min_.y());
  mapped_max.y() = std::min(sampling_max->y(), mapped_region_max_.y());
  // A fixed z is supplied by the caller rather than drawn, so map height cannot
  // make that horizontal sampling region empty.
  mapped_min.z() = p_sample_z_ ? std::max(sampling_min->z(), mapped_region_min_.z()) : sampling_min->z();
  mapped_max.z() = p_sample_z_ ? std::min(sampling_max->z(), mapped_region_max_.z()) : sampling_max->z();

  if ((mapped_min.array() <= mapped_max.array()).all()) {
    *sampling_min = mapped_min;
    *sampling_max = mapped_max;
  }
  return true;
}

void SimpleRobot::addSamplingBoundsVisualization(VisualizationMarkers* markers) const {
  if (!bounding_volume_ || !bounding_volume_->is_setup) {
    return;
  }

  // Orange wireframe: the static hard admissibility constraint.
  bounding_volume_->visualize(markers);

  Eigen::Vector3d sampling_min;
  Eigen::Vector3d sampling_max;
  if (!getSamplingBounds(&sampling_min, &sampling_max)) {
    return;
  }

  // A fixed-height planner samples a plane, not the whole static z band. Give
  // that plane a little thickness so it remains visible from oblique views.
  constexpr double kPlaneThickness = 0.04;
  if (!p_sample_z_) {
    const double z = agent_.getInitialSensorPosition().z();
    sampling_min.z() = z - kPlaneThickness / 2.0;
    sampling_max.z() = z + kPlaneThickness / 2.0;
  }

  // Teal fill: the region from which samplePosition actually draws this cycle,
  // whether it is map-intersected or a fallback to the configured volume.
  VisualizationMarker fill;
  fill.ns = "sampling_volume";
  fill.type = VisualizationMarker::CUBE;
  fill.position = (sampling_min + sampling_max) / 2.0;
  fill.scale = (sampling_max - sampling_min).cwiseMax(Eigen::Vector3d::Constant(kPlaneThickness));
  fill.color = Color::teal();
  fill.color.a = 0.14;
  markers->addMarker(fill);

  // A matching outline keeps the bounds readable over a dense map.
  const Eigen::Vector3d p1(sampling_min.x(), sampling_min.y(), sampling_min.z());
  const Eigen::Vector3d p2(sampling_max.x(), sampling_min.y(), sampling_min.z());
  const Eigen::Vector3d p3(sampling_max.x(), sampling_max.y(), sampling_min.z());
  const Eigen::Vector3d p4(sampling_min.x(), sampling_max.y(), sampling_min.z());
  const Eigen::Vector3d p5(sampling_min.x(), sampling_min.y(), sampling_max.z());
  const Eigen::Vector3d p6(sampling_max.x(), sampling_min.y(), sampling_max.z());
  const Eigen::Vector3d p7(sampling_max.x(), sampling_max.y(), sampling_max.z());
  const Eigen::Vector3d p8(sampling_min.x(), sampling_max.y(), sampling_max.z());

  VisualizationMarker outline;
  outline.ns = "sampling_volume";
  outline.type = VisualizationMarker::LINE_LIST;
  outline.scale.x() = 0.06;
  outline.color = Color::teal();
  outline.color.a = 0.9;
  const auto add_edge = [&outline](const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
    outline.points.push_back(from);
    outline.points.push_back(to);
  };
  add_edge(p1, p2);
  add_edge(p2, p3);
  add_edge(p3, p4);
  add_edge(p4, p1);
  add_edge(p5, p6);
  add_edge(p6, p7);
  add_edge(p7, p8);
  add_edge(p8, p5);
  add_edge(p1, p5);
  add_edge(p2, p6);
  add_edge(p3, p7);
  add_edge(p4, p8);
  markers->addMarker(outline);
}

bool SimpleRobot::passesPlannerConstraints(const RobotState& state, Map* map) const {
  if (bounding_volume_ && !bounding_volume_->contains(state.getSensorPosition())) {
    return false;
  }
  if (map) {
    GeometryPrimitiveCollection hit_regions;
    map->getHitRegions(hit_regions);
    if (isInsideHitRegions(state, hit_regions)) {
      return false;
    }
  }
  return true;
}

bool SimpleRobot::samplePosition(Eigen::Vector3d* pos, double z_default) const {
  Eigen::Vector3d sampling_min;
  Eigen::Vector3d sampling_max;
  if (!getSamplingBounds(&sampling_min, &sampling_max)) {
    return false;
  }

  std::uniform_real_distribution<double> unit(0.0, 1.0);
  (*pos)[0] = sampling_min.x() + unit(rng_.engine) * (sampling_max.x() - sampling_min.x());
  (*pos)[1] = sampling_min.y() + unit(rng_.engine) * (sampling_max.y() - sampling_min.y());
  (*pos)[2] = p_sample_z_ ? sampling_min.z() + unit(rng_.engine) * (sampling_max.z() - sampling_min.z()) : z_default;
  return true;
}

bool SimpleRobot::samplePositionNear(const Eigen::Vector3d& center, double max_distance_3d, double max_distance_xy, Eigen::Vector3d* pos,
                                     double z_default) const {
  if (!bounding_volume_ || max_distance_3d <= 0.0 || max_distance_xy <= 0.0) {
    return false;
  }
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  if (!p_sample_z_) {
    // Every sample lands on the z_default plane, so the 3D bound reduces to a
    // disk radius there: beyond this height gap the plane is out of reach.
    const double height_gap = std::abs(center.z() - z_default);
    if (height_gap >= max_distance_3d) {
      return false;
    }
    const double radius = std::min(std::sqrt(max_distance_3d * max_distance_3d - height_gap * height_gap), max_distance_xy);
    // sqrt keeps the draw uniform over the disk rather than biased to the center.
    const double sampled_radius = radius * std::sqrt(unit(rng_.engine));
    const double angle = 2.0 * M_PI * unit(rng_.engine);
    (*pos)[0] = center.x() + sampled_radius * std::cos(angle);
    (*pos)[1] = center.y() + sampled_radius * std::sin(angle);
    (*pos)[2] = z_default;
    return bounding_volume_->contains(*pos);
  }

  // Free z: draw from the ball and reject what the horizontal bound or the
  // operating volume excludes. Both carve the ball down rather than reshaping it,
  // so a bounded number of tries is enough.
  constexpr int kMaxTries = 10;
  for (int i = 0; i < kMaxTries; ++i) {
    // cbrt for a uniform draw over the ball's volume, direction from a normal.
    const double sampled_radius = max_distance_3d * std::cbrt(unit(rng_.engine));
    std::normal_distribution<double> normal(0.0, 1.0);
    Eigen::Vector3d direction(normal(rng_.engine), normal(rng_.engine), normal(rng_.engine));
    const double norm = direction.norm();
    if (norm < kEpsilon) {
      continue;
    }
    *pos = center + sampled_radius * (direction / norm);
    if ((pos->head<2>() - center.head<2>()).norm() <= max_distance_xy && bounding_volume_->contains(*pos)) {
      return true;
    }
  }
  return false;
}

bool SimpleRobot::checkParamsValid(std::string* error_message) {
  if (!RobotModel::checkParamsValid(error_message)) {
    return false;
  }

  if (!agent_.getMap().as<map::TsdfMap>()) {
    *error_message = "SimpleRobot requires a TsdfMap-derived map (got a non-TSDF map)";
    return false;
  }

  const int num_spheres = static_cast<int>(p_collision_spheres_.size() + p_contact_spheres_.size());
  if (p_min_observed_spheres_ > num_spheres) {
    *error_message = "min_observed_spheres expected <= " + std::to_string(num_spheres) +
                     " (the number of configured collision and contact spheres); a higher threshold can never be met";
    return false;
  }

  return true;
}

bool SimpleRobot::isTraversabilityOrientationRelated() const {
  return p_collision_check_yaw_;
}

bool SimpleRobot::deduceCollisionCheckYaw(std::string* source) const {
  const auto is_off_axis = [](const GeometricSphere& sphere) {
    return std::abs(sphere.center.x()) > 1e-4 || std::abs(sphere.center.y()) > 1e-4;
  };
  const bool collision_off_axis = std::any_of(p_collision_spheres_.begin(), p_collision_spheres_.end(), is_off_axis);
  const bool contact_off_axis = std::any_of(p_contact_spheres_.begin(), p_contact_spheres_.end(), is_off_axis);

  if (source) {
    if (collision_off_axis && contact_off_axis) {
      *source = "off-axis collision and contact spheres";
    } else if (collision_off_axis) {
      *source = "off-axis collision spheres";
    } else if (contact_off_axis) {
      *source = "off-axis contact spheres";
    } else {
      *source = "centered sphere geometry";
    }
  }
  return collision_off_axis || contact_off_axis;
}

void SimpleRobot::configureCollisionCheckYaw(Configurable::ParamMap* param_map) {
  std::string source;
  const bool deduced = deduceCollisionCheckYaw(&source);
  const bool explicitly_configured = param_map->find("collision_check_yaw") != param_map->end();
  bool configured = deduced;
  setParam<bool>(param_map, "collision_check_yaw", &configured, deduced);

  if (explicitly_configured && configured != deduced) {
    agent_.printWarning(std::string("SimpleRobot: collision_check_yaw=") + (configured ? "true" : "false") +
                        " overrides the deduced value " + (deduced ? "true" : "false") + " (" + source +
                        "); honoring the configured value.");
  }

  p_collision_check_yaw_ = configured;
  agent_.printInfo(std::string("[SimpleRobot] collision_check_yaw=") + (p_collision_check_yaw_ ? "true" : "false") +
                   (explicitly_configured ? " (configured; deduced " : " (deduced ") + (deduced ? "true" : "false") + " from " + source +
                   ").");
}

bool SimpleRobot::getGeometricSpheresWorldFrame(const RobotState* robot_state, GeometricSpheres& spheres, bool use_contact_spheres) const {
  spheres.clear();

  if (use_contact_spheres) {
    spheres = p_contact_spheres_;
  } else {
    spheres = p_collision_spheres_;
  }

  for (auto& sphere : spheres) {
    sphere.center = robot_state->getBaseTransform() * sphere.center;
  }

  return true;
}

bool SimpleRobot::getBodyBounds(const RobotState& state, Eigen::Vector3d* min, Eigen::Vector3d* max) const {
  GeometricSpheres spheres;
  GeometricSpheres contact_spheres;
  if (!getCollisionSpheresWorldFrame(&state, spheres) || !getContactSpheresWorldFrame(&state, contact_spheres)) {
    return false;
  }
  spheres.insert(spheres.end(), contact_spheres.begin(), contact_spheres.end());
  if (spheres.empty()) {
    return false;
  }

  // The margin counts: getSphereClearance reads the distance field out to radius + safe_margin.
  *min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  *max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const auto& sphere : spheres) {
    const Eigen::Vector3d extent = Eigen::Vector3d::Constant(sphere.radius + sphere.safe_margin);
    *min = min->cwiseMin(sphere.center - extent);
    *max = max->cwiseMax(sphere.center + extent);
  }
  return true;
}

bool SimpleRobot::planTrajectory(const RobotState& start, const RobotState& goal, Map* map, RobotTrajectory* trajectory) {
  *trajectory = steer(start, goal, map);
  if (trajectory->empty()) {
    return false;
  }
  return isTrajectoryTraversable(*trajectory, map);
}

RobotTrajectory SimpleRobot::steer(const RobotState& start, const RobotState& goal, Map* map) const {
  const double max_walk_step = getMaxLinearVelocity() * getTimeStep();
  const double max_yaw_step = getMaxYawRate() * getTimeStep();

  constexpr double kSteerEpsilon = 1e-12;  // Position and quaternion-dot epsilon; 1e-12 on the dot is 2.8e-6 rad.
  // The rotation appendMotionState treats as the same pose; no last step may fall below it.
  const double kSameOrientationAngle = orientationDotEpsilonToAngleThreshold(kEpsilon);

  // Execution requires motion, not a single-state trajectory. Use the same pose
  // tolerance as caller-side duplicate checks.
  if (start.isSameSensorPose(goal)) {
    throw std::runtime_error("SimpleRobot::steer: coincident start/goal sensor poses -- callers must skip identical-pose pairs.");
  }

  RobotTrajectory trajectory;
  trajectory.push_back(start);

  RobotState current = start;
  while (!current.isSameSensorPose(goal, kSteerEpsilon, kSteerEpsilon)) {
    Eigen::Vector3d walk_to_goal_vector = goal.getSensorPosition() - current.getSensorPosition();
    double walk_to_goal_vector_norm = walk_to_goal_vector.norm();

    if (walk_to_goal_vector_norm <= kSteerEpsilon) {
      double yaw_to_goal = angleDifference(current.getSensorYaw(), goal.getSensorYaw());
      double yaw_to_goal_abs = std::abs(yaw_to_goal);
      if (yaw_to_goal_abs > kSteerEpsilon) {
        auto yaw_quaternion = quaternionFromYaw(current.getSensorYaw() + rateStep(yaw_to_goal, max_yaw_step, kSameOrientationAngle));
        placeRobotAtSensorPose(&current, {current.getSensorPosition(), yaw_quaternion}, map);
      } else {
        throw std::runtime_error("SimpleRobot::steer: position and yaw converged but isSameSensorPose returned false");
      }
    } else {
      Eigen::Vector3d walk_to_goal_unit_vector = walk_to_goal_vector / walk_to_goal_vector_norm;
      double traveling_yaw = std::atan2(walk_to_goal_vector.y(), walk_to_goal_vector.x());
      double yaw_to_traveling = angleDifference(current.getSensorYaw(), traveling_yaw);
      double yaw_to_traveling_abs = std::abs(yaw_to_traveling);

      if (yaw_to_traveling_abs > kSteerEpsilon) {
        auto yaw_quaternion = quaternionFromYaw(current.getSensorYaw() + rateStep(yaw_to_traveling, max_yaw_step, kSameOrientationAngle));
        placeRobotAtSensorPose(&current, {current.getSensorPosition(), yaw_quaternion}, map);
      } else {
        double walk_step_size = std::min(max_walk_step, walk_to_goal_vector_norm);
        placeRobotAtSensorPose(
            &current, {current.getSensorPosition() + walk_step_size * walk_to_goal_unit_vector, current.getSensorOrientation()}, map);
      }
    }

    // Skips sub-tolerance residual steps, which splitting could turn into zero-motion edges.
    appendMotionState(&trajectory, current);
  }

  // Ends exactly on goal, absorbing a skipped residual; throws if a zero-motion step survived anyway.
  closeTrajectoryAtState(&trajectory, goal);

  setUniformTrajectoryTimes(trajectory, getTimeStep());

  if (trajectory.size() < 2) {
    throw std::logic_error("SimpleRobot::steer generated a trajectory with less than 2 points despite non-coincident start/goal.");
  }

  if (!trajectory.back().isSameSensorPose(goal, kSteerEpsilon, kSteerEpsilon)) {
    throw std::runtime_error("SimpleRobot::steer: end of trajectory does not match goal state");
  }

  return trajectory;
}

Traversability SimpleRobot::placeRobotAtSensorPose(RobotState* robot_state, const Transform3d& sensor_transform, Map*) const {
  robot_state->setSensorTransform(sensor_transform);
  robot_state->setBaseTransform(sensor_transform * p_base_in_sensor_transform_);
  return Traversability::kUnset;
}

Traversability SimpleRobot::getStateTraversabilityRaw(const RobotState& state, Map* map) const {
  if (!passesPlannerConstraints(state, map)) {
    return Traversability::kUntraversable;
  }

  if (!map) {
    return Traversability::kUntraversable;
  }

  auto* tsdf_map = map->as<map::TsdfMap>();
  if (!tsdf_map) {
    throw std::runtime_error("SimpleRobot requires a TsdfMap-derived map for traversability queries.");
  }

  GeometricSpheres collision_spheres;
  if (!getCollisionSpheresWorldFrame(&state, collision_spheres)) {
    return Traversability::kUntraversable;
  }

  GeometricSpheres contact_spheres;
  if (!getContactSpheresWorldFrame(&state, contact_spheres)) {
    return Traversability::kUntraversable;
  }

  int num_observed = 0;
  for (const auto& sphere : collision_spheres) {
    const CollisionStatus status = tsdf_map->getSphereCollisionStatus(sphere.center, sphere.radius);
    if (status == CollisionStatus::kColliding) {
      return Traversability::kUntraversable;
    }
    if (status == CollisionStatus::kNotColliding) {
      ++num_observed;
    }
  }

  for (const auto& sphere : contact_spheres) {
    const CollisionStatus status = tsdf_map->getSphereCollisionStatus(sphere.center, sphere.radius);
    if (status == CollisionStatus::kNotColliding) {
      return Traversability::kUntraversable;
    }
    if (status == CollisionStatus::kColliding) {
      ++num_observed;
    }
  }

  // "All spheres" counts the spheres placed for this state, not the configured ones.
  const int num_spheres = static_cast<int>(collision_spheres.size() + contact_spheres.size());
  const int num_required = p_min_observed_spheres_ < 0 ? num_spheres : p_min_observed_spheres_;

  return num_observed >= num_required ? Traversability::kTraversable : Traversability::kPotentiallyTraversable;
}

double SimpleRobot::getStateClearance(const RobotState& state, Map* map) const {
  if (!map) {
    return 1.0;
  }
  auto* tsdf_map = map->as<map::TsdfMap>();
  if (!tsdf_map) {
    return 1.0;
  }

  GeometricSpheres collision_spheres;
  if (!getCollisionSpheresWorldFrame(&state, collision_spheres)) {
    // The body could not be placed, so no part of it can be vouched for; report
    // the worst clearance rather than the full margin a caller would trust.
    return p_clearance_margin_eps_;
  }

  double clearance = 1.0;
  for (const auto& sphere : collision_spheres) {
    clearance = std::min(clearance, getSphereClearance(sphere, tsdf_map));
  }
  return clearance;
}

double SimpleRobot::getSphereClearance(const GeometricSphere& sphere, map::TsdfMap* map) const {
  if (sphere.safe_margin <= kEpsilon) {
    return 1.0;
  }
  double distance = 0.0;
  if (!map->getDistanceAtPosition(sphere.center, &distance)) {
    return 1.0;
  }
  const double relative_margin = (distance - sphere.radius) / sphere.safe_margin;
  return std::min(1.0, std::max(relative_margin, p_clearance_margin_eps_));
}

void SimpleRobot::getVisualizationImpl(const RobotTrajectory& trajectory, VisualizationMarkers* markers) const {
  addSamplingBoundsVisualization(markers);

  if (trajectory.empty()) {
    return;
  }

  // The shells are colored from the same distance field the stationing rule reads.
  Map* map = getMap();
  auto* tsdf_map = map ? map->as<map::TsdfMap>() : nullptr;

  constexpr double kMarginAlpha = 0.3;
  const int stride = std::max(1, p_collision_sphere_visualization_stride_);
  const int last_index = static_cast<int>(trajectory.size()) - 1;

  for (int index = 0; index <= last_index; ++index) {
    // The endpoints are the poses the stationing rule actually judges, so they are
    // drawn whatever the stride skips in between.
    if (index % stride != 0 && index != last_index) {
      continue;
    }
    const RobotState& robot_state = trajectory[index];

    GeometricSpheres collision_spheres_world_frame;
    getCollisionSpheresWorldFrame(&robot_state, collision_spheres_world_frame);
    for (const auto& collision_sphere : collision_spheres_world_frame) {
      VisualizationMarker marker;
      marker.type = VisualizationMarker::SPHERE;
      marker.action = VisualizationMarker::ADD;
      marker.ns = "collision_sphere";
      marker.orientation.w() = 1.0;
      marker.position = collision_sphere.center;
      double r = collision_sphere.radius;
      marker.scale.x() = r * 2;
      marker.scale.y() = r * 2;
      marker.scale.z() = r * 2;
      marker.color = Color::gray();
      markers->addMarker(marker);

      if (collision_sphere.safe_margin > kEpsilon) {
        VisualizationMarker margin_marker;
        margin_marker.type = VisualizationMarker::SPHERE;
        margin_marker.action = VisualizationMarker::ADD;
        margin_marker.ns = "collision_sphere_safe_margin";
        margin_marker.orientation.w() = 1.0;
        margin_marker.position = collision_sphere.center;
        double rm = collision_sphere.radius + collision_sphere.safe_margin;
        margin_marker.scale.x() = rm * 2;
        margin_marker.scale.y() = rm * 2;
        margin_marker.scale.z() = rm * 2;
        if (tsdf_map) {
          // Colored by margin spent: cool is untouched, red grazes an obstacle.
          const double spent = 1.0 - getSphereClearance(collision_sphere, tsdf_map);
          margin_marker.color = getColorFromColormap(spent, 0.0, 1.0, kMarginAlpha, tinycolormap::ColormapType::Turbo);
        } else {
          margin_marker.color = Color::gray();
          margin_marker.color.a = kMarginAlpha;
        }
        markers->addMarker(margin_marker);
      }
    }

    GeometricSpheres contact_spheres_world_frame;
    getContactSpheresWorldFrame(&robot_state, contact_spheres_world_frame);
    for (const auto& contact_sphere : contact_spheres_world_frame) {
      VisualizationMarker marker;
      marker.type = VisualizationMarker::SPHERE;
      marker.action = VisualizationMarker::ADD;
      marker.ns = "contact_sphere";
      marker.orientation.w() = 1.0;
      marker.position = contact_sphere.center;
      double r = contact_sphere.radius;
      marker.scale.x() = r * 2;
      marker.scale.y() = r * 2;
      marker.scale.z() = r * 2;
      marker.color = Color::white();
      markers->addMarker(marker);
    }
  }
}

}  // namespace robot_model
}  // namespace open_apero
