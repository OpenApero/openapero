#pragma once

/**
 * @file node.h
 * @brief Core planning node data structure and associated info types.
 *
 * Defines the Node struct that represents a single planning node with robot state,
 * gain, traversability, and extensible info payload. Also provides NodeInfo
 * subclasses for yaw planning and simulated sensor data, plus Eigen::Vector3d
 * hash/equality utilities.
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/functional/hash.hpp>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {

/**
 * @brief Which sampling population a node belongs to.
 *
 * Provenance owned by the generator, which may update it; evaluators never change it. Generators
 * that sample a single population leave it at kDefault.
 */
enum class SamplingRole : std::uint8_t {
  kDefault,   ///< Population-unaware generator: evaluate with everything the evaluator has.
  kGain,      ///< Sampled for information gain; frontier status is not evaluated.
  kFrontier,  ///< Sampled around a map frontier; only frontier status is evaluated.
};

/**
 * @brief Base class for evaluator-specific information attached to a node.
 *
 * Subclasses carry domain-specific data (e.g., visible voxels, yaw plans).
 * Supports deep copying and polymorphic downcasting.
 */
struct NodeInfo {
  virtual ~NodeInfo() {}

  /**
   * @brief Create a deep copy of this info object.
   * @return A unique pointer to the cloned NodeInfo.
   */
  virtual std::unique_ptr<NodeInfo> deepCopy() const { return std::make_unique<NodeInfo>(*this); }

  /**
   * @brief Downcast to a derived type (const version).
   * @tparam T The target derived type.
   * @return Pointer to the derived type, or nullptr if the cast fails.
   */
  template <class T>
  const T* as() const {
    return dynamic_cast<const T*>(this);
  }

  /**
   * @brief Downcast to a derived type.
   * @tparam T The target derived type.
   * @return Pointer to the derived type, or nullptr if the cast fails.
   */
  template <class T>
  T* as() {
    return dynamic_cast<T*>(this);
  }
};

/**
 * @brief Core planning node containing robot state, gain, traversability, and metadata.
 *
 * Nodes are the fundamental building block of the exploration graph/tree.
 * Each node holds a robot state, information gain anchors, traversability
 * status, and an optional info payload for evaluator-specific data.
 */
struct Node {
 public:
  Node() = default;
  virtual ~Node() {}

  /**
   * @brief Copy constructor. Performs a deep copy of the info payload.
   * @param other The node to copy from.
   */
  Node(const Node& other) : info(other.info ? other.info->deepCopy() : nullptr) { copyFieldsFrom(other); }

  /**
   * @brief Copy assignment operator. Performs a deep copy of the info payload.
   * @param other The node to assign from.
   * @return Reference to this node.
   */
  Node& operator=(const Node& other) {
    if (this != &other) {
      copyFieldsFrom(other);
      info = other.info ? other.info->deepCopy() : nullptr;
    }
    return *this;
  }

  /**
   * @brief Create a shallow copy of the node (excludes info payload).
   * @return A unique pointer to the shallow-copied node.
   */
  inline virtual std::unique_ptr<Node> shallowCopy() {
    auto copy = std::make_unique<Node>();
    copy->copyFieldsFrom(*this);
    return copy;
  }

 protected:
  /// @brief Copies every Node field of @p other except info; a subclass copies its own fields itself.
  void copyFieldsFrom(const Node& other) {
    robot_state = other.robot_state;
    is_candidate_traversable = other.is_candidate_traversable;
    candidate_orientations = other.candidate_orientations;
    gain_anchors = other.gain_anchors;
    is_at_frontier = other.is_at_frontier;
    is_gain_suppressed = other.is_gain_suppressed;
    is_intermediate = other.is_intermediate;
    is_terminating = other.is_terminating;
    is_observed = other.is_observed;
    is_traversable = other.is_traversable;
    sampling_role = other.sampling_role;
  }

 public:
  /**
   * @brief Downcast to a derived node type (const version).
   * @tparam T The target derived type.
   * @return Pointer to the derived type, or nullptr if the cast fails.
   */
  template <class T>
  const T* as() const {
    return dynamic_cast<const T*>(this);
  }

  /**
   * @brief Downcast to a derived node type.
   * @tparam T The target derived type.
   * @return Pointer to the derived type, or nullptr if the cast fails.
   */
  template <class T>
  T* as() {
    return dynamic_cast<T*>(this);
  }

  /**
   * @brief Compute the total gain from all gain anchors attached to this node.
   * @return The sum of all anchor gains.
   */
  double getSingleNodeGain() const {
    double gain = 0.0;
    for (const auto& anchor : gain_anchors) {
      gain += anchor->gain;
    }
    return gain;
  }

  /**
   * @brief Spend this viewpoint: mark it observed, clear is_at_frontier, gain anchors and info.
   *
   * Evaluator::onNodeReached() calls it on arrival; a plugin may also call it for a viewpoint it
   * knows to be observed already. is_terminating is left untouched; the evaluator that set it owns it.
   */
  void markObserved() {
    is_observed = true;
    is_at_frontier = false;
    gain_anchors.clear();
    info.reset();
  }

 public:
  /// @brief The robot state (position and orientation) at this node.
  RobotState robot_state;

  /// @brief Whether each candidate orientation is traversable.
  std::vector<bool> is_candidate_traversable;

  /// @brief Candidate orientations that are collision-free at this node.
  std::vector<Eigen::Quaterniond> candidate_orientations;

  /// @brief Gain anchors associated with this node.
  GainAnchorPtrVector gain_anchors;

  /// @brief PathLedger entries: the most recent path that visited this node, and the most recent one that traversed
  /// the edge from that path's final node to this node.
  std::size_t most_recent_path_visited_this = 0;
  std::size_t most_recent_path_traversed_edge_from_final_node_to_this = 0;

  /// @brief Whether this node lies at a frontier boundary.
  bool is_at_frontier = false;

  /// @brief Whether the path planner's gain suppression zeroed this node's gain for the current search.
  ///
  /// Rewritten before every search (suppressNonMaximumClusters); the gain anchors stay untouched.
  bool is_gain_suppressed = false;

  /// @brief Whether this is an intermediate node (not a decision point).
  bool is_intermediate = false;

  /// @brief Reaching this node ends the episode (goal reached); set by the layer or evaluator that owns it.
  bool is_terminating = false;

  /// @brief Whether this viewpoint has already been observed this episode.
  ///
  /// Set through markObserved() and never cleared; the visit itself is recorded by HistoryData.
  /// Evaluator::evaluate() and Evaluator::updateEvaluation() skip an observed node, so it is never
  /// evaluated again.
  bool is_observed = false;

  /// @brief Whether this node is traversable. Defaults to false: a node never assigned is pruned
  /// on the next maintenance pass.
  bool is_traversable = false;

  /// @brief Which population the generator sampled this node for. Provenance,
  /// not derived state: nothing but the generator assigns it.
  SamplingRole sampling_role = SamplingRole::kDefault;

  /// @brief Optional evaluator-specific information (e.g., visible voxels).
  std::unique_ptr<NodeInfo> info;
};

/// @brief A cluster of node pointers sharing the same spatial location.
using NodeClusterPtrs = std::vector<Node*>;

/**
 * @brief NodeInfo subclass for yaw planning, storing orientation candidates and selection state.
 */
struct YawPlanningInfo : public NodeInfo {
  virtual ~YawPlanningInfo() = default;

  /**
   * @brief Deep copy that preserves the orientation nodes and their dynamic types.
   *
   * Each orientation is cloned with its virtual shallowCopy() plus a deep copy of its info, so
   * the clone shares nothing mutable; owned tree children are not cloned.
   */
  std::unique_ptr<NodeInfo> deepCopy() const override {
    auto copy = std::make_unique<YawPlanningInfo>();
    copy->visited_orientation = visited_orientation;
    copy->active_orientation = active_orientation;

    copy->orientations.reserve(orientations.size());
    for (const auto& orientation : orientations) {
      if (!orientation) {
        copy->orientations.push_back(nullptr);  // a null slot is meaningful: keep the indexing
        continue;
      }
      std::unique_ptr<Node> cloned = orientation->shallowCopy();
      if (orientation->info) {
        cloned->info = orientation->info->deepCopy();
      }
      copy->orientations.push_back(std::move(cloned));
    }
    return copy;
  }

  /// @brief All oriented node versions at this position.
  std::vector<std::unique_ptr<Node>> orientations;

  /// @brief Boolean mask indicating which orientations have been visited.
  std::vector<bool> visited_orientation;

  /// @brief Index of the currently active orientation. -1 = none selected yet;
  /// an indeterminate value here would index `orientations` out of bounds.
  int active_orientation = -1;
};

/**
 * @brief Hash functor for Eigen::Vector3d using boost::hash_combine.
 */
struct Vector3dHash {
  /**
   * @brief Compute a hash for a 3D vector.
   * @param v The vector to hash.
   * @return The computed hash value.
   */
  std::size_t operator()(const Eigen::Vector3d& v) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, v.x());
    boost::hash_combine(seed, v.y());
    boost::hash_combine(seed, v.z());
    return seed;
  }
};

/**
 * @brief Equality comparator for Eigen::Vector3d.
 */
struct Vector3dEqual {
  /**
   * @brief Test two vectors for equality.
   * @param v1 First vector.
   * @param v2 Second vector.
   * @return True if the vectors are exactly equal.
   */
  bool operator()(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const { return v1 == v2; }
};

/// @brief Unordered set of 3D vectors.
using Vector3dSet = std::unordered_set<Eigen::Vector3d, Vector3dHash, Vector3dEqual>;

/// @brief Map from 3D voxel position to its occupancy state (as discovered during raycasting).
using VoxelOccupancyMap = std::unordered_map<Eigen::Vector3d, Occupancy, Vector3dHash, Vector3dEqual>;

/// @brief Combined occupancy and gain information for a single voxel.
struct VoxelInfo {
  Occupancy occupancy = Occupancy::kUnknown;
  double gain = 0.0;
};

/// @brief Map from 3D voxel position to its combined occupancy and gain info.
using VoxelInfoMap = std::unordered_map<Eigen::Vector3d, VoxelInfo, Vector3dHash, Vector3dEqual>;

/**
 * @brief NodeInfo subclass storing simulated sensor results (visible voxels).
 */
struct SimulatedSensorInfo : public NodeInfo {
  virtual ~SimulatedSensorInfo() {}

  /**
   * @brief Create a deep copy of this simulated sensor info.
   * @return A unique pointer to the cloned info.
   */
  std::unique_ptr<NodeInfo> deepCopy() const override { return std::make_unique<SimulatedSensorInfo>(*this); }

  /// @brief Map from visible voxel positions to their occupancy and gain info.
  VoxelInfoMap visible_voxels;
};

}  // namespace open_apero
