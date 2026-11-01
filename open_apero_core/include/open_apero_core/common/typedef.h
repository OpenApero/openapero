#pragma once

/**
 * @file typedef.h
 * @brief Common type aliases, enumerations, and constants for the exploration planner.
 *
 * Centralizes shared types such as occupancy states, traversability levels,
 * collision status, sensor mount types, and numerical constants.
 */

#include <limits>
#include <utility>
#include <vector>

namespace open_apero {

/// @brief Voxel occupancy state.
enum class Occupancy { kFree, kOccupied, kUnknown };

/// @brief Node or edge traversability classification.
///
/// kUnset: no determination yet, the caller must compute it; raw map checks never return it.
/// kPotentiallyTraversable: an ambiguous map verdict, resolved by safe/clearing-radius optimism.
enum class Traversability { kTraversable, kPotentiallyTraversable, kUntraversable, kUnset };

/// @brief Result of a collision check.
enum class CollisionStatus { kColliding, kNotColliding, kUnknown };

/// @brief Sensor mounting type relative to the robot base.
enum class SensorMountType { kFixed, kFreePitch, kUnconstrained };

/// @brief Maximum representable double value.
const double kDoubleMax = std::numeric_limits<double>::max();

/// @brief Minimum (most negative) representable double value.
const double kDoubleMin = std::numeric_limits<double>::lowest();

/// @brief Quiet NaN double value used as a sentinel.
const double kDoubleNan = std::numeric_limits<double>::quiet_NaN();

/// @brief A list of (u, v) index pairs defining edges between nodes — the standard
///        graph-theoretic edge list representation.
using EdgeList = std::vector<std::pair<int, int>>;

/// @brief Small epsilon for floating-point comparisons.
const double kEpsilon = 1e-6;

/// @brief Squared epsilon for squared-distance comparisons.
const double kEpsilonSquared = kEpsilon * kEpsilon;

}  // namespace open_apero
