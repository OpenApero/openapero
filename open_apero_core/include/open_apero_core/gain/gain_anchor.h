#pragma once

/**
 * @file gain_anchor.h
 * @brief Polymorphic gain anchor base class and associated type aliases.
 *
 * Gain anchors represent sources of information gain associated with nodes
 * in the exploration planner.
 */

#include <cstddef>
#include <memory>
#include <vector>

namespace open_apero {

/**
 * @brief Base class for gain anchors that associate an information gain value with a spatial entity.
 *
 * Derived classes can extend this to carry additional domain-specific data.
 * Each anchor has a unique ID derived from its memory address by default.
 */
struct GainAnchor {
  /// @brief The information gain value.
  double gain;

  /// @brief PathLedger entry: the most recent path that collected this anchor's gain.
  std::size_t most_recent_path_collected_this = 0;

  /**
   * @brief Construct a GainAnchor with the given gain value.
   * @param gain The information gain value.
   */
  explicit GainAnchor(double gain) : gain(gain) {}
  virtual ~GainAnchor() = default;

  /**
   * @brief Get a unique identifier for this anchor.
   *
   * PathLedger does not use it: a path collects each anchor object once, so two objects sharing an id count twice.
   * @return A unique ID based on the object's memory address.
   */
  virtual size_t getUniqueId() const { return reinterpret_cast<size_t>(this); }

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
 * @brief Comparator for ordering shared pointers to GainAnchor by unique ID.
 */
struct GainAnchorPtrComparator {
  /**
   * @brief Compare two GainAnchor shared pointers by their unique IDs.
   * @param a First anchor pointer.
   * @param b Second anchor pointer.
   * @return True if a's unique ID is less than b's.
   */
  bool operator()(const std::shared_ptr<GainAnchor>& a, const std::shared_ptr<GainAnchor>& b) const {
    return a->getUniqueId() < b->getUniqueId();
  }
};

using GainAnchorPtr = std::shared_ptr<GainAnchor>;
using GainAnchorPtrVector = std::vector<GainAnchorPtr>;

}  // namespace open_apero
