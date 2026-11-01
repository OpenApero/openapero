#pragma once

/**
 * @file metrics.h
 * @brief Interface for metric groups written to the performance CSV.
 *
 * Shared machinery only; each concrete group lives beside the class it measures
 * (plugin/layer/layer_metrics.h, plugin/layer/planning_layer_metrics.h, agent/agent_metrics.h).
 */

#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace open_apero {

/**
 * @brief Reference to the timer whose wall time wholly contains a field's, for nesting.
 *
 * kLocal resolves to "<prefix>.<name>" within the same metric group; kGlobal to the
 * unprefixed agent-level "<name>".
 */
struct ParentRef {
  enum class Scope { kNone, kLocal, kGlobal };

  Scope scope = Scope::kNone;
  std::string name;  ///< Unqualified parent column (leaf) name.

  static ParentRef none() { return {}; }
  static ParentRef local(std::string name) { return {Scope::kLocal, std::move(name)}; }
  static ParentRef global(std::string name) { return {Scope::kGlobal, std::move(name)}; }
};

/**
 * @brief Ordered collection of named metric fields declared by a Metrics group.
 *
 * Each field pairs a column name with a pointer to the live value, plus a unit and an optional
 * parent for the run manifest. Registered pointers must outlive the collection.
 */
class MetricsFields {
 public:
  using Value = std::variant<const double*, const int*>;

  struct Field {
    std::string name;
    Value value;
    std::string unit;  ///< Unit string (e.g. "s", "m", "count"); "" if none/dynamic.
    ParentRef parent;  ///< Enclosing timer, or ParentRef::none().
  };

  void add(std::string name, const double* value, std::string unit = "", ParentRef parent = {}) {
    fields_.push_back({std::move(name), value, std::move(unit), std::move(parent)});
  }
  void add(std::string name, const int* value, std::string unit = "", ParentRef parent = {}) {
    fields_.push_back({std::move(name), value, std::move(unit), std::move(parent)});
  }

  const std::vector<Field>& fields() const { return fields_; }

 private:
  std::vector<Field> fields_;
};

/**
 * @brief Common reset and field-declaration interface for metric groups.
 *
 * Subclasses declare each field once in declareFields(), chaining the base first so base
 * columns precede derived ones; PerformanceLog handles serialization.
 */
struct Metrics {
  /// @brief Clears the per-cycle fields; the agent calls it once a cycle has selected its step, after the cycle's CSV row (if any).
  virtual void resetCycle() = 0;
  /// @brief Clears the per-episode totals; called when the owning agent or layer resets for a new episode.
  virtual void resetEpisode() = 0;

  /**
   * @brief Declares this group's fields as {column name, field pointer} pairs.
   * @param fields Collector the fields are appended to in column order.
   */
  virtual void declareFields(MetricsFields& fields) const = 0;

  virtual ~Metrics() = default;

  /**
   * @brief Downcast to a derived metrics type.
   * @tparam T The target derived type.
   * @return Pointer to the derived type, or nullptr if the cast fails.
   */
  template <class T>
  T* as() {
    return dynamic_cast<T*>(this);
  }
};

}  // namespace open_apero
