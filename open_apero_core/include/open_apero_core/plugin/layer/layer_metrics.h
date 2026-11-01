#pragma once

/**
 * @file layer_metrics.h
 * @brief Per-layer metrics shared by every Layer.
 *
 * Holds the four timings/counters that are meaningful for *any* layer:
 * per-cycle time spent in expand / update / planPath, and the cumulative
 * number of times the layer won the agent's priority-dispatch. Subclass
 * to add layer-specific fields (see PlanningLayerMetrics).
 */

#include "open_apero_core/logging/metrics.h"

namespace open_apero {

struct LayerMetrics : Metrics {
  // Per-cycle (reset by resetCycle).
  double expand_s = 0.0;
  double update_s = 0.0;
  double plan_path_s = 0.0;

  // Cumulative (reset only by resetEpisode).
  int times_selected_as_winner = 0;

  void resetCycle() override {
    expand_s = 0.0;
    update_s = 0.0;
    plan_path_s = 0.0;
  }

  void resetEpisode() override {
    resetCycle();
    times_selected_as_winner = 0;
  }

  void declareFields(MetricsFields& fields) const override {
    // ExpandTime / UpdateTime are per-layer root timers. PlanPathTime is summed
    // by the agent into the global PlanPathTotal, so it nests under that.
    fields.add("ExpandTime", &expand_s, "s");
    fields.add("UpdateTime", &update_s, "s");
    fields.add("PlanPathTime", &plan_path_s, "s", ParentRef::global("PlanPathTotal"));
    fields.add("TimesSelected", &times_selected_as_winner, "count");
  }
};

}  // namespace open_apero
