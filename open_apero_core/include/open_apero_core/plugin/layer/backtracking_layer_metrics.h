#pragma once

/**
 * @file backtracking_layer_metrics.h
 * @brief Per-cycle metrics of BacktrackingLayer: the base four plus its backtracker's own columns.
 */

#include "open_apero_core/logging/metrics.h"
#include "open_apero_core/plugin/backtracker/backtracker.h"
#include "open_apero_core/plugin/layer/layer_metrics.h"

namespace open_apero {
namespace layer {

struct BacktrackingLayerMetrics : LayerMetrics {
  Backtracker* backtracker = nullptr;  ///< The strategy whose columns ride along; set once it exists.

  // LayerMetrics::resetEpisode calls this through the virtual, so the backtracker's columns clear there too.
  void resetCycle() override {
    LayerMetrics::resetCycle();
    if (backtracker) {
      backtracker->resetCycleMetrics();
    }
  }

  void declareFields(MetricsFields& fields) const override {
    LayerMetrics::declareFields(fields);
    if (backtracker) {
      backtracker->declareMetrics(fields);
    }
  }
};

}  // namespace layer
}  // namespace open_apero
