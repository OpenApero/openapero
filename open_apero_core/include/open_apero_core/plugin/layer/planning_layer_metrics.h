#pragma once

/**
 * @file planning_layer_metrics.h
 * @brief Per-cycle and per-call metrics specific to PlanningLayer's graph-based
 *        planning loop. Extends LayerMetrics with sub-phase timings (node/edge
 *        generation, evaluation, generator/evaluator updates) and graph-size
 *        snapshots taken at expansion / new-start / selection points.
 */

#include "open_apero_core/plugin/layer/layer_metrics.h"

namespace open_apero {
namespace layer {

struct PlanningLayerMetrics : LayerMetrics {
  // --- Per-cycle: PlanningLayer sub-phase timings (seconds) ---
  double generate_node_s = 0.0;      ///< Time in GraphGenerator::generateNewNode() across all expansion attempts.
  double generate_edge_s = 0.0;      ///< Time in GraphGenerator::generateNewEdges().
  double evaluate_node_s = 0.0;      ///< Time evaluating the gain of each newly generated node.
  double generator_update_s = 0.0;   ///< Time in GraphGenerator::updateNodeEdge() inside update() (strictly ⊂ UpdateTime).
  double evaluator_update_s = 0.0;   ///< Time in Evaluator::updateEvaluation() to refresh gain estimates.
  double start_node_update_s = 0.0;  ///< Time updating the start node: GraphGenerator::setNewStartNode() and
                                     ///< insertIntermediateNodeAtCurrentState() in onStateReached() (outside update()).

  // --- Per-cycle: iteration stats ---
  double planned_value = 0.0;          ///< Value of the graph candidate selected before final trajectory validation. May be
                                       ///< nonzero when the layer declines it; TimesSelected records accepted layer wins.
  double real_accumulated_gain = 0.0;  ///< The evaluator's accumulated gain, read in onStateReached() at the start of the cycle.
  int num_expansions = 0;              ///< Number of expansion rounds run inside expand() this cycle.
  int num_nodes_after_expansion = 0;   ///< Graph size after this cycle's start-node update, update and expansion.
  int num_nodes_after_new_start = 0;   ///< Graph size after the setNewStartNode() prune pass.
  int num_nodes_at_selection = 0;      ///< Graph size at the time of next-best-view selection.

  void resetCycle() override {
    LayerMetrics::resetCycle();
    generate_node_s = 0.0;
    generate_edge_s = 0.0;
    evaluate_node_s = 0.0;
    generator_update_s = 0.0;
    evaluator_update_s = 0.0;
    start_node_update_s = 0.0;

    planned_value = 0.0;
    real_accumulated_gain = 0.0;

    num_expansions = 0;
    num_nodes_after_expansion = 0;
    num_nodes_after_new_start = 0;
    num_nodes_at_selection = 0;
  }

  void declareFields(MetricsFields& fields) const override {
    LayerMetrics::declareFields(fields);
    // Sub-phase timers nest under the layer root timer they run inside (parents local to this prefix).
    fields.add("GenerateNode", &generate_node_s, "s", ParentRef::local("ExpandTime"));
    fields.add("GenerateEdge", &generate_edge_s, "s", ParentRef::local("ExpandTime"));
    fields.add("EvaluateNode", &evaluate_node_s, "s", ParentRef::local("ExpandTime"));
    fields.add("GeneratorUpdate", &generator_update_s, "s", ParentRef::local("UpdateTime"));
    fields.add("EvaluatorUpdate", &evaluator_update_s, "s", ParentRef::local("UpdateTime"));
    // The start-node update runs in onStateReached, outside UpdateTime: a root timer of its own.
    fields.add("StartNodeUpdate", &start_node_update_s, "s");
    fields.add("PlannedValue", &planned_value);
    fields.add("RealAccumulatedGain", &real_accumulated_gain);
    fields.add("NumExpansions", &num_expansions, "count");
    fields.add("NumNodesAfterExpansion", &num_nodes_after_expansion, "count");
    fields.add("NumNodesAfterNewStart", &num_nodes_after_new_start, "count");
    fields.add("NumNodesAtSelection", &num_nodes_at_selection, "count");
  }
};

}  // namespace layer
}  // namespace open_apero
