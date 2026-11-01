#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <jsk_rviz_plugins/OverlayText.h>
#include <open_apero_core/agent/benchmark_agent.h>
#include <open_apero_core/common/typedef.h>
#include <open_apero_core/graph/graph_node.h>
#include <open_apero_core/plugin/path_planner/graph_depth_wise_beam_search.h>
#include <open_apero_core/plugin/path_planner/graph_node_wise_beam_search.h>
#include <open_apero_core/plugin/path_planner/graph_shortest_path_tree_search.h>
#include <open_apero_core/plugin/path_planner/graph_tsp_search.h>
#include <open_apero_core/plugin/plugin_factory_config.h>
#include <open_apero_core/state/history_data.h>
#include <open_apero_core/visualization/visualization_helpers.h>
#include <open_apero_ros_bridge/log_level.h>
#include <open_apero_ros_bridge/marker_conversion.h>
#include <open_apero_ros_bridge/status_overlay.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "open_apero_graph_benchmark/benchmark_status.h"
#include "open_apero_graph_benchmark/benchmark_sweep.h"
#include "open_apero_graph_benchmark/graph_manager.h"

using namespace open_apero;

// Not open_apero_core's kEpsilon (1e-6, meant for comparisons): kGainEpsilon classifies a gain as zero in the
// diagnostics, kSweepStepTolerance nudges floor() when counting sweep steps. At 1e-6 both change the output
// (a small gain reported as zero; an extra SPT/TSP step past a maximum just under a step multiple).
constexpr double kGainEpsilon = 1e-12;
constexpr double kSweepStepTolerance = 1e-9;

namespace {
bool visualize = false;
bool visualize_edges = false;
bool debug_diagnostics = false;
double visualization_rate = 5.0;
double perception_radius = std::numeric_limits<double>::max();
::ros::Publisher visualization_pub;
::ros::Publisher status_overlay_pub;
}  // namespace

struct RunContext {
  std::string graph_instance;
  std::string planner;
  std::string objective;
  std::string replan_type;
};

struct GraphStats {
  std::size_t visible_nodes = 0;
  std::size_t frontier_nodes = 0;
  std::size_t positive_gain_nodes = 0;
};

RunContext makeRunContext(const std::string& result_filepath) {
  const std::filesystem::path path(result_filepath);
  RunContext context;
  context.planner = path.stem().string();
  context.objective = path.parent_path().filename().string();
  context.replan_type = path.parent_path().parent_path().filename().string();
  context.graph_instance = path.parent_path().parent_path().parent_path().filename().string();
  return context;
}

// Dataset folders encode the perception model they were produced under: `ap_` means a-priori (the whole graph is
// visible), `op_<R>m_` means online perception with radius R. Running a dataset under the other model silently
// produces results that do not match its name, so say so -- as a warning, since custom dataset names are allowed.
void warnOnPerceptionRadiusMismatch(const std::string& graph_type_folder, double radius) {
  const std::string dataset = std::filesystem::path(graph_type_folder).filename().string();
  const bool has_finite_radius = std::isfinite(radius) && radius < std::numeric_limits<double>::max();

  std::smatch match;
  if (dataset.rfind("ap_", 0) == 0) {
    // A-priori means the whole graph is visible from wherever the robot stands, so the radius only has to reach
    // the far corner -- the `<W>x<H>` extent in the name bounds the largest node-to-node distance by its diagonal.
    // Any radius at least that large is silent.
    if (!std::regex_search(dataset, match, std::regex(R"((\d+)x(\d+))"))) {
      return;  // Custom name with no extent to check the radius against.
    }
    const double required_radius = std::hypot(std::stod(match[1].str()), std::stod(match[2].str()));
    if (!has_finite_radius || radius >= required_radius) {
      return;
    }
    ROS_WARN_STREAM("Dataset '" << dataset << "' is named a-priori (ap_) but perception_radius=" << radius << " does not span its "
                                << match[1].str() << 'x' << match[2].str() << " extent (needs >= " << required_radius
                                << "), so part of the graph stays hidden.");
    return;
  }

  if (std::regex_search(dataset, match, std::regex(R"(^op_(\d+(?:\.\d+)?)m_)"))) {
    const double expected_radius = std::stod(match[1].str());
    if (!has_finite_radius || std::abs(expected_radius - radius) > kEpsilon) {
      ROS_WARN_STREAM("Dataset '" << dataset << "' is named for an online-perception radius of " << expected_radius
                                  << " but perception_radius=" << radius << ".");
    }
  }
}

GraphStats getGraphStats(GraphType* graph) {
  GraphStats stats;
  stats.visible_nodes = boost::num_vertices(*graph);
  auto vertices = boost::vertices(*graph);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    const Node* node = (*graph)[*it].get();
    if (node->is_at_frontier) {
      ++stats.frontier_nodes;
    }
    if (node->getSingleNodeGain() > kGainEpsilon) {
      ++stats.positive_gain_nodes;
    }
  }
  return stats;
}

void publishStatusOverlay(const RunContext& context, const std::string& phase, GraphManager* graph_manager, Node* current_node,
                          std::size_t step, double step_gain, double total_gain, double total_cost, double cost_budget) {
  if (!visualize) {
    return;
  }

  BenchmarkStatus status;
  status.graph_instance = context.graph_instance;
  status.planner = context.planner;
  status.objective = context.objective;
  status.replan_type = context.replan_type;
  status.phase = phase;
  status.step = step;
  status.node_id = graph_manager->getOriginalId(current_node);
  status.step_gain = step_gain;
  status.total_gain = total_gain;
  status.total_cost = total_cost;
  status.cost_budget = cost_budget;

  std::vector<std::string> lines;
  std::istringstream text(formatBenchmarkStatus(status));
  for (std::string line; std::getline(text, line);) {
    lines.push_back(line);
  }
  status_overlay_pub.publish(open_apero::ros::buildStatusOverlay(lines));
}

void publishVisualization(const VisualizationMarkers& markers) {
  if (markers.getMarkers().empty()) {
    return;
  }

  visualization_msgs::MarkerArray msg;

  VisualizationMarker delete_marker;
  delete_marker.id = 0;
  delete_marker.ns = "deleteall";
  delete_marker.action = VisualizationMarker::DELETEALL;

  visualization_msgs::Marker delete_msg;
  open_apero::ros::visualizationMarkerToMsg(delete_marker, &delete_msg);
  msg.markers.push_back(delete_msg);

  open_apero::ros::visualizationMarkersToMsg(markers, &msg);

  auto now = ::ros::Time::now();
  for (visualization_msgs::Marker& m : msg.markers) {
    m.header.frame_id = "world";
    m.header.stamp = now;
  }

  visualization_pub.publish(msg);

  ::ros::Duration(1.0 / visualization_rate).sleep();
}

void publishBenchmarkFrame(const RunContext& context, const std::string& phase, GraphManager* graph_manager, Node* current_node,
                           const VisualizationMarkers& path_markers, HistoryData* history_data, double total_cost_budget, std::size_t step,
                           double step_gain, double total_gain, double total_cost) {
  if (!visualize) {
    return;
  }

  VisualizationMarkers graph_markers;
  graph_manager->getVisualizationMarkers(&graph_markers, visualize_edges);

  VisualizationMarkers history_markers;
  history_data->getVisualization(&history_markers, total_cost_budget);

  VisualizationMarkers markers;
  markers.addMarkers(path_markers.getMarkers());
  markers.addMarkers(graph_markers.getMarkers());
  markers.addMarkers(history_markers.getMarkers());
  publishVisualization(markers);
  publishStatusOverlay(context, phase, graph_manager, current_node, step, step_gain, total_gain, total_cost, total_cost_budget);
}

void warnIfNoPositiveGain(const RunContext& context, GraphType* graph, bool* warning_emitted) {
  if (!debug_diagnostics || *warning_emitted) {
    return;
  }
  const GraphStats stats = getGraphStats(graph);
  if (stats.positive_gain_nodes == 0) {
    ROS_WARN_STREAM("[benchmark_gain] graph=" << context.graph_instance << " planner=" << context.planner << " visible_nodes="
                                              << stats.visible_nodes << " positive_gain_nodes=0; gain-based choices may be tied");
    *warning_emitted = true;
  }
}

void logExecutedStep(const RunContext& context, GraphType* graph, std::size_t step, int from_node, int to_node, double step_gain,
                     double total_gain, double edge_cost, double total_cost, double cost_budget, std::size_t planned_path_length,
                     double planned_value, int zero_gain_streak, bool repeated_transition) {
  if (!debug_diagnostics) {
    return;
  }
  const GraphStats stats = getGraphStats(graph);
  ROS_INFO_STREAM("[benchmark_step] graph=" << context.graph_instance << " planner=" << context.planner
                                            << " objective=" << context.objective << " replan=" << context.replan_type << " step=" << step
                                            << " from=" << from_node << " to=" << to_node << " step_gain=" << step_gain
                                            << " total_gain=" << total_gain << " edge_cost=" << edge_cost << " total_cost=" << total_cost
                                            << '/' << cost_budget << " visible_nodes=" << stats.visible_nodes
                                            << " frontier_nodes=" << stats.frontier_nodes
                                            << " positive_gain_nodes=" << stats.positive_gain_nodes << " plan_nodes=" << planned_path_length
                                            << " planned_value=" << planned_value << " zero_gain_streak=" << zero_gain_streak
                                            << " repeated_transition=" << (repeated_transition ? "true" : "false"));
}

// What one run reports back to its block: why it stopped and which result file it claimed.
struct RunResult {
  std::string result_filepath;
  // False only when the result file could not be opened. The block counts such a run as skipped, so the
  // totals cannot overstate a sweep that produced no results for part of it.
  bool wrote_file = false;
  // Whether the file was already there before this run truncated it. Audit label only.
  bool existed_before = false;
  // Short, tallyable label for reason_text: noplan, nodegone, noedge, overbudget, oneplan or exhausted. Two of
  // the six (nodegone, noedge) embed node IDs in the text, so the block histogram keys on this label instead.
  const char* reason_kind = "";
  std::string reason_text;
};

RunResult runIteration(PathPlanner* path_planner, BenchmarkAgent* agent, GraphManager* graph_manager, double cost_budget,
                       const std::string& result_filepath, const std::string& replan_type) {
  const RunContext context = makeRunContext(result_filepath);
  agent->setCostBudget(cost_budget);

  RunResult result;
  result.result_filepath = result_filepath;
  // Sampled before the open, which creates the file: afterwards every run looks like an overwrite. A failed
  // check must not cost a result, so an error here leaves the label unreported rather than refusing the write.
  std::error_code exists_error;
  result.existed_before = std::filesystem::exists(result_filepath, exists_error) && !exists_error;

  std::ofstream result_file(result_filepath, std::ios::trunc);
  if (!result_file.is_open()) {
    ROS_ERROR_STREAM("Skipping run: cannot open result file " << result_filepath);
    return result;
  }

  std::vector<Node*> path;
  double value = std::numeric_limits<double>::quiet_NaN();
  double gain = 0.0;
  double cost = 0.0;
  double computation_time = 0.0;

  graph_manager->reset();
  GraphType* graph = graph_manager->generateGraphFromData(0, perception_radius);
  Node* current_node = graph_manager->getCurrentNode();

  auto write_result = [&]() {
    std::stringstream ss;
    ss << gain << ' ' << cost << ' ' << computation_time << ' ' << graph_manager->getOriginalId(current_node) << ' '
       << boost::num_vertices(*graph) << "\n";
    result_file << ss.str();
    result_file.flush();
  };
  write_result();

  HistoryData history_data;
  VisualizationMarkers latest_path_markers;
  std::map<std::pair<int, int>, int> transition_counts;
  bool no_positive_gain_warning_emitted = false;
  bool repeated_transition = false;
  int zero_gain_streak = 0;
  std::size_t step = 0;
  double last_step_gain = 0.0;
  std::string termination_reason;
  // Records the tallyable kind alongside the human wording at every exit, so the two cannot drift apart. The
  // block-line histogram counts kinds while the per-run debug line and summary retain the full text.
  const auto stop = [&result, &termination_reason](const char* kind, std::string text) {
    result.reason_kind = kind;
    termination_reason = std::move(text);
  };

  warnIfNoPositiveGain(context, graph, &no_positive_gain_warning_emitted);
  publishBenchmarkFrame(context, "initializing", graph_manager, current_node, latest_path_markers, &history_data,
                        agent->getOriginalCostBudget(), step, last_step_gain, gain, cost);

  while (cost < cost_budget) {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::tie(path, value) = path_planner->planPath(current_node, graph);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    computation_time += duration.count() / 1e6;

    if (path.size() < 2) {
      // Not a fault: every planner prunes expansions against the remaining cost budget, so once no edge fits
      // it returns a single-node path. This is the normal terminator under everystep and goal.
      stop("noplan", "no path with at least one edge");
      break;
    }

    // Hold planned waypoints as original IDs, not raw pointers: execute_step() regenerates the graph, which may
    // prune vertices and free the nodes behind them, and `path` outlives those calls in none/goal mode.
    std::vector<int> planned_node_ids;
    planned_node_ids.reserve(path.size());
    for (Node* node : path) {
      planned_node_ids.push_back(graph_manager->getOriginalId(node));
    }

    latest_path_markers = VisualizationMarkers();
    if (visualize) {
      std::vector<Edge*> path_edges;
      for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        if (Edge* edge = getEdge(path[i], path[i + 1], graph)) {
          path_edges.push_back(edge);
        }
      }
      getPlannedPathVisualizationMarkers(path_edges, &latest_path_markers);
    }

    auto execute_step = [&](int to_node_id) {
      const int from_node_id = graph_manager->getOriginalId(current_node);

      Node* next_node = graph_manager->getNodeByOriginalId(to_node_id);
      if (next_node == nullptr) {
        stop("nodegone", "planned node " + std::to_string(to_node_id) + " is no longer in the graph");
        return false;
      }

      Edge* edge = getEdge(current_node, next_node, graph);
      if (edge == nullptr) {
        stop("noedge", "no edge from node " + std::to_string(from_node_id) + " to node " + std::to_string(to_node_id));
        return false;
      }

      const double step_gain = next_node->getSingleNodeGain();
      const double new_cost = edge->cost;

      // Tested before committing, so `cost` only ever counts traversed edges.
      if (cost + new_cost > cost_budget) {
        stop("overbudget", "next edge exceeds cost budget");
        return false;
      }
      gain += step_gain;
      cost += new_cost;

      current_node = next_node;
      current_node->gain_anchors.clear();
      agent->reduceCostBudget(new_cost);
      history_data.addVisitedEdge(*edge);
      graph = graph_manager->generateGraphFromData(to_node_id, perception_radius);

      ++step;
      last_step_gain = step_gain;
      zero_gain_streak = step_gain <= kGainEpsilon ? zero_gain_streak + 1 : 0;
      const int transition_count = ++transition_counts[{from_node_id, to_node_id}];
      repeated_transition = transition_count > 1;

      if (debug_diagnostics && transition_count == 2) {
        ROS_WARN_STREAM("[benchmark_cycle] graph=" << context.graph_instance << " planner=" << context.planner
                                                   << " repeated_transition=" << from_node_id << "->" << to_node_id << " step=" << step
                                                   << " total_gain=" << gain << " total_cost=" << cost);
      }
      warnIfNoPositiveGain(context, graph, &no_positive_gain_warning_emitted);
      logExecutedStep(context, graph, step, from_node_id, to_node_id, step_gain, gain, new_cost, cost, cost_budget, path.size(), value,
                      zero_gain_streak, repeated_transition);
      publishBenchmarkFrame(context, "running", graph_manager, current_node, latest_path_markers, &history_data,
                            agent->getOriginalCostBudget(), step, step_gain, gain, cost);
      write_result();
      return true;
    };

    if (replan_type == "everystep") {
      write_result();
      if (!execute_step(planned_node_ids.at(1))) {
        break;
      }
    } else if (replan_type == "none" || replan_type == "goal") {
      for (std::size_t i = 1; i < planned_node_ids.size(); ++i) {
        write_result();
        if (!execute_step(planned_node_ids.at(i))) {
          break;
        }
      }

      if (replan_type == "none" || !termination_reason.empty()) {
        if (termination_reason.empty()) {
          stop("oneplan", "single plan completed");
        }
        break;
      }
    } else {
      // Unreachable: main() validates replan_type before any run starts. Kept as a guard.
      throw std::runtime_error("Error: Unknown replan type: " + replan_type);
    }
  }

  // Every break above sets a reason, so an empty one means the loop condition failed: cost >= cost_budget.
  if (termination_reason.empty()) {
    stop("exhausted", "cost budget exhausted");
  }
  publishBenchmarkFrame(context, "finished", graph_manager, current_node, latest_path_markers, &history_data,
                        agent->getOriginalCostBudget(), step, last_step_gain, gain, cost);
  if (debug_diagnostics) {
    const GraphStats stats = getGraphStats(graph);
    ROS_INFO_STREAM("[benchmark_summary] graph=" << context.graph_instance << " planner=" << context.planner
                                                 << " objective=" << context.objective << " replan=" << context.replan_type << " reason=\""
                                                 << termination_reason << "\" steps=" << step << " total_gain=" << gain
                                                 << " total_cost=" << cost << '/' << cost_budget << " final_node="
                                                 << graph_manager->getOriginalId(current_node) << " visible_nodes=" << stats.visible_nodes
                                                 << " positive_gain_nodes=" << stats.positive_gain_nodes << " result=" << result_filepath);
  }
  result.wrote_file = true;
  result.reason_text = termination_reason;
  return result;
}

// Result filenames are a load-bearing part of the output format: python/open_apero_graph_benchmark/benchmark_ranges.py groups results by
// parsing them, so the two spellings below are fixed. Beam widths are integers (`dbs_1.txt`); percentages
// always carry two decimals (`spt_0.50.txt`), including the trailing zero. Centralized in these two overloads
// so the four planner wrappers cannot drift apart on it.
std::string makeResultPath(const std::string& path_folder, const std::string& algorithm, int beam_width) {
  std::ostringstream oss;
  oss << path_folder << '/' << algorithm << '_' << beam_width << ".txt";
  return oss.str();
}

std::string makeResultPath(const std::string& path_folder, const std::string& algorithm, double top_node_percentage) {
  std::ostringstream oss;
  oss << path_folder << '/' << algorithm << '_' << std::fixed << std::setprecision(2) << top_node_percentage << ".txt";
  return oss.str();
}

RunResult runDbsIteration(Objective obj, int beam_width, int beam_depth, BenchmarkAgent* agent, GraphManager* graph_manager,
                          double cost_budget, const std::string& path_folder, const std::string& replan_type) {
  auto searcher = std::make_unique<path_planner::GraphDepthWiseBeamSearch>(*agent);
  searcher->setupRaw(obj, beam_depth, beam_width, false);
  return runIteration(searcher.get(), agent, graph_manager, cost_budget, makeResultPath(path_folder, "dbs", beam_width), replan_type);
}

RunResult runNbsIteration(Objective obj, int beam_width, int beam_depth, BenchmarkAgent* agent, GraphManager* graph_manager,
                          double cost_budget, const std::string& path_folder, const std::string& replan_type) {
  auto searcher = std::make_unique<path_planner::GraphNodeWiseBeamSearch>(*agent);
  searcher->setupRaw(obj, beam_depth, beam_width, false);
  return runIteration(searcher.get(), agent, graph_manager, cost_budget, makeResultPath(path_folder, "nbs", beam_width), replan_type);
}

RunResult runSptIteration(Objective obj, double top_node_percentage, BenchmarkAgent* agent, GraphManager* graph_manager, double cost_budget,
                          const std::string& path_folder, const std::string& replan_type) {
  auto searcher = std::make_unique<path_planner::GraphShortestPathTreeSearch>(*agent);
  searcher->setupRaw(obj, top_node_percentage);
  return runIteration(searcher.get(), agent, graph_manager, cost_budget, makeResultPath(path_folder, "spt", top_node_percentage),
                      replan_type);
}

RunResult runTspIteration(Objective obj, double top_node_percentage, BenchmarkAgent* agent, GraphManager* graph_manager, double cost_budget,
                          const std::string& path_folder, const std::string& replan_type) {
  auto searcher = std::make_unique<path_planner::GraphTspSearch>(*agent);
  searcher->setupRaw(obj, top_node_percentage);
  return runIteration(searcher.get(), agent, graph_manager, cost_budget, makeResultPath(path_folder, "tsp", top_node_percentage),
                      replan_type);
}

// Warns about result files in @p result_folder that this benchmark's own sweeps could have written but this
// block did not. Those are leftovers from an earlier sweep with different parameters, and nothing downstream
// can tell them apart: load_and_group_files() in python/open_apero_graph_benchmark/benchmark_ranges.py groups by filename alone, so they
// are plotted next to the fresh results. Silent when the folder holds exactly what the sweep claimed, which is
// the case for every shipped dataset under the launch defaults.
void reportStaleResultFiles(const std::string& result_folder, const std::vector<std::string>& claimed_filenames,
                            const std::string& block_prefix) {
  std::error_code scan_error;
  std::vector<std::string> stale;
  for (const auto& entry : std::filesystem::directory_iterator(result_folder, scan_error)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const std::string filename = entry.path().filename().string();
    if (!isBenchmarkResultFilename(filename)) {
      continue;
    }
    if (std::find(claimed_filenames.begin(), claimed_filenames.end(), filename) == claimed_filenames.end()) {
      stale.push_back(filename);
    }
  }
  if (scan_error || stale.empty()) {
    // A scan failure needs no message of its own: the block has just written its results through this same
    // directory, so anything actually wrong with it has already been reported per run.
    return;
  }
  // directory_iterator order is unspecified, so sort to keep the message a function of the folder's contents.
  std::sort(stale.begin(), stale.end());

  // Cap the list: a folder holding a hundred leftovers should say so, not print all hundred names.
  constexpr std::size_t kMaxNamesListed = 8;
  std::ostringstream names;
  for (std::size_t i = 0; i < stale.size() && i < kMaxNamesListed; ++i) {
    names << (i == 0 ? "" : ",") << stale[i];
  }
  if (stale.size() > kMaxNamesListed) {
    names << ",(+" << (stale.size() - kMaxNamesListed) << " more)";
  }
  ROS_WARN_STREAM("[benchmark_stale] " << block_prefix << stale.size() << " files match this benchmark's result"
                                       << " naming but were not written by this sweep: " << names.str()
                                       << "; python/open_apero_graph_benchmark/benchmark_ranges.py will plot them alongside"
                                       << " the fresh results."
                                       << " Folder: " << result_folder);
}

int main(int argc, char** argv) {
  // rosconsole's stdout is fully buffered under a pipe (roslaunch, a redirect), so line-buffer it for the
  // progress lines to appear as they are written. Must precede any output on the stream, hence before ros::init.
  setvbuf(stdout, nullptr, _IOLBF, 0);

  ::ros::init(argc, argv, "benchmark_node");

  ::ros::NodeHandle nh("~");

  // Route open_apero_core's logging::* through rosconsole and apply ~log_level to both filters, so one knob
  // covers the node and the library it links. Setting the severity alone would not be enough: without a sink
  // the core writes straight to stdout with its own timestamp, never reaching /rosout.
  open_apero::installRosconsoleSink();
  open_apero::setLogLevelFromParam(nh);

  visualization_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
  status_overlay_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("status_overlay", 1, true);

  nh.param("visualize", visualize, false);
  nh.param("visualize_edges", visualize_edges, false);
  nh.param("debug_diagnostics", debug_diagnostics, false);
  nh.param("visualization_rate", visualization_rate, 5.0);
  nh.param("perception_radius", perception_radius, std::numeric_limits<double>::max());

  int beam_depth;
  nh.param("beam_depth", beam_depth, 40);

  double cost_budget;
  nh.param("cost_budget", cost_budget, 100.0);

  double top_node_percentage_min;
  double top_node_percentage_max;
  double percentage_step;
  nh.param("top_node_percentage_min", top_node_percentage_min, 0.0);
  nh.param("top_node_percentage_max", top_node_percentage_max, 1.0);
  nh.param("percentage_step", percentage_step, 0.1);

  int min_beam_width_dbs;
  int max_beam_width_dbs;
  int beam_width_step_dbs;
  nh.param("min_beam_width_dbs", min_beam_width_dbs, 1);
  nh.param("max_beam_width_dbs", max_beam_width_dbs, 50000000);
  nh.param("beam_width_step_dbs", beam_width_step_dbs, 2);

  int min_beam_width_nbs;
  int max_beam_width_nbs;
  int beam_width_step_nbs;
  nh.param("min_beam_width_nbs", min_beam_width_nbs, 1);
  nh.param("max_beam_width_nbs", max_beam_width_nbs, 2000000);
  nh.param("beam_width_step_nbs", beam_width_step_nbs, 2);

  int random_seed;
  nh.param("random_seed", random_seed, 0);

  // Every input, the graph instances included, is validated before the first result file is opened.
  if (!(visualization_rate > 0.0)) {
    ROS_FATAL_STREAM("Invalid 'visualization_rate' " << visualization_rate << ". Must be positive.");
    return 1;
  }
  // A finite budget is what ends a run whose every planned edge fits: the loop below stops at cost >= cost_budget.
  if (!std::isfinite(cost_budget)) {
    ROS_FATAL_STREAM("Invalid 'cost_budget' " << cost_budget << ". Must be finite.");
    return 1;
  }
  std::string graph_type_folder;
  if (!nh.getParam("graph_type_folder", graph_type_folder) || graph_type_folder.empty()) {
    ROS_FATAL("Required parameter 'graph_type_folder' is unset or empty.");
    return 1;
  }
  if (!std::filesystem::is_directory(graph_type_folder)) {
    ROS_FATAL_STREAM("Parameter 'graph_type_folder' is not a directory: " << graph_type_folder);
    return 1;
  }
  // Results never go into graph_type_folder: data/ holds the tracked inputs only.
  std::string results_folder;
  if (!nh.getParam("results_folder", results_folder) || results_folder.empty()) {
    ROS_FATAL("Required parameter 'results_folder' is unset or empty.");
    return 1;
  }

  std::string replan_type_str;
  nh.param<std::string>("replan_type", replan_type_str, "everystep");
  std::vector<std::string> replan_types;
  if (replan_type_str == "try_every_option") {
    replan_types = {"none", "everystep", "goal"};
  } else {
    replan_types.push_back(replan_type_str);
  }
  for (const std::string& replan_type : replan_types) {
    if (replan_type != "none" && replan_type != "goal" && replan_type != "everystep") {
      ROS_FATAL_STREAM("Unknown replan type '" << replan_type << "'. Expected one of: none, goal, everystep, try_every_option.");
      return 1;
    }
  }

  std::string objective_name;
  nh.param<std::string>("objective", objective_name, "path_gain");
  std::vector<Objective> objectives;
  try {
    if (objective_name == "try_every_option") {
      objectives = {Objective::kPathGain, Objective::kPathRatio, Objective::kExpectedGain, Objective::kTableGain};
    } else {
      objectives = {objectiveFromString(objective_name)};
    }
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Invalid 'objective' parameter: " << error.what());
    return 1;
  }

  // The beam sweeps are geometric, so a step of 1 or less (or a non-positive start) never terminates.
  const auto validate_beam_sweep = [](const char* name, int min_width, int step) {
    if (min_width > 0 && step > 1) {
      return true;
    }
    ROS_FATAL_STREAM("Invalid " << name << " sweep: min_beam_width=" << min_width << " step=" << step
                                << ". Requires min_beam_width > 0 and step > 1.");
    return false;
  };
  if (!validate_beam_sweep("DBS", min_beam_width_dbs, beam_width_step_dbs) ||
      !validate_beam_sweep("NBS", min_beam_width_nbs, beam_width_step_nbs)) {
    return 1;
  }

  // Validate before the cast below: a non-finite bound makes static_cast<int> undefined, and min > max would
  // silently produce an empty sweep -- no SPT or TSP output at all, from a run that otherwise looks successful.
  if (!std::isfinite(top_node_percentage_min) || !std::isfinite(top_node_percentage_max) || !std::isfinite(percentage_step)) {
    ROS_FATAL_STREAM("Non-finite percentage sweep bounds: min=" << top_node_percentage_min << " max=" << top_node_percentage_max
                                                                << " step=" << percentage_step);
    return 1;
  }
  if (percentage_step <= 0.0) {
    ROS_FATAL_STREAM("Invalid 'percentage_step' " << percentage_step << ". Must be positive.");
    return 1;
  }
  if (top_node_percentage_min < 0.0 || top_node_percentage_max > 1.0 || top_node_percentage_min > top_node_percentage_max) {
    ROS_FATAL_STREAM("Invalid percentage sweep [" << top_node_percentage_min << ", " << top_node_percentage_max
                                                  << "]. Requires 0 <= min <= max <= 1.");
    return 1;
  }
  // Drive the percentage sweep from an integer index so accumulated floating-point error cannot drop or
  // duplicate the final step.
  std::vector<double> top_node_percentages;
  const int num_percentage_steps =
      static_cast<int>(std::floor((top_node_percentage_max - top_node_percentage_min) / percentage_step + kSweepStepTolerance));
  for (int i = 0; i <= num_percentage_steps; ++i) {
    top_node_percentages.push_back(top_node_percentage_min + i * percentage_step);
  }

  // Enumerate both beam sweeps up front rather than stepping them inline: the progress lines below need the
  // planned run count before the first run, and the sweep is a function of the parameters only, so it is the
  // same for every block. validate_beam_sweep above has already rejected the inputs that would make these empty.
  const std::vector<int> dbs_beam_widths = geometricBeamWidths(min_beam_width_dbs, max_beam_width_dbs, beam_width_step_dbs);
  const std::vector<int> nbs_beam_widths = geometricBeamWidths(min_beam_width_nbs, max_beam_width_nbs, beam_width_step_nbs);
  const std::size_t runs_per_block = dbs_beam_widths.size() + nbs_beam_widths.size() + 2 * top_node_percentages.size();

  warnOnPerceptionRadiusMismatch(graph_type_folder, perception_radius);

  std::vector<std::string> directories;
  for (const auto& entry : std::filesystem::directory_iterator(graph_type_folder)) {
    if (entry.is_directory()) {
      directories.push_back(entry.path().string());
    }
  }
  // directory_iterator order is unspecified; keep the processing order a function of the dataset.
  std::sort(directories.begin(), directories.end());

  // A graph instance is a subfolder holding a graph.txt, as verify_benchmark_results.py counts them.
  std::vector<std::string> subfolders;
  for (const auto& directory : directories) {
    std::error_code graph_file_error;
    if (std::filesystem::is_regular_file(std::filesystem::path(directory) / "graph.txt", graph_file_error)) {
      subfolders.push_back(directory);
    } else {
      ROS_WARN_STREAM("Skipping " << directory << ": no graph.txt, so not a graph instance.");
    }
  }

  // Each instance is loaded and built from its start node here, so a malformed one stops the node before any
  // result file is opened.
  std::vector<std::unique_ptr<GraphManager>> graph_managers;
  graph_managers.reserve(subfolders.size());
  for (const auto& graph_folder : subfolders) {
    auto graph_manager = std::make_unique<GraphManager>();
    try {
      graph_manager->readGraphDataFromFile(graph_folder + "/graph.txt");
      graph_manager->generateGraphFromData(0, perception_radius);
    } catch (const std::exception& error) {
      ROS_FATAL_STREAM("Cannot load graph instance " << graph_folder << ": " << error.what());
      return 1;
    }
    graph_managers.push_back(std::move(graph_manager));
  }

  const std::size_t blocks_per_graph = objectives.size() * replan_types.size();
  const std::size_t total_blocks = subfolders.size() * blocks_per_graph;
  const std::size_t total_planned_runs = total_blocks * runs_per_block;
  const int block_index_width = static_cast<int>(std::to_string(total_blocks).size());
  const int run_index_width = static_cast<int>(std::to_string(total_planned_runs).size());

  // State the root and the size of the job once, so every line below can name its graph instance relatively
  // instead of repeating the path, and can carry a denominator instead of a bare count.
  ROS_INFO_STREAM(subfolders.size() << " graph instances in " << graph_type_folder << "; " << total_blocks << " blocks x " << runs_per_block
                                    << " runs = " << total_planned_runs << " planned. Results go to " << results_folder
                                    << ", overwriting existing files.");

  int total_runs = 0;
  int total_skipped = 0;
  double total_seconds = 0.0;
  std::size_t graph_index = 0;
  std::size_t block_index = 0;

  for (const auto& graph_folder : subfolders) {
    const std::string graph_name = std::filesystem::path(graph_folder).filename().string();
    GraphManager& graph_manager = *graph_managers[graph_index];
    ++graph_index;

    ROS_INFO_STREAM("graph " << graph_index << '/' << subfolders.size() << "  " << graph_name << "  " << graph_manager.getTotalNodeCount()
                             << " nodes, " << blocks_per_graph << " blocks, " << blocks_per_graph * runs_per_block << " runs");

    PluginFactoryConfig factory;
    Configurable::ParamMap* param_map = nullptr;
    std::unique_ptr<BenchmarkAgent> agent = std::make_unique<BenchmarkAgent>(&factory, param_map);
    agent->setRandomSeed(random_seed);

    for (const auto& obj : objectives) {
      for (const auto& replan_type : replan_types) {
        std::string result_folder = results_folder + "/" + graph_name + "/" + replan_type + "/" + toString(obj);
        ++block_index;

        // 34, not the label's own 31 (`graph_0 expected_gain/everystep`): a two-digit graph index
        // reaches exactly 32 and would leave no separator before the run count.
        constexpr int kBlockLabelWidth = 34;
        std::ostringstream block_label;
        block_label << graph_name << ' ' << toString(obj) << '/' << replan_type;
        std::ostringstream block_prefix;
        block_prefix << "block " << std::setw(block_index_width) << block_index << '/' << total_blocks << "  " << std::left
                     << std::setw(kBlockLabelWidth) << block_label.str() << std::right;

        // create_directories returns false without an error when the directory already exists, so the
        // error_code, not the bool, says whether the block can write. Skipping a block leaves the others as they
        // would be: no state crosses a run, since every planner reseeds from (random_seed, its own namespace) in
        // setupRaw and runIteration resets the cost budget before anything else.
        std::error_code create_error;
        std::filesystem::create_directories(result_folder, create_error);
        if (create_error) {
          ROS_ERROR_STREAM("Skipping " << block_prefix.str() << ": cannot create result directory " << result_folder << ": "
                                       << create_error.message() << " (" << runs_per_block << " runs not attempted)");
          total_skipped += static_cast<int>(runs_per_block);
          continue;
        }

        ROS_INFO_STREAM(block_prefix.str() << std::setw(3) << runs_per_block << " runs  start");

        int block_runs = 0;
        int block_skipped = 0;
        int block_created = 0;
        std::map<std::string, int> block_reason_counts;
        std::vector<std::string> claimed_filenames;
        claimed_filenames.reserve(runs_per_block);

        // Counts the run, records why it stopped and which file it claimed, and -- only at ~log_level:=debug --
        // prints the one line per run the block line deliberately does not carry. It lives here because only
        // the block loop knows a run's index within the block and within the sweep.
        const auto tally = [&](const RunResult& result) {
          claimed_filenames.push_back(std::filesystem::path(result.result_filepath).filename().string());
          if (!result.wrote_file) {
            ++block_skipped;
          } else {
            ++block_runs;
            if (!result.existed_before) {
              ++block_created;
            }
            ++block_reason_counts[result.reason_kind];
          }
          ROS_DEBUG_STREAM("[benchmark_run] block="
                           << block_index << '/' << total_blocks << " run=" << (block_runs + block_skipped) << '/' << runs_per_block
                           << " overall=" << (total_runs + total_skipped + block_runs + block_skipped) << '/' << total_planned_runs
                           << " result=" << claimed_filenames.back() << " reason=" << (result.wrote_file ? result.reason_kind : "skipped")
                           << (result.existed_before ? "" : " new"));
        };
        const ::ros::WallTime block_start = ::ros::WallTime::now();

        // Every run starts from the loaded gains: runIteration() resets the graph and rebuilds it from the loaded
        // data.
        for (const int beam_width : dbs_beam_widths) {
          tally(runDbsIteration(obj, beam_width, beam_depth, agent.get(), &graph_manager, cost_budget, result_folder, replan_type));
        }

        for (const int beam_width : nbs_beam_widths) {
          tally(runNbsIteration(obj, beam_width, beam_depth, agent.get(), &graph_manager, cost_budget, result_folder, replan_type));
        }

        for (const double top_node_percentage : top_node_percentages) {
          tally(runSptIteration(obj, top_node_percentage, agent.get(), &graph_manager, cost_budget, result_folder, replan_type));
        }

        for (const double top_node_percentage : top_node_percentages) {
          tally(runTspIteration(obj, top_node_percentage, agent.get(), &graph_manager, cost_budget, result_folder, replan_type));
        }

        const double block_seconds = (::ros::WallTime::now() - block_start).toSec();
        total_runs += block_runs;
        total_skipped += block_skipped;
        total_seconds += block_seconds;

        // Progress counts attempted runs, skipped ones included, so it reaches total_planned_runs.
        const int attempted = total_runs + total_skipped;

        std::ostringstream block_skips;
        if (block_skipped > 0) {
          block_skips << "  (" << block_skipped << " skipped)";
        }
        std::ostringstream block_new;
        if (block_created > 0) {
          block_new << "  (" << block_created << " new)";
        }
        // Listed only when the block's runs did not all stop for the same reason.
        std::ostringstream block_reasons;
        if (block_reason_counts.size() > 1) {
          block_reasons << "  reasons=";
          bool first = true;
          for (const auto& [kind, count] : block_reason_counts) {
            block_reasons << (first ? "" : ",") << kind << ':' << count;
            first = false;
          }
        }
        ROS_INFO_STREAM(block_prefix.str() << std::setw(3) << block_runs << " runs  " << std::fixed << std::setprecision(1) << std::setw(6)
                                           << block_seconds << "s  progress " << std::setw(run_index_width) << attempted << '/'
                                           << total_planned_runs << " (" << std::setw(5) << (100.0 * attempted / total_planned_runs) << "%)"
                                           << block_skips.str() << block_reasons.str() << block_new.str());

        // Only after a block that wrote every run.
        if (block_runs == static_cast<int>(runs_per_block)) {
          reportStaleResultFiles(result_folder, claimed_filenames, block_prefix.str());
        }
      }
    }
  }

  ROS_INFO_STREAM((total_runs + total_skipped) << '/' << total_planned_runs << " runs in " << std::fixed << std::setprecision(1)
                                               << total_seconds << 's');
  if (total_skipped > 0) {
    // Skipped runs leave result files missing or stale, which only surfaces later as gaps in the plots.
    ROS_ERROR_STREAM(total_skipped << " runs were skipped and produced no results; see the errors above.");
  }

  return 0;
}
