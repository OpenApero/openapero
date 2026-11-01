#include "open_apero_graph_benchmark/graph_manager.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <open_apero_core/common/typedef.h>
#include <open_apero_core/logging/logging.h>
#include <open_apero_core/visualization/visualization_helpers.h>

namespace open_apero {

namespace {
// A gain-model region whose center and radius are all within this of zero is the all-zero sentinel that
// defines the fallback gain range, rather than a real region at the origin.
constexpr double kFallbackRegionTolerance = 1e-4;

std::ifstream openReadFile(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    throw std::runtime_error("Error: Could not open file " + filepath);
  }
  return file;
}

std::ofstream openWriteFile(const std::string& filepath) {
  std::ofstream file(filepath, std::ios::trunc);
  if (!file.is_open()) {
    throw std::runtime_error("Error: Could not open file " + filepath);
  }
  return file;
}
}  // namespace

void GraphManager::readGraphDataFromFile(const std::string& input_graph_filepath) {
  graph_data_.clear();
  max_gain_ = kDoubleMin;
  min_gain_ = kDoubleMax;

  std::ifstream input_file = openReadFile(input_graph_filepath);

  std::string line;
  while (std::getline(input_file, line)) {
    std::istringstream iss(line);
    int original_node_id;
    double pos_x, pos_y, gain;

    if (!(iss >> original_node_id >> pos_x >> pos_y >> gain)) {
      throw std::runtime_error("Malformed line in graph file " + input_graph_filepath + ": " + line);
    }

    int neighbor_id;
    std::vector<int> neighbors;
    while (iss >> neighbor_id) {
      neighbors.push_back(neighbor_id);
    }

    NodeData node_data;
    node_data.pos = Eigen::Vector3d(pos_x, pos_y, 0.0);
    node_data.gain = gain;
    max_gain_ = std::max(max_gain_, gain);
    min_gain_ = std::min(min_gain_, gain);
    node_data.adjacent_original_ids = neighbors;

    graph_data_[original_node_id] = node_data;
  }

  input_file.close();
}

void GraphManager::reset() {
  graph_.clear();
  current_node_ = nullptr;
  node_ptr_to_original_id_map_.clear();
  original_id_to_node_ptr_map_.clear();
  original_id_to_gain_map_.clear();
  original_id_to_node_id_map_.clear();
}

GraphType* GraphManager::generateGraphFromData(int current_node_id, double max_radius) {
  std::vector<std::pair<int, std::vector<int>>> edges_to_add;

  const auto current_node_data = graph_data_.find(current_node_id);
  if (current_node_data == graph_data_.end()) {
    throw std::runtime_error("Error: Node " + std::to_string(current_node_id) + " is not in the loaded graph data");
  }
  const Eigen::Vector3d current_position = current_node_data->second.pos;

  for (const auto& [original_node_id, node_data] : graph_data_) {
    if (original_id_to_node_ptr_map_.count(original_node_id)) {
      continue;
    }

    if ((node_data.pos - current_position).norm() > max_radius) {
      continue;
    }

    auto node = std::make_shared<GraphNode>();
    node->robot_state.setBasePosition(node_data.pos);
    node->robot_state.setSensorPosition(node_data.pos);
    node->is_traversable = true;
    node->gain_anchors = {std::make_shared<GainAnchor>(node_data.gain)};

    NodeId new_node_id = boost::add_vertex(node, graph_);

    node_ptr_to_original_id_map_[node.get()] = original_node_id;
    original_id_to_node_ptr_map_[original_node_id] = node.get();
    original_id_to_gain_map_[original_node_id] = node->getSingleNodeGain();
    original_id_to_node_id_map_[original_node_id] = new_node_id;

    // Adjacency keeps the graph file's order: Boost's per-vertex adjacency order is the second half of the
    // candidate enumeration order planners see, which GraphTspSearch results depend on (see the GraphData docs).
    edges_to_add.push_back({original_node_id, node_data.adjacent_original_ids});
  }

  if (!original_id_to_node_ptr_map_.count(current_node_id)) {
    throw std::runtime_error("Error: Current node " + std::to_string(current_node_id) + " is not in the generated graph");
  }
  current_node_ = original_id_to_node_ptr_map_.at(current_node_id);

  for (const auto& [original_node_id, adjacent_original_node_ids] : edges_to_add) {
    NodeId node_id = original_id_to_node_id_map_.at(original_node_id);

    for (const int& adjacent_original_node_id : adjacent_original_node_ids) {
      if (!original_id_to_node_ptr_map_.count(adjacent_original_node_id)) {
        continue;
      }

      NodeId adjacent_node_id = original_id_to_node_id_map_.at(adjacent_original_node_id);
      if (boost::edge(node_id, adjacent_node_id, graph_).second) {
        continue;
      }

      auto edge_from_node = std::make_shared<Edge>();
      edge_from_node->trajectory = {graph_[node_id]->robot_state, graph_[adjacent_node_id]->robot_state};
      edge_from_node->cost =
          (edge_from_node->trajectory.back().getSensorPosition() - edge_from_node->trajectory.front().getSensorPosition()).norm();
      boost::add_edge(node_id, adjacent_node_id, edge_from_node, graph_);

      auto edge_to_node = std::make_shared<Edge>();
      edge_to_node->trajectory = {graph_[adjacent_node_id]->robot_state, graph_[node_id]->robot_state};
      edge_to_node->cost =
          (edge_to_node->trajectory.back().getSensorPosition() - edge_to_node->trajectory.front().getSensorPosition()).norm();
      boost::add_edge(adjacent_node_id, node_id, edge_to_node, graph_);
    }
  }

  auto vertices = boost::vertices(graph_);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    GraphNode* candidate_node = graph_[*it].get();
    int candidate_original_id = getOriginalId(candidate_node);
    const Eigen::Vector3d& candidate_position = candidate_node->robot_state.getSensorPosition();

    // A node is at the frontier when an unperceived node lies within max_radius of it.
    candidate_node->is_at_frontier = false;
    for (const auto& [original_node_id, node_data] : graph_data_) {
      if (candidate_original_id == original_node_id) {
        continue;
      }

      if (original_id_to_node_ptr_map_.count(original_node_id)) {
        continue;
      }

      if ((node_data.pos - candidate_position).norm() <= max_radius) {
        candidate_node->is_at_frontier = true;
        break;
      }
    }
  }

  std::vector<Node*> connected_nodes = runBfs(current_node_, &graph_);
  for (auto node : connected_nodes) {
    node->as<GraphNode>()->connect_with_start_node = true;
  }

  // Resolve each unreachable vertex to its original ID *before* removing it: the vertex bundle (and with it the
  // GraphNode the maps are keyed on) is destroyed by remove_vertex, so it cannot be read afterwards.
  std::vector<std::pair<NodeId, int>> nodes_to_remove;
  for (auto it = vertices.first; it != vertices.second; ++it) {
    GraphNode* node = graph_[*it].get();
    if (node->connect_with_start_node) {
      continue;
    }
    nodes_to_remove.emplace_back(*it, getOriginalId(node));
  }

  for (const auto& [node_id, original_id] : nodes_to_remove) {
    node_ptr_to_original_id_map_.erase(graph_[node_id].get());
    original_id_to_node_ptr_map_.erase(original_id);
    original_id_to_gain_map_.erase(original_id);
    original_id_to_node_id_map_.erase(original_id);

    boost::clear_vertex(node_id, graph_);
    boost::remove_vertex(node_id, graph_);
  }

  return &graph_;
}

void GraphManager::readGainModelFromFile(const std::string& gain_model_filepath) {
  std::ifstream input_file = openReadFile(gain_model_filepath);

  std::string line;
  while (std::getline(input_file, line)) {
    std::istringstream iss(line);
    GainRegion region;
    if (!(iss >> region.x_center >> region.y_center >> region.radius >> region.min_gain >> region.max_gain)) {
      logging::error("Incorrect format in gain model file " + gain_model_filepath + ", skipping line: " + line);
      continue;
    }

    // An all-zero region defines the fallback gain range.
    if (std::abs(region.x_center) < kFallbackRegionTolerance && std::abs(region.y_center) < kFallbackRegionTolerance &&
        std::abs(region.radius) < kFallbackRegionTolerance) {
      region.radius = std::numeric_limits<double>::infinity();
      logging::info("Default gain range: min = " + std::to_string(region.min_gain) + ", max = " + std::to_string(region.max_gain));
    }

    // Gains are sampled as integers (see sampleGain()), which the generator and the gain_<min>to<max> dataset
    // naming both assume. Reject bounds that would not survive that, rather than truncating them. All three
    // checks are needed before sampleGain()'s static_cast<int>: a non-finite or out-of-int-range bound is
    // undefined behavior to convert, and floor() alone accepts both (floor(inf) == inf, and 1e300 is
    // integral).
    const auto reject = [&](const std::string& why) {
      throw std::runtime_error("Error: " + why + " gain bounds [" + std::to_string(region.min_gain) + ", " +
                               std::to_string(region.max_gain) + "] in gain model file " + gain_model_filepath +
                               "; gains are sampled as integers");
    };
    if (!std::isfinite(region.min_gain) || !std::isfinite(region.max_gain)) {
      reject("Non-finite");
    }
    if (region.min_gain != std::floor(region.min_gain) || region.max_gain != std::floor(region.max_gain)) {
      reject("Non-integer");
    }
    if (region.min_gain < static_cast<double>(std::numeric_limits<int>::min()) ||
        region.max_gain > static_cast<double>(std::numeric_limits<int>::max())) {
      reject("Out-of-int-range");
    }
    if (region.min_gain > region.max_gain) {
      reject("Inverted");
    }

    gain_regions_.push_back(region);
  }

  input_file.close();
}

void GraphManager::sampleGain(GraphNode* node) {
  double x = node->robot_state.getSensorPosition().x();
  double y = node->robot_state.getSensorPosition().y();
  bool found = false;

  for (const auto& region : gain_regions_) {
    if (std::pow(x - region.x_center, 2) + std::pow(y - region.y_center, 2) <= std::pow(region.radius, 2)) {
      // Integral by design; readGainModelFromFile rejects bounds that are not.
      std::uniform_int_distribution<int> distribution(static_cast<int>(region.min_gain), static_cast<int>(region.max_gain));
      auto gain = distribution(rng_.engine);
      node->gain_anchors = {std::make_shared<GainAnchor>(gain)};
      found = true;
      break;
    }
  }

  if (!found) {
    throw std::runtime_error("Error: No gain-model region found for node at position (" + std::to_string(x) + ", " + std::to_string(y) +
                             ")");
  }
}

void GraphManager::assignNewGains() {
  auto vertices = boost::vertices(graph_);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    GraphNode* node = graph_[*it].get();
    sampleGain(node);

    original_id_to_gain_map_[getOriginalId(node)] = node->getSingleNodeGain();
  }
}

void GraphManager::writeGraphToFile(const std::string& output_graph_filepath) {
  std::ofstream output_graph = openWriteFile(output_graph_filepath);

  // Drive the output from the vertices actually present in the graph. Original IDs are not required to be dense
  // or to survive graph generation (generateGraphFromData prunes nodes unreachable from the start node), so
  // walking 0..num_vertices-1 would both skip live nodes and dereference absent ones.
  std::vector<std::pair<int, NodeId>> nodes_by_original_id;
  auto vertices = boost::vertices(graph_);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    nodes_by_original_id.emplace_back(getOriginalId(graph_[*it].get()), *it);
  }
  std::sort(nodes_by_original_id.begin(), nodes_by_original_id.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

  for (const auto& [original_id, node_id] : nodes_by_original_id) {
    const Node* node = graph_[node_id].get();

    output_graph << original_id << ' ' << node->robot_state.getSensorPosition().x() << ' ' << node->robot_state.getSensorPosition().y()
                 << ' ' << original_id_to_gain_map_.at(original_id) << ' ';

    // Sorted so written files are canonical. The input order cannot round-trip regardless: listS prepends, so
    // Boost hands neighbors back reversed. Reading existing datasets is unaffected (see the GraphData docs).
    std::vector<int> adjacent_original_ids;
    auto neighbors = boost::adjacent_vertices(node_id, graph_);
    for (auto it = neighbors.first; it != neighbors.second; ++it) {
      adjacent_original_ids.push_back(getOriginalId(graph_[*it].get()));
    }

    std::sort(adjacent_original_ids.begin(), adjacent_original_ids.end());
    for (int neighbor_original_id : adjacent_original_ids) {
      output_graph << neighbor_original_id << ' ';
    }

    output_graph << '\n';
  }

  output_graph.close();
}

void GraphManager::getVisualizationMarkers(VisualizationMarkers* markers, bool visualize_edges) {
  std::vector<Node*> all_nodes;
  auto vertices = boost::vertices(graph_);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    all_nodes.push_back((graph_)[*it].get());
  }

  std::vector<Edge*> all_edges;
  if (visualize_edges) {
    auto edges = boost::edges(graph_);
    for (auto it = edges.first; it != edges.second; ++it) {
      all_edges.push_back((graph_)[*it].get());
    }
  }

  getNodeVisualizationMarkers(markers, all_nodes, /*visualize_orientation=*/false, min_gain_, max_gain_);
  if (visualize_edges) {
    getEdgeVisualizationMarkers(markers, all_edges, /*is_edge_bidirectional=*/false);
  }

  if (current_node_) {
    VisualizationMarker current_marker;
    current_marker.ns = "current_node";
    current_marker.type = VisualizationMarker::SPHERE;
    current_marker.action = VisualizationMarker::ADD;
    current_marker.position = current_node_->robot_state.getSensorPosition();
    current_marker.position.z() += 0.16;
    current_marker.scale = Eigen::Vector3d(0.42, 0.42, 0.20);
    current_marker.color = Color(1.0, 0.0, 1.0, 0.95);
    markers->addMarker(current_marker);
  }
}

}  // namespace open_apero
