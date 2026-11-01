#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_graph_benchmark/graph_manager.h"

namespace open_apero {
namespace {

// Marker positions come straight from the graph file with no arithmetic in between, so this only absorbs
// representation noise.
constexpr double kPositionTolerance = 1e-9;

/// @brief Scratch file holding graph text in the benchmark format: `id x y gain [neighbor...]`.
class TemporaryGraphFile {
 public:
  /// @brief Default three-node chain: gains 0/50/100 at x = 0/1/5.
  TemporaryGraphFile()
      : TemporaryGraphFile(
            "0 0 0 0 1\n"
            "1 1 0 50 0 2\n"
            "2 5 0 100 1\n") {}

  explicit TemporaryGraphFile(const std::string& contents) {
    char path[] = "/tmp/open_apero_graph_benchmark_XXXXXX";
    const int fd = mkstemp(path);
    if (fd >= 0) {
      close(fd);
    }
    path_ = path;

    std::ofstream file(path_);
    file << contents;
  }

  ~TemporaryGraphFile() { unlink(path_.c_str()); }

  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

/// @brief Original IDs of every node currently in the graph, ascending.
std::vector<int> originalIdsInGraph(const GraphManager& manager, GraphType* graph) {
  std::vector<int> ids;
  auto vertices = boost::vertices(*graph);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    ids.push_back(manager.getOriginalId((*graph)[*it].get()));
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

std::string readFile(const std::string& path) {
  std::ifstream file(path);
  return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

const VisualizationMarker* findNodeMarkerAtX(const VisualizationMarkers& markers, double x) {
  for (const auto& marker : markers.getMarkers()) {
    if (marker.ns == "nodes" && std::abs(marker.position.x() - x) < kPositionTolerance) {
      return &marker;
    }
  }
  return nullptr;
}

const VisualizationMarker* findCurrentNodeMarker(const VisualizationMarkers& markers) {
  const VisualizationMarker* result = nullptr;
  for (const auto& marker : markers.getMarkers()) {
    if (marker.ns != "current_node") {
      continue;
    }
    EXPECT_EQ(result, nullptr) << "Expected exactly one current-node marker";
    result = &marker;
  }
  return result;
}

TEST(GraphManagerVisualization, UsesStableWholeGraphGainRange) {
  TemporaryGraphFile input;
  GraphManager manager;
  manager.readGraphDataFromFile(input.path());

  EXPECT_EQ(manager.getTotalNodeCount(), 3u);
  EXPECT_DOUBLE_EQ(manager.getMinGain(), 0.0);
  EXPECT_DOUBLE_EQ(manager.getMaxGain(), 100.0);

  manager.generateGraphFromData(0, 1.1);
  VisualizationMarkers local_markers;
  manager.getVisualizationMarkers(&local_markers, false);
  const VisualizationMarker* local_mid_gain = findNodeMarkerAtX(local_markers, 1.0);
  ASSERT_NE(local_mid_gain, nullptr);
  EXPECT_EQ(local_mid_gain->type, static_cast<int>(VisualizationMarker::SPHERE));
  EXPECT_GT(local_mid_gain->color.a, 0.0);
  const Color local_color = local_mid_gain->color;

  manager.generateGraphFromData(1, 5.0);
  VisualizationMarkers expanded_markers;
  manager.getVisualizationMarkers(&expanded_markers, false);
  const VisualizationMarker* expanded_mid_gain = findNodeMarkerAtX(expanded_markers, 1.0);
  ASSERT_NE(expanded_mid_gain, nullptr);

  EXPECT_DOUBLE_EQ(expanded_mid_gain->color.r, local_color.r);
  EXPECT_DOUBLE_EQ(expanded_mid_gain->color.g, local_color.g);
  EXPECT_DOUBLE_EQ(expanded_mid_gain->color.b, local_color.b);
  EXPECT_DOUBLE_EQ(expanded_mid_gain->color.a, local_color.a);
}

TEST(GraphManagerVisualization, EmitsOneMarkerAtCurrentNode) {
  TemporaryGraphFile input;
  GraphManager manager;
  manager.readGraphDataFromFile(input.path());

  manager.generateGraphFromData(0, 5.0);
  VisualizationMarkers initial_markers;
  manager.getVisualizationMarkers(&initial_markers, false);
  const VisualizationMarker* initial_current = findCurrentNodeMarker(initial_markers);
  ASSERT_NE(initial_current, nullptr);
  EXPECT_DOUBLE_EQ(initial_current->position.x(), 0.0);
  EXPECT_EQ(initial_current->type, static_cast<int>(VisualizationMarker::SPHERE));
  EXPECT_DOUBLE_EQ(initial_current->color.r, 1.0);
  EXPECT_DOUBLE_EQ(initial_current->color.g, 0.0);
  EXPECT_DOUBLE_EQ(initial_current->color.b, 1.0);

  manager.generateGraphFromData(2, 5.0);
  VisualizationMarkers moved_markers;
  manager.getVisualizationMarkers(&moved_markers, false);
  const VisualizationMarker* moved_current = findCurrentNodeMarker(moved_markers);
  ASSERT_NE(moved_current, nullptr);
  EXPECT_DOUBLE_EQ(moved_current->position.x(), 5.0);
}

// Pins the pruning invariant: a node pruned for being unreachable must be re-added once it becomes reachable.
TEST(GraphManagerPruning, PrunedNodeIsReaddedOnceItBecomesReachable) {
  // Node 2's only neighbor is node 4, which sits outside the initial 3 m radius, so node 2 arrives with no edges.
  TemporaryGraphFile input(
      "0 0 0 0 1\n"
      "1 1 0 0 0 4\n"
      "2 2 0 0 4\n"
      "4 4 0 0 1 2\n");
  GraphManager manager;
  manager.readGraphDataFromFile(input.path());

  GraphType* graph = manager.generateGraphFromData(0, 3.0);
  EXPECT_EQ(originalIdsInGraph(manager, graph), (std::vector<int>{0, 1})) << "node 2 is isolated and must be pruned";
  EXPECT_EQ(manager.getNodeByOriginalId(2), nullptr);

  // Widening the radius brings node 4 in, which links node 2 back to the start component.
  graph = manager.generateGraphFromData(1, 20.0);
  EXPECT_EQ(originalIdsInGraph(manager, graph), (std::vector<int>{0, 1, 2, 4}));
  ASSERT_NE(manager.getNodeByOriginalId(2), nullptr);
  EXPECT_EQ(manager.getOriginalId(manager.getNodeByOriginalId(2)), 2);
}

TEST(GraphManagerLookup, GetOriginalIdRejectsNodesOutsideTheGraph) {
  TemporaryGraphFile input;
  GraphManager manager;
  manager.readGraphDataFromFile(input.path());
  manager.generateGraphFromData(0, 5.0);

  GraphNode stranger;
  EXPECT_THROW(manager.getOriginalId(&stranger), std::out_of_range);
  // A silent operator[] insert would have registered the stranger as ID 0, displacing the real node 0.
  EXPECT_EQ(manager.getOriginalId(manager.getNodeByOriginalId(0)), 0);
}

TEST(GraphManagerLookup, GainBoundsAreInitializedBeforeAnyGraphIsLoaded) {
  GraphManager manager;
  EXPECT_EQ(manager.getTotalNodeCount(), 0u);
  EXPECT_EQ(manager.getMinGain(), kDoubleMax);
  EXPECT_EQ(manager.getMaxGain(), kDoubleMin);
}

// writeGraphToFile writes the vertices actually present: original IDs need not be dense, which is the normal case
// for a graph whose nodes were pruned.
TEST(GraphManagerSerialization, RoundTripsSparseOriginalIds) {
  TemporaryGraphFile input(
      "5 0 0 10 7\n"
      "7 1 0 20 5 9\n"
      "9 2 0 30 7\n");
  GraphManager manager;
  manager.readGraphDataFromFile(input.path());
  manager.generateGraphFromData(5, 10.0);

  TemporaryGraphFile output("");
  manager.writeGraphToFile(output.path());

  EXPECT_EQ(readFile(output.path()),
            "5 0 0 10 7 \n"
            "7 1 0 20 5 9 \n"
            "9 2 0 30 7 \n");

  GraphManager reloaded;
  reloaded.readGraphDataFromFile(output.path());
  GraphType* graph = reloaded.generateGraphFromData(5, 10.0);
  EXPECT_EQ(originalIdsInGraph(reloaded, graph), (std::vector<int>{5, 7, 9}));
}

TEST(GraphManagerGainModel, RejectsBoundsThatIntegerSamplingWouldSilentlyChange) {
  GraphManager manager;

  TemporaryGraphFile fractional("0 0 0 0 12.5\n");
  EXPECT_THROW(manager.readGainModelFromFile(fractional.path()), std::runtime_error);

  TemporaryGraphFile inverted("3 3 1 100 0\n");
  EXPECT_THROW(manager.readGainModelFromFile(inverted.path()), std::runtime_error);

  // Out-of-int-range bounds are undefined to feed to sampleGain()'s static_cast<int>, and the floor() check
  // above accepts them because 1e300 is integral. (A textual "inf" is not tested here: operator>> does not
  // parse it, so such a line is rejected by the format check before validation ever runs. The isfinite guard
  // in readGainModelFromFile is defensive against other callers, not reachable through this path.)
  TemporaryGraphFile too_large("0 0 0 0 1e300\n");
  EXPECT_THROW(manager.readGainModelFromFile(too_large.path()), std::runtime_error);

  TemporaryGraphFile too_negative("3 3 1 -1e300 0\n");
  EXPECT_THROW(manager.readGainModelFromFile(too_negative.path()), std::runtime_error);

  TemporaryGraphFile valid("0 0 0 0 100\n");
  EXPECT_NO_THROW(manager.readGainModelFromFile(valid.path()));
}

// The graph's Boost vertex order and per-vertex adjacency order are what the path planners enumerate candidates
// in, so they decide the winner among candidates that Path::operator> cannot separate.
//
// The contract this pins is the narrow one: identical file contents -> identical vertex and adjacency order, so
// a run is repeatable. It does not extend to invariance under the order nodes are listed in the file: the order
// follows GraphData's hash buckets, which the GraphTspSearch results in data/ depend on (see the GraphData docs).
TEST(GraphManagerDeterminism, TraversalOrderIsRepeatableButNotLineOrderInvariant) {
  const auto traversal_order = [](const std::string& contents) {
    TemporaryGraphFile input(contents);
    GraphManager manager;
    manager.readGraphDataFromFile(input.path());
    GraphType* graph = manager.generateGraphFromData(0, 10.0);

    std::vector<std::vector<int>> order;
    auto vertices = boost::vertices(*graph);
    for (auto it = vertices.first; it != vertices.second; ++it) {
      std::vector<int> row{manager.getOriginalId((*graph)[*it].get())};
      auto neighbors = boost::adjacent_vertices(*it, *graph);
      for (auto n = neighbors.first; n != neighbors.second; ++n) {
        row.push_back(manager.getOriginalId((*graph)[*n].get()));
      }
      order.push_back(row);
    }
    return order;
  };

  const std::string contents =
      "0 0 0 0 1 2\n"
      "1 1 0 0 0 2 3\n"
      "2 2 0 0 0 1 3\n"
      "3 3 0 0 1 2\n";

  // What is guaranteed: the same file read twice yields the same traversal order, so a benchmark run is
  // repeatable on a fixed toolchain. Candidate enumeration order is vertex order x adjacency order, so without
  // this a rerun could pick a different equal-scoring path.
  EXPECT_EQ(traversal_order(contents), traversal_order(contents));

  // What is not guaranteed: invariance to the order nodes are listed in, which would need GraphData ordered by ID
  // and would change the GraphTspSearch results (see the GraphData docs).
  const auto ascending = traversal_order(contents);
  const auto descending = traversal_order(
      "3 3 0 0 1 2\n"
      "2 2 0 0 0 1 3\n"
      "1 1 0 0 0 2 3\n"
      "0 0 0 0 1 2\n");
  EXPECT_NE(ascending, descending) << "GraphData iteration order became line-order invariant; see the GraphData docs.";
}

TEST(GraphManagerGainModel, SeededSamplingIsReproducible) {
  TemporaryGraphFile topology(
      "0 0 0 0 1\n"
      "1 1 0 0 0 2\n"
      "2 2 0 0 1\n");
  TemporaryGraphFile gain_model("0 0 0 0 100\n");

  const auto sample_gains = [&](int seed) {
    GraphManager manager;
    manager.setGainSamplingSeed(seed);
    manager.readGraphDataFromFile(topology.path());
    GraphType* graph = manager.generateGraphFromData(0, 10.0);
    manager.readGainModelFromFile(gain_model.path());
    manager.assignNewGains();

    std::vector<double> gains;
    auto vertices = boost::vertices(*graph);
    for (auto it = vertices.first; it != vertices.second; ++it) {
      gains.push_back((*graph)[*it]->getSingleNodeGain());
    }
    return gains;
  };

  EXPECT_EQ(sample_gains(7), sample_gains(7));
}

}  // namespace
}  // namespace open_apero
