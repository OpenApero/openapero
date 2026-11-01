/**
 * @file test_node_markers.cpp
 * @brief Tests of getNodeVisualizationMarkers(): color is gain on the Heat scale, alpha whether a node earns gain,
 *        thickness whether it is a frontier node.
 */

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/third_party/tinycolormap.h"
#include "open_apero_core/visualization/colormap.h"
#include "open_apero_core/visualization/visualization_helpers.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {
namespace {

class NodeMarkersTest : public ::testing::Test {
 protected:
  /// @brief A traversable node carrying one anchor of @p gain; gain 0 carries none.
  Node* addNode(double gain, bool is_at_frontier = false) {
    auto node = std::make_shared<GraphNode>();
    node->robot_state.setSensorPosition(Eigen::Vector3d(static_cast<double>(owned_.size()), 0.0, 0.0));
    node->is_traversable = true;
    node->is_at_frontier = is_at_frontier;
    if (gain > 0.0) {
      node->gain_anchors = {std::make_shared<GainAnchor>(gain)};
    }
    owned_.push_back(node);
    nodes_.push_back(node.get());
    return node.get();
  }

  /// @brief The markers, one per node in the order the nodes were added.
  std::vector<VisualizationMarker> draw() {
    VisualizationMarkers markers;
    getNodeVisualizationMarkers(&markers, nodes_);
    return markers.getMarkers();
  }

  /// @brief @p color is the Heat scale at @p x, 0 the blue end and 1 the red end.
  static void expectHeat(const Color& color, double x) {
    const tinycolormap::Color heat = tinycolormap::GetColor(x, tinycolormap::ColormapType::Heat);
    EXPECT_DOUBLE_EQ(color.r, heat.r());
    EXPECT_DOUBLE_EQ(color.g, heat.g());
    EXPECT_DOUBLE_EQ(color.b, heat.b());
  }

  std::vector<std::shared_ptr<GraphNode>> owned_;
  std::vector<Node*> nodes_;
};

TEST_F(NodeMarkersTest, ASuppressedNodeIsDrawnAsAZeroGainNode) {
  addNode(0.0);
  Node* suppressed = addNode(50.0);
  suppressed->is_gain_suppressed = true;
  addNode(10.0);

  const std::vector<VisualizationMarker> markers = draw();

  ASSERT_EQ(markers.size(), 3u);
  EXPECT_DOUBLE_EQ(markers[1].color.a, 0.1);
  expectHeat(markers[1].color, 0.0);
  EXPECT_DOUBLE_EQ(markers[2].color.a, 1.0);
  expectHeat(markers[2].color, 1.0);  // The suppressed 50 does not set the top of the scale; the 10 does.
}

TEST_F(NodeMarkersTest, AFrontierNodeWithNoGainIsFaintButThick) {
  addNode(0.0, /*is_at_frontier=*/true);
  addNode(0.0);

  const std::vector<VisualizationMarker> markers = draw();

  ASSERT_EQ(markers.size(), 2u);
  EXPECT_DOUBLE_EQ(markers[0].color.a, 0.1);
  EXPECT_GT(markers[0].scale.y(), markers[1].scale.y());
}

TEST_F(NodeMarkersTest, EqualGainsTakeTheColormapsLowEnd) {
  addNode(5.0);
  addNode(5.0);

  const std::vector<VisualizationMarker> markers = draw();

  ASSERT_EQ(markers.size(), 2u);
  expectHeat(markers[0].color, 0.0);
  EXPECT_DOUBLE_EQ(markers[0].color.a, 1.0);
}

}  // namespace
}  // namespace open_apero
