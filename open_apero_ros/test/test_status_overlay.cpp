#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <open_apero_ros_bridge/status_overlay.h>

namespace open_apero {
namespace ros {
namespace {

// DejaVu Sans Mono at 18 pt and 96 dpi, hinted: 15 px a glyph, 23 + 6 px a line; 8 px margin
// each side and 2 px slack.
constexpr int kGlyphPx = 15;
constexpr int kLinePx = 29;
constexpr int kFramePx = 2 * 8 + 2;

TEST(StatusOverlayTest, KeepsSpacingInsideOnePreformattedBlock) {
  const jsk_rviz_plugins::OverlayText msg = buildStatusOverlay({"On planning", "Elapsed   64.1 s"});
  EXPECT_EQ(msg.text, "<pre style=\"margin: 8px;\">On planning\nElapsed   64.1 s</pre>");
  EXPECT_EQ(msg.action, jsk_rviz_plugins::OverlayText::ADD);
  EXPECT_EQ(msg.font, "DejaVu Sans Mono");
}

TEST(StatusOverlayTest, EscapesMarkup) {
  const jsk_rviz_plugins::OverlayText msg = buildStatusOverlay({"a<b & c>d"});
  EXPECT_EQ(msg.text, "<pre style=\"margin: 8px;\">a&lt;b &amp; c&gt;d</pre>");
}

TEST(StatusOverlayTest, BoxFitsTheWidestLineAndEveryLine) {
  const std::vector<std::string> lines{"On backtracking", "Traversed 4.2 m", "Elapsed   64.1 s"};
  const jsk_rviz_plugins::OverlayText msg = buildStatusOverlay(lines);
  EXPECT_EQ(msg.width, 16 * kGlyphPx + kFramePx);
  EXPECT_EQ(msg.height, 3 * kLinePx + kFramePx);
  EXPECT_EQ(msg.left, 16);
  EXPECT_EQ(msg.top, 16);
  // The width is measured on the line as written, not on the escaped text.
  EXPECT_EQ(buildStatusOverlay({"a&b"}).width, 3 * kGlyphPx + kFramePx);
}

TEST(StatusOverlayTest, NoLinesIsAnEmptyBox) {
  const jsk_rviz_plugins::OverlayText msg = buildStatusOverlay({});
  EXPECT_EQ(msg.text, "<pre style=\"margin: 8px;\"></pre>");
  EXPECT_EQ(msg.width, kFramePx);
  EXPECT_EQ(msg.height, kFramePx);
}

}  // namespace
}  // namespace ros
}  // namespace open_apero
