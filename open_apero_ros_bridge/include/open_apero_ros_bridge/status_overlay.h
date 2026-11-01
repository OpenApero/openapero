#pragma once

/**
 * @file status_overlay.h
 * @brief The RViz status overlay: lines of text in a jsk_rviz_plugins/OverlayText box.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <jsk_rviz_plugins/OverlayText.h>

namespace open_apero {
namespace ros {
namespace status_overlay_detail {

// DejaVu Sans Mono at kTextSizePt points and 96 dpi. Qt hints glyph advance, ascent and
// descent to whole pixels, so each is rounded up.
constexpr double kTextSizePt = 18.0;
constexpr double kPixelsPerPoint = 96.0 / 72.0;
constexpr double kGlyphAdvanceEm = 0.602;
constexpr double kAscentEm = 0.928;
constexpr double kDescentEm = 0.236;
constexpr int kMarginPx = 8;
constexpr int kSlackPx = 2;

inline std::string escapeMarkup(const std::string& text) {
  std::string escaped;
  escaped.reserve(text.size());
  for (const char c : text) {
    switch (c) {
      case '&':
        escaped += "&amp;";
        break;
      case '<':
        escaped += "&lt;";
        break;
      case '>':
        escaped += "&gt;";
        break;
      default:
        escaped += c;
    }
  }
  return escaped;
}

}  // namespace status_overlay_detail

/**
 * @brief The status overlay showing `lines` top to bottom, at the top-left of the view.
 *
 * The plugin draws the text as Qt rich text, so the lines go out as one preformatted block:
 * spacing is kept as written and markup characters are escaped. The box is sized from
 * DejaVu Sans Mono's metrics, so a line never wraps.
 * @param lines Plain text; columns padded with spaces line up, the font is monospaced.
 * @return The message to publish.
 */
inline jsk_rviz_plugins::OverlayText buildStatusOverlay(const std::vector<std::string>& lines) {
  namespace detail = status_overlay_detail;
  std::string text;
  std::size_t widest = 0u;
  for (std::size_t i = 0u; i < lines.size(); ++i) {
    if (i > 0u) {
      text += "\n";
    }
    text += detail::escapeMarkup(lines[i]);
    widest = std::max(widest, lines[i].size());
  }

  const double em_px = detail::kTextSizePt * detail::kPixelsPerPoint;
  const int glyph_px = static_cast<int>(std::ceil(detail::kGlyphAdvanceEm * em_px));
  const int line_px = static_cast<int>(std::ceil(detail::kAscentEm * em_px) + std::ceil(detail::kDescentEm * em_px));

  jsk_rviz_plugins::OverlayText msg;
  msg.action = jsk_rviz_plugins::OverlayText::ADD;
  msg.text = "<pre style=\"margin: " + std::to_string(detail::kMarginPx) + "px;\">" + text + "</pre>";
  msg.width = static_cast<int>(widest) * glyph_px + 2 * detail::kMarginPx + detail::kSlackPx;
  msg.height = static_cast<int>(lines.size()) * line_px + 2 * detail::kMarginPx + detail::kSlackPx;
  msg.left = 16;
  msg.top = 16;
  msg.text_size = static_cast<float>(detail::kTextSizePt);
  msg.line_width = 2;
  msg.font = "DejaVu Sans Mono";
  msg.bg_color.r = 0.02;
  msg.bg_color.g = 0.03;
  msg.bg_color.b = 0.04;
  msg.bg_color.a = 0.72;
  msg.fg_color.r = 0.80;
  msg.fg_color.g = 0.96;
  msg.fg_color.b = 1.0;
  msg.fg_color.a = 1.0;
  return msg;
}

}  // namespace ros
}  // namespace open_apero
