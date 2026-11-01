#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

void VisualizationMarkers::addMarker(const VisualizationMarker& marker) {
  VisualizationMarker m = marker;
  m.id = getNextVisualizationId();
  markers_.push_back(std::move(m));
}

void VisualizationMarkers::addMarkers(const std::vector<VisualizationMarker>& markers) {
  for (const auto& marker : markers) {
    addMarker(marker);
  }
}

int VisualizationMarkers::getNextVisualizationId() {
  if (markers_.empty()) {
    return 0;
  } else {
    return markers_.back().id + 1;
  }
}

const std::vector<VisualizationMarker>& VisualizationMarkers::getMarkers() const {
  return markers_;
}

std::vector<VisualizationMarker>& VisualizationMarkers::getMarkers() {
  return markers_;
}

}  // namespace open_apero
