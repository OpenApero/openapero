#pragma once

/**
 * @file visualization_helpers.h
 * @brief Helper functions for converting planning graphs and paths to visualization markers.
 */

#include <vector>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/**
 * @brief Generate visualization markers for a set of nodes.
 *
 * Arrows (@p visualize_orientation) or spheres, intermediate nodes as cubes. Color maps gain on the Heat
 * scale (blue to red) over the supplied nodes' range, a suppressed node counting as gain 0. Alpha is 0 for an
 * untraversable node and 0.1 for one with no gain, suppressed or not; frontier nodes are drawn thicker.
 *
 * @param markers Output marker container.
 * @param all_nodes The nodes to visualize.
 * @param visualize_orientation Whether to draw orientation arrows (else spheres).
 */
void getNodeVisualizationMarkers(VisualizationMarkers* markers, const std::vector<Node*>& all_nodes, bool visualize_orientation = true);

/**
 * @brief Generate node markers using an explicit, stable gain-color range.
 *
 * Equal @p min_gain and @p max_gain give the same fallback color as the range-derived overload.
 *
 * @param markers Output marker container.
 * @param all_nodes The nodes to visualize.
 * @param visualize_orientation Whether to draw orientation arrows (else spheres).
 * @param min_gain Lower bound of the gain colormap.
 * @param max_gain Upper bound of the gain colormap.
 */
void getNodeVisualizationMarkers(VisualizationMarkers* markers, const std::vector<Node*>& all_nodes, bool visualize_orientation,
                                 double min_gain, double max_gain);

/**
 * @brief Generate visualization markers for a set of edges.
 *
 * One achromatic tone; width and alpha mark considered edges. One mid-arc arrowhead per edge,
 * bidirectional pairs offset sideways, rotation-only edges skipped. At most four markers, in the
 * namespaces `edges`, `edges_unconsidered`, `edge_directions`, `edge_directions_unconsidered`.
 *
 * @param markers Output marker container.
 * @param all_edges The edges to visualize.
 * @param is_edge_bidirectional Whether edges traverse both directions.
 */
void getEdgeVisualizationMarkers(VisualizationMarkers* markers, const std::vector<Edge*>& all_edges, bool is_edge_bidirectional);

/**
 * @brief Generates visualization markers for a planned path: a colored line list plus one arrow per
 *        trajectory state (ns `planned_path_orientations`).
 *
 * Follows the sensor positions. When @p map resolves ground heights (Map::getGroundHeight), a
 * second polyline in `planned_path_ground` drapes the path onto the ground, omitting segments
 * whose endpoints have no ground height.
 *
 * @param path_edges The sequence of edges representing the planned path.
 * @param markers Output marker container.
 * @param map Optional map used to drape the path onto the ground; null skips the
 *        ground polyline entirely.
 */
void getPlannedPathVisualizationMarkers(const std::vector<Edge*>& path_edges, VisualizationMarkers* markers, const Map* map = nullptr);

}  // namespace open_apero
