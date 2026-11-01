#pragma once

/**
 * @file colormap.h
 * @brief Colormap-based marker colors for scalar values and trajectory progress.
 */

#include "open_apero_core/third_party/tinycolormap.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

/**
 * @brief Map a scalar value to a color using a colormap.
 * @param v The value to map.
 * @param vmin Minimum value of the range.
 * @param vmax Maximum value of the range.
 * @param alpha Alpha (opacity) value for the resulting color.
 * @param colormap The colormap type to use (default: Heat).
 * @return The mapped color.
 */
Color getColorFromColormap(float v, float vmin, float vmax, float alpha,
                           tinycolormap::ColormapType colormap = tinycolormap::ColormapType::Heat);

/**
 * @brief Get a color for trajectory visualization based on a normalized parameter.
 *
 * @p t wraps modulo [tmin, tmax), so t == tmax is colored like tmin. Precondition: tmax > tmin.
 * @param t The parameter value.
 * @param tmin Minimum parameter value.
 * @param tmax Maximum parameter value.
 * @param alpha Alpha (opacity) value.
 * @return The trajectory color.
 */
Color getTrajectoryColor(float t, float tmin, float tmax, float alpha);

}  // namespace open_apero
