#include "open_apero_core/visualization/colormap.h"

#include <cmath>

namespace open_apero {

Color getColorFromColormap(float v, float vmin, float vmax, float alpha, tinycolormap::ColormapType colormap) {
  if (v < vmin) {
    v = vmin;
  }

  if (v > vmax) {
    v = vmax;
  }

  tinycolormap::Color color = tinycolormap::GetColor((v - vmin) / (vmax - vmin), colormap);

  Color c;
  c.r = color.r();
  c.g = color.g();
  c.b = color.b();
  c.a = alpha;
  return c;
}

Color getTrajectoryColor(float t, float tmin, float tmax, float alpha) {
  float range = tmax - tmin;

  if (range > 0) {
    t = tmin + fmod(t - tmin, range);
    if (t < tmin) {
      t += range;  // Ensure t is within the range after wrapping
    }
  }

  tinycolormap::Color color = tinycolormap::GetParulaColor((t - tmin) / (tmax - tmin));

  Color c;
  c.r = color.r();
  c.g = color.g();
  c.b = color.b();
  c.a = alpha;
  return c;
}

}  // namespace open_apero
