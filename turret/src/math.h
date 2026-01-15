#ifndef MATH_H_
#define MATH_H_

#include "point.h"
#include <cstddef>

namespace math {

template <typename T> T Clamp(T val, T min, T max) {
  if (val < min) {
    return min;
  }
  if (val > max) {
    return max;
  }
  return val;
}

// The sensor is rotated 90 degrees clockwise. This function transforms the
// error coordinates to match the turret's orientation. A point(x, y) in the
// sensor's frame becomes (y, -x) in the turret's frame.
template <typename T> void RotateError90Cw(geometry::Point<T> &error) {
  float temp_x = error.x;
  error.x = error.y;
  error.y = -temp_x;
}

} // namespace math

#endif // MATH_H_
