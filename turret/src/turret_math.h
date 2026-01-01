#ifndef TURRET_MATH_H_
#define TURRET_MATH_H_

#include <cstddef>
namespace turret {

template <typename T> T Clamp(T val, T min, T max) {
  if (val < min) {
    return min;
  }
  if (val > max) {
    return max;
  }
  return val;
}

} // namespace turret_math

#endif // TURRET_MATH_H_
