#ifndef TURRET_MATH_H_
#define TURRET_MATH_H_

#include <cstddef>
namespace turret_math {

template <typename T, size_t S>
using Grid1d = T[S];

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
