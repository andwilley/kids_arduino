#ifndef TURRET_MATH
#define TURRET_MATH

namespace turret_math {

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

#endif // TURRET_MATH
