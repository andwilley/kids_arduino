#ifndef TURRET_GRID_H_
#define TURRET_GRID_H_

#include "point.h"
#include <stddef.h>

namespace turret_grid {

// TODO: evaluate if these are needed first:
// - index to point
// - point to index
// - get neighbors (from an index? from a point?)
template <typename T, size_t W, size_t H> class Grid2d {
public:
  static constexpr size_t kRows = H;
  static constexpr size_t kCols = W;
  static constexpr size_t kSize = W * H;

  Grid2d() {}

  template <typename C> T &At(const Point<C> &p) {
    return data_[static_cast<size_t>(p.y) * W + static_cast<size_t>(p.x)];
  }

  template <typename C> const T &At(const Point<C> &p) const {
    return data_[static_cast<size_t>(p.y) * W + static_cast<size_t>(p.x)];
  }

  template <typename C> bool InBounds(const Point<C> &p) const {
    return static_cast<size_t>(p.x) < W && static_cast<size_t>(p.y) < H;
  }

  bool Load(const T *source, size_t count) {
    if (count != kSize) {
      return false;
    }
    for (size_t i = 0; i < kSize; ++i) {
      data_[i] = source[i];
    }
    return true;
  }

  T *data() { return data_; }
  const T *data() const { return data_; }

private:
  T data_[kSize];
};

} // namespace turret_grid

#endif // TURRET_GRID_H_
