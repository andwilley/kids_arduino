#ifndef TURRET_GRID_H_
#define TURRET_GRID_H_

#include "logger.h"
#include "point.h"
#include <stddef.h>

namespace turret {

template <typename C> class NeighborList {
public:
  void push_back(Point<C> p) {
    if (count_ < 8) {
      points_[count_] = p;
      ++count_;
    }
  }

  // Iterator support for range-based loops
  const Point<C> *begin() const { return points_; }
  const Point<C> *end() const { return points_ + count_; }

private:
  Point<C> points_[8];
  uint8_t count_ = 0;
};

// TODO: evaluate if these are needed first:
// - index to point
// - point to index
// - get neighbors (from an index? from a point?)
template <typename T, typename C, size_t W, size_t H> class Grid2d {
public:
  static constexpr size_t kRows = H;
  static constexpr size_t kCols = W;
  static constexpr size_t kSize = W * H;
  static constexpr float kRowMidPt = (kRows - 1) / 2.0f;
  static constexpr float kColMidPt = (kCols - 1) / 2.0f;

  Grid2d() {}

  // Caller must verify point is in bounds.
  T &At(const Point<C> &p) { return data_[IndexOf(p)]; }

  // Caller must verify point is in bounds.
  const T &At(const Point<C> &p) const { return data_[IndexOf(p)]; }

  bool InBounds(const Point<C> &p) const {
    return static_cast<size_t>(p.x) < W && static_cast<size_t>(p.y) < H;
  }

  // Caller must verify index is in bounds.
  static Point<C> PointFrom(size_t index) {
    return {
        .x = static_cast<C>(index % kCols),
        .y = static_cast<C>(index / kCols),
    };
  }

  // Caller must verify point is in bounds.
  static size_t IndexOf(const Point<C> &p) {
    return static_cast<size_t>(p.y) * kCols + static_cast<size_t>(p.x);
  }

  Point<C> MaxPoint() {
    size_t max_i = 0;
    T max = data_[max_i];
    for (int i = 1; i < kSize; ++i) {
      if (data_[i] > max) {
        max = data_[i];
        max_i = i;
      }
    }
    return PointFrom(max_i);
  }

  T *data() { return data_; }
  const T *data() const { return data_; }

  NeighborList<C> Neighbors(Point<C> center);

  void Print() {
    char line_buffer[Log.kEffectiveLogLength];
    for (size_t h = 0; h < kRows; ++h) {
      size_t offset = 0;
      for (size_t w = 0; w < kCols; ++w) {
        size_t remaining = sizeof(line_buffer) - offset;
        if (remaining < 8) {
          break;
        }
        int written = snprintf(line_buffer + offset, remaining, "%.1f, ",
                               data_[h * kCols + w]);
        if (written < 0 || (size_t)written >= remaining) {
          break;
        }

        offset += written;
      }
      Log.Log(kInfo, "[%s]", line_buffer);
    }
  }

private:
  T data_[kSize]{};

  const Point<int> dirs[8] = {{0, -1}, {1, -1}, {1, 0},  {1, 1},
                              {0, 1},  {-1, 1}, {-1, 0}, {-1, -1}};
};

template <typename T, typename C, size_t W, size_t H>
NeighborList<C> Grid2d<T, C, W, H>::Neighbors(Point<C> center) {
  NeighborList<C> list;
  for (const Point<int> dir : dirs) {
    Point<C> next = center + dir;
    if (InBounds(next)) {
      list.push_back(next);
    }
  }
  return list;
}

} // namespace turret

#endif // TURRET_GRID_H_
