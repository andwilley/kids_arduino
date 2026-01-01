#ifndef TURRET_GRID_H_
#define TURRET_GRID_H_

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
  Point<C> *begin() const { return points_; }
  Point<C> *end() const { return points_ + count_; }

private:
  Point<C> points_[8];
  uint8_t count_ = 0;

  static constexpr Point<int> dirs[8] = {{0, -1}, {1, -1}, {1, 0},  {1, 1},
                                         {0, 1},  {-1, 1}, {-1, 0}, {-1, -1}};
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

  T &At(const Point<C> &p) { return data_[IndexOf(p)]; }

  const T &At(const Point<C> &p) const { return data_[IndexOf(p)]; }

  bool InBounds(const Point<C> &p) const {
    return static_cast<size_t>(p.x) < W && static_cast<size_t>(p.y) < H;
  }

  // Caller must verify index is in bounds.
  static Point<C> PointFrom(size_t index) {
    return {
      .x = static_cast<C>(index % kCols), .y = static_cast<C>(index / kCols),
    }
  }

  // Caller must verify point is in bounds.
  static size_t IndexOf(const Point<C> &p) {
    return static_cast<size_t>(p.y) * kCols + static_cast<size_t>(p.x);
  }


  T *data() { return data_; }
  const T *data() const { return data_; }

  NeighborList<C> Neighbors(Point<C> center);

  void PrintToSerial() {
    for (int i = 1; i <= kSize; ++i) {
      float val = data_[i - 1];
      Serial.print(val);
      Serial.print(", ");
      if (i % 8 == 0) {
        Serial.println();
      }
    }
    Serial.println();
  }

private:
  T data_[kSize];
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
