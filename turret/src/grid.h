#ifndef GRID_H_
#define GRID_H_

#include "logger.h"
#include "point.h"
#include <stddef.h>

namespace grid {

template <typename C> class NeighborList {
public:
  void push_back(const geometry::Point<C> p) {
    if (count_ < 8) {
      points_[count_] = p;
      ++count_;
    }
  }

  // Iterator support for range-based loops
  const geometry::Point<C> *begin() const { return points_; }
  const geometry::Point<C> *end() const { return points_ + count_; }

private:
  geometry::Point<C> points_[8];
  uint8_t count_ = 0;
};

template <typename T, typename C, size_t W, size_t H> class Grid2d {
public:
  static constexpr size_t kRows = H;
  static constexpr size_t kCols = W;
  static constexpr size_t kSize = W * H;
  static constexpr geometry::Point<float> kMidPt = {.x = (kCols - 1) / 2.0f,
                                                    .y = (kRows - 1) / 2.0f};

  Grid2d() {}

  // Caller must verify point is in bounds.
  const T &At(const geometry::Point<C> &p) const { return data_[IndexOf(p)]; }

  bool InBounds(const geometry::Point<C> &p) const {
    return static_cast<size_t>(p.x) < W && static_cast<size_t>(p.y) < H;
  }

  // Caller must verify index is in bounds.
  static geometry::Point<C> PointFrom(size_t index) {
    return {
        .x = static_cast<C>(index % kCols),
        .y = static_cast<C>(index / kCols),
    };
  }

  // Caller must verify point is in bounds.
  static size_t IndexOf(const geometry::Point<C> &p) {
    return static_cast<size_t>(p.y) * kCols + static_cast<size_t>(p.x);
  }

  geometry::Point<C> MaxPoint() {
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

  NeighborList<C> Neighbors(geometry::Point<C> center);

  void Print() const;

private:
  T data_[kSize]{};

  const geometry::Point<int> dirs[8] = {{0, -1}, {1, -1}, {1, 0},  {1, 1},
                                        {0, 1},  {-1, 1}, {-1, 0}, {-1, -1}};
};

template <typename T, typename C, size_t W, size_t H>
void Grid2d<T, C, W, H>::Print() const {
  char line_buffer[logger::Log.kEffectiveLogLength];
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
    logger::Log.Log(logger::kInfo, "[%s]", line_buffer);
  }
}

template <typename T, typename C, size_t W, size_t H>
NeighborList<C> Grid2d<T, C, W, H>::Neighbors(geometry::Point<C> center) {
  NeighborList<C> list;
  for (const geometry::Point<int> dir : dirs) {
    geometry::Point<C> next = center + dir;
    if (InBounds(next)) {
      list.push_back(next);
    }
  }
  return list;
}

} // namespace grid

#endif // GRID_H_
