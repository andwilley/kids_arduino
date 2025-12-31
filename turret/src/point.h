#ifndef TURRET_POINT_H_
#define TURRET_POINT_H_

template <typename T> struct Point {
  T x;
  T y;

  Point operator+(const Point &rhs) const {
    return {.x = x + rhs.x, .y = y + rhs.y};
  }
  Point operator-(const Point &rhs) const {
    return {.x = x - rhs.x, .y = y - rhs.y};
  }
  Point operator*(const T &rhs) const { return {.x = x * rhs, .y = y * rhs}; }
  Point operator/(const T &rhs) const { return {.x = x / rhs, .y = y / rhs}; }
  void operator+=(const Point &rhs) {
    x += rhs.x;
    y += rhs.y;
  }
  void operator-=(const Point &rhs) {
    x -= rhs.x;
    y -= rhs.y;
  }

  bool operator==(const Point &rhs) const { return x == rhs.x && y == rhs.y; }
  bool operator!=(const Point &rhs) const { return x != rhs.x || y != rhs.y; }

  Point ApplyTolerance(int xTolerance, int yTolerance) const;
};

template <typename T>
Point<T> Point<T>::ApplyTolerance(int xTolerance, int yTolerance) const {
  if (abs(x) > xTolerance && abs(y) > yTolerance) {
    return *this;
  }

  T adjusted_x = x;
  T adjusted_y = y;

  if (abs(x) < xTolerance) {
    adjusted_x = T(0);
  }
  if (abs(y) < yTolerance) {
    adjusted_y = T(0);
  }
  return {
      .x = adjusted_x,
      .y = adjusted_y,
  };
}

#endif // TURRET_POINT_H_
