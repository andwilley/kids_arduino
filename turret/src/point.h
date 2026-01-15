#ifndef POINT_H_
#define POINT_H_

namespace geometry {

template <typename T> struct Point {
  T x;
  T y;

  Point operator+(const Point &rhs) const;
  Point operator+(const T &rhs) const;
  Point operator-(const Point &rhs) const;
  Point operator-(const T &rhs) const;
  Point operator*(const T &rhs) const;
  Point operator*(const Point &rhs) const;
  Point operator/(const T &rhs) const;

  void operator+=(const Point &rhs);
  void operator-=(const Point &rhs);

  bool operator==(const Point &rhs) const;
  bool operator!=(const Point &rhs) const;
};

template <typename T> Point<T> Point<T>::operator+(const Point &rhs) const {
  return {.x = x + rhs.x, .y = y + rhs.y};
}
template <typename T> Point<T> Point<T>::operator+(const T &rhs) const {
  return {.x = x + rhs, .y = y + rhs};
}
template <typename T> Point<T> Point<T>::operator-(const Point &rhs) const {
  return {.x = x - rhs.x, .y = y - rhs.y};
}
template <typename T> Point<T> Point<T>::operator-(const T &rhs) const {
  return {.x = x - rhs, .y = y - rhs};
}
template <typename T> Point<T> Point<T>::operator*(const T &rhs) const {
  return {.x = x * rhs, .y = y * rhs};
}
template <typename T> Point<T> Point<T>::operator*(const Point &rhs) const {
  return {.x = x * rhs.x, .y = y * rhs.y};
}
template <typename T> Point<T> Point<T>::operator/(const T &rhs) const {
  return {.x = x / rhs, .y = y / rhs};
}
template <typename T> void Point<T>::operator+=(const Point<T> &rhs) {
  x += rhs.x;
  y += rhs.y;
}
template <typename T> void Point<T>::operator-=(const Point<T> &rhs) {
  x -= rhs.x;
  y -= rhs.y;
}

template <typename T> bool Point<T>::operator==(const Point<T> &rhs) const {
  return x == rhs.x && y == rhs.y;
}
template <typename T> bool Point<T>::operator!=(const Point<T> &rhs) const {
  return !this == rhs;
}

} // namespace geometry

#endif // POINT_H_
