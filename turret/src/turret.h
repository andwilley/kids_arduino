#ifndef TURRET
#define TURRET

#include <Adafruit_AMG88xx.h>
#ifdef GMOCK_FLAG
#include <cstddef>
#endif

void turretSetup();
void turretLoop();

template <typename T> struct Point {
  T x;
  T y;

  Point operator+(Point other) { return {.x = x + other.x, .y = y + other.y}; }
  Point operator-(Point other) { return {.x = x - other.x, .y = y - other.y}; }
  Point operator*(const float rhs) { return {.x = x * rhs, .y = y * rhs}; }
  Point operator/(const float divisor) {
    return {.x = x / divisor, .y = y / divisor};
  }
  void operator+=(Point other) {
    x += other.x;
    y += other.y;
  }
  void operator-=(Point other) {
    x -= other.x;
    y -= other.y;
  }

  bool operator==(Point other) { return x == other.x && y == other.y; }
  bool operator!=(Point other) { return x != other.x && y != other.y; }
};

Point<float> FindHeatCenter(float *temps, size_t size);

#endif // TURRET
