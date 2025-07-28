#ifndef TURRET
#define TURRET

#include <Adafruit_AMG88xx.h>

void turretSetup();
void turretLoop();

template <typename T> struct Point {
  T x;
  T y;

  Point operator+(Point other) { return {.x = x + other.x, .y = y + other.y}; }
  void operator+=(Point other) {
    x += other.x;
    y += other.y;
  }

  bool operator==(Point other) { return x == other.x && y == other.y; }
};

Point<float> FindHeatCenter(float *temps, size_t size);

#endif // TURRET
