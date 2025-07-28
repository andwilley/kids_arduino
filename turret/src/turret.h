#ifndef TURRET
#define TURRET

#include <Adafruit_AMG88xx.h>

void turretSetup();
void turretLoop();

struct Point {
  float x;
  float y;

  Point operator+(Point other) { return {.x = x + other.x, .y = y + other.y}; }

  bool operator==(Point other) { return x == other.x && y == other.y; }
};

const Point kNoPoint = {.x = -1, .y = -1};

Point FindHeatCenter(float *temps);

#endif // TURRET
