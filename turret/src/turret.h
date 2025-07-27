#ifndef TURRET
#define TURRET

#include <Adafruit_AMG88xx.h>

void turretSetup();
void turretLoop();

struct Point {
  float x;
  float y;
};

Point FindHeatCenter(float* temps);

#endif
