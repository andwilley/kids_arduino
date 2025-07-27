#include <AUnit.h>
#include "turret.h"

test(turret_setup) {
  // This is a basic test case.
  // We can add assertions here later.
  assertTrue(true);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  aunit::TestRunner::run();
}

void loop() {
}
