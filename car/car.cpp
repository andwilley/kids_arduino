#include "car.h"
#include <Arduino.h>
#include <IRremote.hpp>

// Motor A
constexpr int kAFwdPwm = 5;
constexpr int kARevPwm = 9;

namespace car {

void setup() {
  Serial.begin(9600);
  while (!Serial);
  pinMode(kAFwdPwm, OUTPUT);
  pinMode(kARevPwm, OUTPUT);

  Serial.println("Done setting up!");
}

void loop() {
  Serial.println("Going!");

  // go!
  analogWrite(kAFwdPwm, 255);
  digitalWrite(kARevPwm, LOW);

  delay(2000);
}

} // namespace car
