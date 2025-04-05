#include "car_uno.h"
#include <Arduino.h>
#include <IRremote.hpp>

// Motor A
constexpr int kDirectionPinA = 12;
constexpr int kPwmPinA = 3;
constexpr int kBrakePinA = 9;

constexpr int kDirectionPinB = 13;
constexpr int kPwmPinB = 11;
constexpr int kBrakePinB = 8;

namespace car {

void setup() {
  Serial.begin(9600);
  while (!Serial);
  pinMode(kDirectionPinA, OUTPUT);
  pinMode(kPwmPinA, OUTPUT);
  pinMode(kBrakePinA, OUTPUT);

  pinMode(kDirectionPinB, OUTPUT);
  pinMode(kPwmPinB, OUTPUT);
  pinMode(kBrakePinB, OUTPUT);

  Serial.println("Done setting up!");
}

void loop() {
  Serial.println("Going!");

  // set direction
  digitalWrite(kDirectionPinA, LOW);
  digitalWrite(kDirectionPinB, LOW);

  // no brakes
  digitalWrite(kBrakePinA, LOW);
  digitalWrite(kBrakePinB, LOW);

  // go!
  analogWrite(kPwmPinA, 150);
  analogWrite(kPwmPinB, 150);

  delay(2000);
}

} // namespace car
