#define IR_USE_AVR_TIMER1

#include "car_uno.h"
#include <Arduino.h>
#include <IRremote.hpp>

// IR
#define DECODE_NEC // transmission type
constexpr int kIrReceivePin = 2;

constexpr int kLeft(0x8);
constexpr int kRight(0x5A);
constexpr int kUp(0x18);
constexpr int kDown(0x52);
constexpr int kOk(0x1C);
constexpr int kCmd1(0x45);
constexpr int kCmd2(0x46);
constexpr int kCmd3(0x47);
constexpr int kCmd4(0x44);
constexpr int kCmd5(0x40);
constexpr int kCmd6(0x43);
constexpr int kCmd7(0x7);
constexpr int kCmd8(0x15);
constexpr int kCmd9(0x9);
constexpr int kCmd0(0x19);
constexpr int kStar(0x16);
constexpr int kHashtag(0xD);

// Motor A
constexpr int kDirectionPinA(12);
constexpr int kPwmPinA(3);
constexpr int kBrakePinA(9);

// Motor B
constexpr int kBadPin(13);
constexpr int kDirectionPinB(10);
constexpr int kPwmPinB(11);
constexpr int kBrakePinB(8);

constexpr int kMaxSpeed(255);
constexpr int kMinSpeed(-255);
constexpr int kSpeedStep(75);

namespace car {

int a_speed(0);
int b_speed(0);
int invert(false);

int Clamp(int v, int lo, int hi) { return max(min(v, hi), lo); }

void ChangeSpeedA(int step) {
  a_speed = Clamp(a_speed + step, kMinSpeed, kMaxSpeed);
}

void ChangeSpeedB(int step) {
  b_speed = Clamp(b_speed + step, kMinSpeed, kMaxSpeed);
}

void HandleCommand(int command) {
  if ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT))
    return;
  Serial.print("Got command: ");
  Serial.println(command);

  switch (command) {
  case kUp:
    Serial.println("up");
    // Match the high speed
    if (a_speed != b_speed) {
      int hi = max(a_speed, b_speed);
      a_speed = hi;
      b_speed = hi;
      return;
    }
    ChangeSpeedA(kSpeedStep);
    ChangeSpeedB(kSpeedStep);
    return;
  // fall through
  case kDown:
    Serial.println("down");
    // Match the low speed if mismatch
    if (a_speed != b_speed) {
      int lo = min(a_speed, b_speed);
      a_speed = lo;
      b_speed = lo;
      return;
    }
    ChangeSpeedA(-kSpeedStep);
    ChangeSpeedB(-kSpeedStep);
    return;
  case kOk:
    Serial.println("down");
    a_speed = 0;
    b_speed = 0;
    return;
  case kLeft:
    Serial.println("left");
    if (invert) {
      ChangeSpeedA(-kSpeedStep);
    } else {
      ChangeSpeedB(-kSpeedStep);
    }
    return;
  case kRight:
    Serial.println("right");
    if (invert) {
      ChangeSpeedB(-kSpeedStep);
    } else {
      ChangeSpeedA(-kSpeedStep);
    }
    return;
  case kCmd5:
    Serial.println("5");
    invert = !invert;
    a_speed = 0;
    b_speed = 0;
    return;
  default:
    Serial.println("default");
    return;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(kDirectionPinA, OUTPUT);
  pinMode(kPwmPinA, OUTPUT);
  pinMode(kBrakePinA, OUTPUT);

  // pinMode(kBadPin, INPUT);
  pinMode(kDirectionPinB, OUTPUT);
  pinMode(kPwmPinB, OUTPUT);
  pinMode(kBrakePinB, OUTPUT);

  IrReceiver.begin(kIrReceivePin, ENABLE_LED_FEEDBACK);

  digitalWrite(kDirectionPinA, LOW);
  digitalWrite(kDirectionPinB, LOW);

  // no brakes
  digitalWrite(kBrakePinA, LOW);
  digitalWrite(kBrakePinB, LOW);
}

// calculate output speed with direction
int SetSpeeds() {
  int a = invert ? -a_speed : a_speed;
  int b = invert ? -b_speed : b_speed;
  digitalWrite(kDirectionPinA, a < 0 ? HIGH : LOW);
  digitalWrite(kDirectionPinB, b < 0 ? HIGH : LOW);
  analogWrite(kPwmPinA, abs(a));
  analogWrite(kPwmPinB, abs(b));
}

void Debug(bool on) {
  if (on) {
    Serial.print("invert: ");
    Serial.println(invert);
    Serial.print("A speed: ");
    Serial.println(a_speed);
    Serial.print("B speed: ");
    Serial.println(b_speed);
    delay(1000);
  }
}

void loop() {
  if (IrReceiver.decode()) {
    int command = IrReceiver.decodedIRData.command;
    IrReceiver.resume();
    HandleCommand(command);
  }

  SetSpeeds();
  Debug(true);
}

} // namespace car
