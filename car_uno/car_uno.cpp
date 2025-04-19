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
constexpr int kDirectionPinA = 12;
constexpr int kPwmPinA = 3;
constexpr int kBrakePinA = 9;

// Motor B
constexpr int kDirectionPinB = 13;
constexpr int kPwmPinB = 11;
constexpr int kBrakePinB = 8;

namespace car {

int a_speed(255);
int b_speed(255);

void handleCommand(int command) {
  if ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT))
    return;
  Serial.print("Got command: ");
  Serial.println(command);

  switch (command) {
  case kUp:
    a_speed = 100;
    b_speed = 100;
    return;
  // fall through
  case kDown:
    a_speed = 0;
    b_speed = 0;
    return;
  default:
    Serial.println("command no recognized");
    return;
  }
}

void setup() {
  Serial.begin(9600);

  // printActiveIRProtocols(&Serial);

  pinMode(kDirectionPinA, OUTPUT);
  pinMode(kPwmPinA, OUTPUT);
  pinMode(kBrakePinA, OUTPUT);

  pinMode(kDirectionPinB, OUTPUT);
  pinMode(kPwmPinB, OUTPUT);
  pinMode(kBrakePinB, OUTPUT);

  IrReceiver.begin(kIrReceivePin, ENABLE_LED_FEEDBACK);

  // TODO: move this to loop and control with remote
  // set direction
  digitalWrite(kDirectionPinA, LOW);
  digitalWrite(kDirectionPinB, LOW);

  // no brakes
  digitalWrite(kBrakePinA, LOW);
  digitalWrite(kBrakePinB, LOW);
}

void loop() {
  if (IrReceiver.decode()) {
    int command = IrReceiver.decodedIRData.command;
    IrReceiver.resume();
    handleCommand(command);
  }

  // go!
  Serial.print("A speed: ");
  Serial.println(a_speed);
  Serial.print("B speed: ");
  Serial.println(b_speed);
  analogWrite(kPwmPinA, a_speed);
  analogWrite(kPwmPinB, b_speed);

  delay(100);
}

} // namespace car
