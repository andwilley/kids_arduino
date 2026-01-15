#ifndef SERVO_CONSTANTS_H_
#define SERVO_CONSTANTS_H_

#include <ESP32Servo.h>

namespace servos {

constexpr int kStopSpeed = 90;
constexpr int kMaxNegativeSpeed = 0;
constexpr int kMaxPositiveSpeed = 180;

} // namespace servos

#endif // SERVO_CONSTANTS_H_
