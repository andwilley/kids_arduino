#ifndef TURRET_SERVO_CONSTANTS_H_
#define TURRET_SERVO_CONSTANTS_H_

#include <ESP32Servo.h>

namespace turret {

constexpr int kStopSpeed = 90;
constexpr int kMaxNegativeSpeed = 0;
constexpr int kMaxPositiveSpeed = 180;

} // namespace turret

#endif // TURRET_SERVO_CONSTANTS_H_
