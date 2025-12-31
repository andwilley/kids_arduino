#ifndef TURRET_SERVO_CONSTANTS
#define TURRET_SERVO_CONSTANTS

#include <ESP32Servo.h>

namespace turret_servo_constants {

constexpr int kStopSpeed = 90;
constexpr int kMaxNegativeSpeed = 0;
constexpr int kMaxPositiveSpeed = 180;

} // namespace turret_servo_constants

#endif // TURRET_SERVO_CONSTANTS
