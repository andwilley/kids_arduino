#ifndef TURRET_CONTINUOUS_SERVO_H_
#define TURRET_CONTINUOUS_SERVO_H_

#include "servo_constants.h"
#include "turret_math.h"
#include <ESP32Servo.h>

namespace turret {

class ContinuousServo {
public:
  ContinuousServo(int pin, int dead_band = 0)
      : pin_(pin), dead_band_(dead_band) {}

  ContinuousServo(const ContinuousServo &s) = delete;
  ContinuousServo &operator=(const ContinuousServo &s) = delete;

  void Init() { servo_.attach(pin_); }

  void SetSpeed(int input_value);

  void MapSetSpeed(float value, float from_min, float from_max) {
    float proportion = (value - from_min) / (from_max - from_min);
    SetSpeed(kMaxNegativeSpeed +
             (proportion * (kMaxPositiveSpeed - kMaxNegativeSpeed)));
  }

private:
  int pin_;
  int dead_band_;

  int current_value_ = kStopSpeed;
  Servo servo_;

  void Write(int value) {
    current_value_ = Clamp(value, kMaxNegativeSpeed, kMaxPositiveSpeed);
    servo_.write(current_value_);
  }
};

} // namespace turret

#endif // TURRET_CONTINUOUS_SERVO_H_
