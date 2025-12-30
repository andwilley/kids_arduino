#ifndef TURRET_CONTINUOUS_SERVO
#define TURRET_CONTINUOUS_SERVO

#include "servo_constants.h"
#include <ESP32Servo.h>

namespace turret_continuous_servo {

class ContinuousServo {
public:
  ContinuousServo(int pin, int dead_band = 0)
      : _pin(pin), _dead_band(dead_band) {}

  ContinuousServo(const ContinuousServo &s) = delete;
  ContinuousServo &operator=(const ContinuousServo &s) = delete;

  void Init() { _servo.attach(_pin); }

  void SetSpeed(int speed);

private:
  int _pin;
  int _dead_band;
  int _current_value = turret_servo_constants::kStopSpeed;
  Servo _servo;
};

void ContinuousServo::SetSpeed(int speed) {
  if (speed == turret_servo_constants::kStopSpeed) {
    _current_value = turret_servo_constants::kStopSpeed;
    _servo.write(_current_value);
    return;
  }

  int signedDeadBand = _dead_band;
  if (speed < turret_servo_constants::kStopSpeed) {
    signedDeadBand = -1 * _dead_band;
  }

  _current_value = speed + _dead_band;
  _servo.write(_current_value);
}

} // namespace turret_continuous_servo
