#ifndef TURRET_FIXED_RANGE_SERVO
#define TURRET_FIXED_RANGE_SERVO

#include "servo_constants.h"
#include "turret_math.h"
#include <ESP32Servo.h>

using turret_math::Clamp;

namespace turret_fixed_range_servo {

class FixedRangeServo {
public:
  FixedRangeServo(int pin, int initial_value, int min = 0, int max = 180)
      : _pin(pin), _current_value(initial_value), _min(min), _max(max) {}

  FixedRangeServo(const FixedRangeServo &s) = delete;
  FixedRangeServo &operator=(const FixedRangeServo &s) = delete;

  void Init() { _servo.attach(_pin); }

  // Set the servo speed. Continuous movement is simulated by constant small
  // movements. To effect this change, Move must be called inside loop().
  void SetSpeed(int speed);

  // Pass in micros() from the main loop
  void Update(uint64_t micros);

private:
  int _pin;
  int _current_value;
  int _min;
  int _max;
  Servo _servo;
  float _velocity = 0.0;
};

void FixedRangeServo::SetSpeed(int speed) {
  // take the requested speed [0, 180] with 90 as stopped, like a continuous
  // servo. Translate this speed to a combination of movement direction and gaps
  // between mvmt.
  pitchSpeed = Clamp(speed, 0, 180);
  int norm = Clamp(abs(speed - turret_servo_constants::kStopSpeed), 0,
                   turret_servo_constants::kStopSpeed);
  // The old calculation resulted in a step size of [299, 300], which is
  // far too slow. This new calculation maps the speed to a step size
  // from kMaxPitchStep down to 0.
  pitchStepSize = round(kMaxPitchStep * (1.0 - (float(norm) / 90.0)));
}

// Check if the pitch servo should be moving, and move it
void Update(uint64_t micros) {
  if (pitchSpeed == turret_servo_constants::kStopSpeed) {
    return;
  }
  if (millis() > lastPitchMoveMs + pitchStepSize) {
    lastPitchMoveMs = micros;
    if (pitchSpeed > turret_servo_constants::kStopSpeed) {
      pitchServoVal++;
    } else {
      pitchServoVal--;
    }
    pitchServoVal = Clamp(pitchServoVal, kPitchMin, kPitchMax);
    pitchServo.write(pitchServoVal);
  }
}

} // namespace turret_fixed_range_servo

#endif // TURRET_FIXED_RANGE_SERVO
