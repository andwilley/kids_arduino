#include "continuous_servo.h"
#include "servo_constants.h"

namespace servos {

void ContinuousServo::SetSpeed(int input_value) {
  if (input_value == kStopSpeed) {
    Write(kStopSpeed);
    return;
  }

  int signed_dead_band = dead_band_;
  if (input_value < kStopSpeed) {
    signed_dead_band = -1 * dead_band_;
  }

  Write(input_value + signed_dead_band);
}

} // namespace servos
