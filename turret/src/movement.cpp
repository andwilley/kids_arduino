#include "movement.h"

#include <Arduino.h>

namespace turret {

void Movement::Track(uint64_t dt) {
  if (is_tracking_) {
    heat_sensor_.Read();

    geometry::Point<float> current_error =
        heat_sensor_.FindHeatCenter(geometry::Point<float>{
            .x = yaw_pid_.GetLastError(), .y = pitch_pid_.GetLastError()});
    math::RotateError90Cw(current_error);

    float yaw_output = yaw_pid_.Compute(current_error.x, dt);
    float pitch_output = pitch_pid_.Compute(current_error.y, dt);

    yaw_servo_.MapSetSpeed(yaw_output, -kControlRange, kControlRange);
    pitch_servo_.MapSetSpeed(pitch_output, -kControlRange, kControlRange);
  }
}

void Movement::Update(uint64_t current_micros) {
  pitch_servo_.Update(current_micros);
}

void Movement::Fire() {
  roll_servo_.SetSpeed(servos::kStopSpeed + kRollSpeed);
  delay(kRollPrecision);
  roll_servo_.SetSpeed(servos::kStopSpeed);
}

} // namespace turret
