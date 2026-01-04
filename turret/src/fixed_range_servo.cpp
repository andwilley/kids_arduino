#include "fixed_range_servo.h"
#include "servo_constants.h"
#include "turret_math.h"
#include <ESP32Servo.h>

namespace turret {

void FixedRangeServo::SetSpeed(float input_value) {
  const float clamped_input =
      Clamp(input_value, static_cast<float>(kMaxNegativeSpeed),
            static_cast<float>(kMaxPositiveSpeed));

  // Map [0..180] to [-MaxSpeed..+MaxSpeed]
  // (Input - 90) / 90 gives range -1.0 to 1.0
  const float normalized_speed = (clamped_input - static_cast<float>(kStopSpeed)) /
                           static_cast<float>(kStopSpeed);

  velocity_ = normalized_speed * max_degrees_per_second_;
}

// Check if the pitch servo should be moving, and move it
void FixedRangeServo::Update(uint64_t current_micros) {
  // Time delta (seconds)
  double dt = (current_micros - last_update_micros_) / kMicrosPerSecond;
  last_update_micros_ = current_micros;

  if (dt > kMaxDeltaT) {
    dt = 0.0;
  }

  if (velocity_ != 0.0) {
    Write(current_angle_ + velocity_ * dt);
  }
}

} // namespace turret
