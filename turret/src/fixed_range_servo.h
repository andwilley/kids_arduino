#ifndef TURRET_FIXED_RANGE_SERVO_H_
#define TURRET_FIXED_RANGE_SERVO_H_

#include "servo_constants.h"
#include "turret_math.h"
#include <ESP32Servo.h>

namespace turret {

constexpr double kMaxDeltaT = 0.1;

class FixedRangeServo {
public:
  FixedRangeServo(int pin, float initial_angle, int min_angle = 0,
                  int max_angle = 180, float max_degrees_per_second = 60.0)
      : pin_(pin), current_angle_(initial_angle), min_angle_(min_angle),
        max_angle_(max_angle), max_degrees_per_second_(max_degrees_per_second) {
  }

  FixedRangeServo(const FixedRangeServo &s) = delete;
  FixedRangeServo &operator=(const FixedRangeServo &s) = delete;

  void Init(uint64_t current_micros) {
    servo_.attach(pin_);
    Write(current_angle_);
    last_update_micros_ = current_micros;
  }

  // Set the servo speed. Continuous movement is simulated by constant small
  // movements. To effect this change, ::Update must be called inside loop().
  void SetSpeed(float input_value);

  // Pass in micros() from the main loop
  void Update(uint64_t micros);

  void MapSetSpeed(float value, float from_min, float from_max) {
    float proportion = (value - from_min) / (from_max - from_min);
    SetSpeed(kMaxNegativeSpeed +
             (proportion * (kMaxPositiveSpeed - kMaxNegativeSpeed)));
  }

  int MinAngle() { return min_angle_; }
  int MaxAngle() { return max_angle_; }

private:
  int pin_;
  float current_angle_;
  int min_angle_;
  int max_angle_;
  float max_degrees_per_second_;

  Servo servo_;
  uint64_t last_update_micros_ = 0;
  float velocity_ = 0.0;

  void Write(float angle) {
    current_angle_ =
        Clamp(static_cast<int>(round(angle)), min_angle_, max_angle_);
    servo_.write(current_angle_);
  }

  static constexpr double kMicrosPerSecond = 1000000.0;
};

void FixedRangeServo::SetSpeed(float input_value) {
  float clamped_input =
      Clamp(input_value, static_cast<float>(kMaxNegativeSpeed),
            static_cast<float>(kMaxPositiveSpeed));

  // Map [0..180] to [-MaxSpeed..+MaxSpeed]
  // (Input - 90) / 90 gives range -1.0 to 1.0
  float normalized_speed = (clamped_input - static_cast<float>(kStopSpeed)) /
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

#endif // TURRET_FIXED_RANGE_SERVO_H_
