#ifndef TURRET_MOVEMENT_H_
#define TURRET_MOVEMENT_H_

#include "continuous_servo.h"
#include "fixed_range_servo.h"
#include "heat_sensor.h"
#include "pid.h"
#include "servo_constants.h"

namespace turret {

constexpr int kYawServoPin = 6;
constexpr int kPitchServoPin = 7;
constexpr int kRollServoPin = 8;

// How close to kStopSpeed is considered "stopped", needed to overcome static
// friction of the turret
constexpr int kYawDeadband = 9;

// new pitch servo with esp32 nano
constexpr int kPitchMax = 115;
constexpr int kPitchMin = 35;
constexpr int kPitchInit = (kPitchMax + kPitchMin) / 2;

// [0, 1]; 0 is lots of smoothing (bias to previous), 1.0 is none
// (bias to new).
constexpr float kAlpha = 0.8;

// PID values
constexpr pid::Ks kYawKs = {
    .p = 1.20,
    .d = 0.018,
    .i = 0.002,
};
constexpr pid::Ks kPitchKs = {
    .p = 15.0,
    .d = 2.0,
    .i = 0.0,
};
// The range of the output from the PID controller. This is used to map the
// controller output to the servo output range. A smaller value makes the
// controller more sensitive.
constexpr int kControlRange = 120;

// time to activate the motor to fire, in millis
constexpr int kRollPrecision = 400;
constexpr int kRollSpeed = 90;

// TODO: Eventually find these dynamically on startup and re-eval periodically.
// Temperature (C) considered "background", varies by time of year.
constexpr float kBackgroundTemp = 21.1; // 70F
// Degrees above "background" to consider for aquisition.
constexpr float kSeedTempThreshold = 4.0;
constexpr float kSearchTempThreshold = 2.0;

// Handle movement definitions for the turret. Includes discete movements (up,
// left, etc) as well as tracking with the heat sensor.
class Movement {
public:
  Movement(bool is_tracking) : is_tracking_(is_tracking) {}

  void Init(uint64_t last_micros) {
    yaw_servo_.Init();
    pitch_servo_.Init(last_micros);
    roll_servo_.Init();
    heat_sensor_.Init();
  }

  bool IsTracking() { return is_tracking_; }

  void LeftMove() {}

  void RightMove() {}

  void UpMove() {}

  void DownMove() {}

  void Fire();

  void ToggleTracking() { is_tracking_ = !is_tracking_; }

  void Track(uint64_t dt);

  void Update(uint64_t current_micros);

private:
  bool is_tracking_;

  servos::ContinuousServo yaw_servo_{kYawServoPin, kYawDeadband};
  servos::FixedRangeServo pitch_servo_{kPitchServoPin, kPitchInit, kPitchMin,
                                       kPitchMax};
  servos::ContinuousServo roll_servo_{kRollServoPin};

  heat_sensor::HeatSensor heat_sensor_{kBackgroundTemp, kSeedTempThreshold,
                                       kSearchTempThreshold, kAlpha};

  pid::Pid<float> yaw_pid_{kYawKs};
  pid::Pid<float> pitch_pid_{kPitchKs};
};

} // namespace turret

#endif // TURRET_MOVEMENT_H_
