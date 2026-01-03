#ifndef TURRET_H_
#define TURRET_H_

#include "pid.h"
#include <Adafruit_AMG88xx.h>
#ifdef GMOCK_FLAG
#endif

namespace turret {

void Setup();
void Loop();

constexpr int kYawServoPin = 6;
constexpr int kPitchServoPin = 7;
constexpr int kRollServoPin = 8;

// Turn on debug logging. introduces latency.
constexpr bool kDebug = true;

// new pitch servo with esp32 nano
constexpr int kPitchMax = 115;
constexpr int kPitchMin = 35;
constexpr int kPitchInit = (kPitchMax + kPitchMin) / 2;

// time to activate the motor to fire, in millis
constexpr int kRollPrecision = 158;
constexpr int kRollSpeed = 90;

// How close to kStopSpeed is considered "stopped", needed to overcome static
// friction of the turret
constexpr int kYawDeadband = 9;

// Don't react for very small deviations from 0 error
constexpr float kErrorToleranceX = 1.0;
constexpr float kErrorToleranceY = 1.0;

// PID values
constexpr Ks kYawKs = {
    .p = 1.20,
    .d = 0.018,
    .i = 0.002,
};
constexpr Ks kPitchKs = {
    .p = 15.0,
    .d = 2.0,
    .i = 0.0,
};
// The range of the output from the PID controller. This is used to map the
// controller output to the servo output range. A smaller value makes the
// controller more sensitive.
constexpr int kControlRange = 120;

// TODO: Eventually find these dynamically on startup and re-eval periodically.
// Temperature (C) considered "background", varies by time of year.
constexpr float kBackgroundTemp = 21.1; // 70F
// Degrees above "background" to consider for aquisition.
constexpr float kSeedTempThreshold = 4.0;
constexpr float kSearchTempThreshold = 2.0;

// [0, 1]; 0 is lots of smoothing (bias to previous), 1.0 is none
// (bias to new).
constexpr float kAlpha = 0.8;

} // namespace turret

#endif // TURRET_H_
