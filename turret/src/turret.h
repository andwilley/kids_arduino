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

// How close to 90 is considered "stopped", needed to overcome static friction
// of the turret
constexpr int kYawDeadband = 25;

// Don't react for very small deviations from 0 error
constexpr float kErrorToleranceX = 0.3;
constexpr float kErrorToleranceY = 1.0;

// PID values
constexpr Ks kYawKs = {
    .p = 0.045,
    .d = 0.20,
    .i = 0.005,
};
constexpr Ks kPitchKs = {
    .p = 5.0,
    .d = 0.0,
    .i = 0.0,
};
// The range of the output from the PID controller. This is used to map the
// controller output to the servo output range. A smaller value makes the
// controller more sensitive.
constexpr int kControlRange = 120;

constexpr float kSensorFov = 31.5;

// TODO: Eventually find these dynamically on startup and re-eval periodically.
// Temperature (C) considered "background", varies by time of year.
constexpr float kBackgroundTemp = 21.5;
// Degrees above "background" to consider for aquisition.
constexpr float kTempThreashold = 2.0;

} // namespace turret

#endif // TURRET_H_
