#ifndef TURRET_H_
#define TURRET_H_

#include "point.h"
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
constexpr int kPitchInit = (kPitchMax + kPitchMin) / 2 + kPitchMin;

// time to activate the motor to fire, in millis
constexpr int kRollPrecision = 158;
constexpr int kRollSpeed = 90;

// How close to 90 is considered "stopped", needed to overcome static friction
// of the turret
constexpr int kYawDeadband = 25;

// Don't react for very small deviations from 0 error
constexpr float kErrorToleranceX = 0.3;
constexpr float kErrorToleranceY = 0.1;

// PID values
constexpr float kP = 30.0;
constexpr float kD = 0.0;
constexpr float kI = 0.0;
// The range of the output from the PID controller. This is used to map the
// controller output to the servo output range. A smaller value makes the
// controller more sensitive.
constexpr int kControlRange = 100;

constexpr float kGridToAngle = kSensorFov / kGridMidPt;

// TODO: Eventually find these dynamically on startup and re-eval periodically.
// Temperature (C) considered "background", varies by time of year.
constexpr float kBackgroundTemp = 21.5;
// Degrees above "background" to consider for aquisition.
constexpr float kTempThreashold = 1.5;

} // namespace turret

#endif // TURRET_H_
