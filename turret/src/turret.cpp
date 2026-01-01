#include "continuous_servo.h"
#include "fixed_range_servo.h"
#include "heat_sensor.h"
#include "pid.h"
#include "servo_constants.h"
#ifdef GMOCK_FLAG
#include "Arduino.h"
#include <cmath>
#endif // GMOCK_FLAG

//////////////////////////////////////////////////
//  LIBRARIES  //
//////////////////////////////////////////////////
#include "bitmask_set.h"
#include "point.h"
#include "ring_buffer_queue.h"
#include "turret.h"
#include "turret_math.h"
#include <Arduino.h>
#include <IRremote.hpp>

// IR transmission type
#define DECODE_NEC

// IR remote command codes
#define left 0x8
#define right 0x5A
#define up 0x52
#define down 0x18
#define ok 0x1C
#define cmd1 0x45
#define cmd2 0x46
#define cmd3 0x47
#define cmd4 0x44
#define cmd5 0x40
#define cmd6 0x43
#define cmd7 0x7
#define cmd8 0x15
#define cmd9 0x9
#define cmd0 0x19
#define star 0x16
#define hashtag 0xD

/*
 * TODO
 * *   class for the PID math
 * *   class for the heat sensor, consilidate work on that data source,
 *     including finding the heat center, finding the highest value, thresholds,
 *     grid index (float) to angle, etc.
 * *   class for debug logging. perhaps include levels? Ability to write to
 *     serial and control with flags and env vars.
 */

namespace turret {

ContinuousServo yawServo(kYawServoPin, kYawDeadband);
FixedRangeServo pitchServo(kPitchServoPin, kPitchInit, kPitchMin, kPitchMax);
ContinuousServo rollServo(kRollServoPin);
HeatSensor heat_sensor(kBackgroundTemp, kTempThreashold, kSensorFov);
Pid<Point<float>> pid(kP, kI, kD);

bool isTracking = false;
uint64_t last_micros = 0;

void Setup() {
  Serial.begin(9600);

  last_micros = micros();

  yawServo.Init();
  pitchServo.Init(last_micros);
  rollServo.Init();

  heat_sensor.Init();

  IrReceiver.begin(5, ENABLE_LED_FEEDBACK);
}

void LeftMove() {}

void RightMove() {}

void UpMove() {}

void DownMove() {}

void Fire() { // function for firing a single dart
  rollServo.SetSpeed(kStopSpeed + kRollSpeed); // start rotating the servo
  delay(kRollPrecision); // time for approximately 60 degrees of rotation
  rollServo.SetSpeed(kStopSpeed); // stop rotating the servo
  delay(5);                       // delay for smoothness
}

void ToggleTracking() { isTracking = !isTracking; }

void HandleCommand(int command) {
  if ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    Serial.println("DEBOUNCING REPEATED NUMBER - IGNORING INPUT");
    return;
  }

  switch (command) {
  case up:
    if (!isTracking) {
      return;
    }
    UpMove();
    break;

  case down:
    if (!isTracking) {
      return;
    }
    DownMove();
    break;

  case left:
    if (!isTracking) {
      return;
    }
    LeftMove();
    break;

  case right:
    if (!isTracking) {
      return;
    }
    RightMove();
    break;

  case ok:
    Fire();
    break;

  case star:
    ToggleTracking();
    break;

  default:
    // Unknown command, do nothing
    Serial.println("Command Read Failed or Unknown, Try Again");
    break;
  }
}

float MapToServo(float value, float from_min, float from_max) {
  float proportion = (value - from_min) / (from_max - from_min);
  return kMaxNegativeSpeed +
         (proportion * (kMaxPositiveSpeed - kMaxNegativeSpeed));
}

// The sensor is rotated 90 degrees clockwise. This function transforms the
// error coordinates to match the turret's orientation. A point(x, y) in the
// sensor's frame becomes (y, -x) in the turret's frame.
void RotateError90Cw(Point<float> &error) {
  float temp_x = error.x;
  error.x = error.y;
  error.y = -temp_x;
}

void Loop() {
  Point<float> current_error = {0.0, 0.0};
  float yaw_out = 0.0;
  float pitch_out = 0.0;
  uint64_t current_micros = micros();
  uint64_t dt = current_micros - last_micros;
  last_micros = current_micros;

  if (isTracking) {
    heat_sensor.Read();

    current_error = heat_sensor.FindHeatCenter();
    current_error =
        current_error.ApplyTolerance(kErrorToleranceX, kErrorToleranceY);
    RotateError90Cw(current_error);

    Point<float> output = pid.Compute(current_error, dt);

    yaw_out = MapToServo(output.x, -kControlRange, kControlRange);
    pitch_out = MapToServo(output.y, -kControlRange, kControlRange);
    pitchServo.SetSpeed(pitch_out);
    yawServo.SetSpeed(yaw_out);

  } else {
    if (IrReceiver.decode()) {
      int command = IrReceiver.decodedIRData.command;
      IrReceiver.resume();
      HandleCommand(command);
    }
  }
  pitchServo.Update(current_micros);
}

} // namespace turret
