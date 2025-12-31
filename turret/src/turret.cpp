#include "continuous_servo.h"
#include "fixed_range_servo.h"
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

bool isTracking = false;

uint64_t last_micros = 0;

void Setup() {
  Serial.begin(9600);

  last_micros = micros();

  yawServo.Init();
  pitchServo.Init(last_micros);
  rollServo.Init();

  IrReceiver.begin(5, ENABLE_LED_FEEDBACK);
}


void LeftMove() {}

void RightMove() {}

void UpMove() {}

void DownMove() {}

void Fire() {                               // function for firing a single dart
  rollServo.write(kStopSpeed + kRollSpeed); // start rotating the servo
  delay(kRollPrecision);       // time for approximately 60 degrees of rotation
  rollServo.write(kStopSpeed); // stop rotating the servo
  delay(5);                    // delay for smoothness
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

uint64_t DeltaT() {
  uint64_t cur = micros();
  uint64_t dt = cur - last_micros;
  last_micros = cur;
  return dt == 0 ? 1 : dt;
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
  int out_x = 0;
  int out_y = 0;
  // read all the pixels
  heat_sensor.readPixels(pixels);

  if (isTracking) {
    current_error = FindHeatCenter(pixels, AMG88xx_PIXEL_ARRAY_SIZE);
    RotateError90Cw(current_error);

    // PID to get to center
    Point<float> derivative_error = (current_error - last_error) / DeltaT();
    last_error = current_error;

    Point<float> output = current_error * kP + derivative_error * kD;
    out_x =
        map(long(Clamp(int(round(output.x)), -kControlRange, kControlRange)),
            -kControlRange, kControlRange, 0, 180);
    out_y =
        map(long(Clamp(int(round(output.y)), -kControlRange, kControlRange)),
            -kControlRange, kControlRange, 0, 180);
    SetPitchSpeed(out_y);
    SetYawSpeed(out_x);
    MovePitch();

    if (kDebug) {
      PrintGrid(pixels);

      Serial.print("current error x: ");
      Serial.print(current_error.x);
      Serial.print("\n");

      Serial.print("current error y: ");
      Serial.print(current_error.y);
      Serial.print("\n");

      Serial.print("output x: ");
      Serial.print(out_x);
      Serial.print("\n");

      Serial.print("output y: ");
      Serial.print(out_y);
      Serial.print("\n");

      Serial.print("pitch speed: ");
      Serial.print(pitchSpeed);
      Serial.print("\n");

      Serial.print("pitch servo val: ");
      Serial.print(pitchServoVal);
      Serial.print("\n");

      Serial.print("yaw servo val: ");
      Serial.print(yawServoVal);
      Serial.print("\n");
      Serial.print("\n");
      Serial.print("\n");

      // Don't dump to serial so fast we can't read it.
      delay(500);
    }
  } else {
    if (IrReceiver.decode()) {
      int command = IrReceiver.decodedIRData.command;
      IrReceiver.resume();
      HandleCommand(command);
    }
    delay(5);
  }
}

} // namespace turret
