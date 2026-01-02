#include "continuous_servo.h"
#include "fixed_range_servo.h"
#include "heat_sensor.h"
#include "pid.h"
#include "servo_constants.h"
#ifdef GMOCK_FLAG
#include "Arduino.h"
#include <cmath>
#endif // GMOCK_FLAG

#include "logger.h"
#include "point.h"
#include "turret.h"
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

namespace turret {

ContinuousServo yaw_servo(kYawServoPin, kYawDeadband);
FixedRangeServo pitch_servo(kPitchServoPin, kPitchInit, kPitchMin, kPitchMax);
ContinuousServo roll_servo(kRollServoPin);
HeatSensor heat_sensor(kBackgroundTemp, kTempThreashold, kSensorFov);
Pid<float> yaw_pid(kYawKs);
Pid<float> pitch_pid(kPitchKs);
Logger Log(true);

bool isTracking = true;
uint64_t last_micros = 0;

void Setup() {
  Serial.begin(9600);
  Log.Init();

  last_micros = micros();

  yaw_servo.Init();
  pitch_servo.Init(last_micros);
  roll_servo.Init();

  heat_sensor.Init();

  IrReceiver.begin(5, ENABLE_LED_FEEDBACK);
}

void LeftMove() {}

void RightMove() {}

void UpMove() {}

void DownMove() {}

void Fire() {
  roll_servo.SetSpeed(kStopSpeed + kRollSpeed);
  delay(kRollPrecision);
  roll_servo.SetSpeed(kStopSpeed);
  delay(5);
}

void ToggleTracking() { isTracking = !isTracking; }

void HandleCommand(int command) {
  if ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    Log.Log(kWarning, "DEBOUNCING REPEATED NUMBER - IGNORING INPUT");
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
    Log.Log(kInfo, "Command Read Failed or Unknown, Try Again");
    break;
  }
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

    float yaw_output = yaw_pid.Compute(current_error.x, dt);
    float pitch_output = pitch_pid.Compute(current_error.y, dt);

    yaw_servo.MapSetSpeed(yaw_output, -kControlRange, kControlRange);
    pitch_servo.MapSetSpeed(pitch_output, -kControlRange, kControlRange);
  } else {
    if (IrReceiver.decode()) {
      int command = IrReceiver.decodedIRData.command;
      IrReceiver.resume();
      HandleCommand(command);
    }
  }
  pitch_servo.Update(current_micros);
}

} // namespace turret
