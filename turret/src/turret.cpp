#include "continuous_servo.h"
#include "fixed_range_servo.h"
#include "heat_sensor.h"
#include "pid.h"
#include "remote.h"
#include "servo_constants.h"
#ifdef GMOCK_FLAG
#include "Arduino.h"
#include <cmath>
#endif // GMOCK_FLAG

#include "logger.h"
#include "point.h"
#include "turret.h"
#include <Arduino.h>

namespace turret {

// TODO: move these to movement, and expose from there
ContinuousServo yaw_servo(kYawServoPin, kYawDeadband);
FixedRangeServo pitch_servo(kPitchServoPin, kPitchInit, kPitchMin, kPitchMax);
ContinuousServo roll_servo(kRollServoPin);

HeatSensor heat_sensor(kBackgroundTemp, kSeedTempThreshold,
                       kSearchTempThreshold, kAlpha);
Pid<float> yaw_pid(kYawKs);
Pid<float> pitch_pid(kPitchKs);
Logger Log(kDebug);

bool isTracking = true;
uint64_t last_micros = 0;

void Setup() {
  Serial.begin(9600);
  Log.Init();

  remote::Remote::Instance().Init();

  last_micros = micros();

  yaw_servo.Init();
  pitch_servo.Init(last_micros);
  roll_servo.Init();

  heat_sensor.Init();
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

// TODO: move this to remote.h
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
  uint64_t current_micros = micros();
  uint64_t dt = current_micros - last_micros;
  last_micros = current_micros;

  if (isTracking) {
    heat_sensor.Read();

    Point<float> current_error = heat_sensor.FindHeatCenter(Point<float>{
        .x = yaw_pid.GetLastError(), .y = pitch_pid.GetLastError()});
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
