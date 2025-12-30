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
#include <Adafruit_AMG88xx.h>
#include <Arduino.h>
#include <IRremote.hpp>
#include <Servo.h>

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
 * *   implement continuous integral movement in fixed servo class
 * *   class for the PID math
 * *   class for the heat sensor, consilidate work on that data source,
 *     including finding the heat center, finding the highest value, thresholds,
 *     etc.
 * *   move utilities like traversing a 1D grid to turret_math.h
 * *   class for debug logging. perhaps include levels? Ability to write to
 *     serial and control with flags and env vars.
 */

using turret_math::Clamp;

namespace turret {

//////////////////////////////////////////////////
//  PINS AND PARAMETERS  //
//////////////////////////////////////////////////

Adafruit_AMG88xx heat_sensor;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

Servo yawServo;
Servo pitchServo;
Servo rollServo;

// [0, 180], defines the angle of the servo, with 0 being down.
int pitchServoVal = kPitchInit;
// yaw and roll servos are continuous. Input values are positive/negative speeds
// with 90 as the zero value [0, 180].
int yawServoVal = 90;
int rollServoVal = 90;

//////////////////////////////////////////////////
// Variables for tracking //
//////////////////////////////////////////////////

bool isTracking = false;

// The last time pitch moved. Track this to move it with varying pauses to
// simulate speed.
int lastPitchMoveMs = 0;
// Requested pitch speed [0, 180] with 90 stopped.
int pitchSpeed = 90;
// A ms delay that translates to effective servo speed, bigger is slower.
int pitchStepSize = 0;
// Tune this for best results. This sets the upper bound for linear
// interpolation.
constexpr int kMaxPitchStep = 300;

uint64_t last_millis = 0;
Point<float> last_error = {.x = 0.0, .y = 0.0};

//////////////////////////////////////////////////
//  SETUP  //
//////////////////////////////////////////////////

void HomeServos() {
  yawServo.write(kStopSpeed);
  delay(20);
  rollServo.write(kStopSpeed);
  delay(20);
  pitchServo.write(pitchServoVal);
  delay(20);
}

void TurretSetup() {
  Serial.begin(9600);
  Serial.println(F("AMG88xx test"));

  bool status;

  // default settings
  status = heat_sensor.begin();
  if (!status) {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
  }

  Serial.println("-- Thermistor Test --");

  Serial.println();

  delay(100); // let sensor boot up

  yawServo.attach(6);
  pitchServo.attach(7);
  rollServo.attach(8);

  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin
  // from the internal boards definition as default feedback LED
  IrReceiver.begin(5, ENABLE_LED_FEEDBACK);

  Serial.print(F("Ready to receive IR signals of protocols: "));

  last_millis = millis();

  HomeServos();
}

////////////////////////////////////////////////
//  L O O P  //
////////////////////////////////////////////////

int Col(int index) { return index % kGridSize; }

int Row(int index) { return index / kGridSize; }

Point<int> ToPoint(int index) { return {.x = Col(index), .y = Row(index)}; }

int Index(Point<int> p) { return p.y * kGridSize + p.x; }

bool InBounds(Point<int> p) {
  return (p.x < kGridSize && p.x >= 0) && (p.y < kGridSize && p.y >= 0);
}

bool InBounds(int index) {
  return index >= 0 && index < AMG88xx_PIXEL_ARRAY_SIZE;
}

int Move(Point<int> move, int index) {
  Point<int> current = ToPoint(index) + move;
  return InBounds(current) ? Index(current) : -1;
}

// These sets can't be bigger than the search space.
BitmaskSet<AMG88xx_PIXEL_ARRAY_SIZE> visited;
RingBufferQueue<int, AMG88xx_PIXEL_ARRAY_SIZE> q;

void GetNeighbors(int index, int *neighbors_out) {
  // up left
  neighbors_out[0] = Move({.x = -1, .y = -1}, index);
  // up
  neighbors_out[1] = InBounds(index - kGridSize) ? index - kGridSize : -1;
  // up right
  neighbors_out[2] = Move({.x = 1, .y = -1}, index);
  // right
  neighbors_out[3] = Move({.x = 1, .y = 0}, index);
  // down right
  neighbors_out[4] = Move({.x = 1, .y = 1}, index);
  // down
  neighbors_out[5] = InBounds(index + kGridSize) ? index + kGridSize : -1;
  // down left
  neighbors_out[6] = Move({.x = -1, .y = 1}, index);
  // left
  neighbors_out[7] = Move({.x = -1, .y = 0}, index);
}

// grid represents a continuous range over the columns or rows, rather than
// discrete int columns.
float GridToAngle(float grid) {
  float norm = grid - kGridMidPt;
  return norm * kGridToAngle;
}

Point<float> FindHeatCenter(float *temps, size_t size) {
  visited.Clear();
  q.Clear();

  // Get the index with the highest temp
  int max_i = 0;
  float max = kBackgroundTemp;
  for (int i = 0; i < size; ++i) {
    if (temps[i] > kBackgroundTemp + kTempThreashold && temps[i] > max) {
      max_i = i;
      max = temps[i];
    }
  }

  // No temps above threshold
  if (max == kBackgroundTemp) {
    return {.x = 0.0, .y = 0.0};
  }

  // BFS
  bool success = q.Enqueue(max_i);
  if (!success) {
    return {.x = 0.0, .y = 0.0};
  }

  visited.Set(max_i);

  float x_total = 0;
  float y_total = 0;
  float temp_total = 0;

  while (!q.Empty()) {
    int current = q.Dequeue();
    float cur_temp = temps[current];

    // Sums for weighted average
    x_total += cur_temp * Col(current);
    y_total += cur_temp * Row(current);
    temp_total += cur_temp;

    int neighbors[kNeighborSize];
    GetNeighbors(current, neighbors);
    for (int i = 0; i < kNeighborSize; ++i) {
      int neighbor_idx = neighbors[i];
      if (neighbor_idx != -1 && !visited.Contains(neighbor_idx) &&
          temps[neighbor_idx] > kBackgroundTemp + kTempThreashold) {
        visited.Set(neighbor_idx);
        q.Enqueue(neighbor_idx);
      }
    }
  }

  if (temp_total == 0) {
    return {.x = 0.0, .y = 0.0};
  }

  Point<float> error = {GridToAngle(x_total / temp_total),
                        GridToAngle(y_total / temp_total)};
  return error.ApplyTolerance(kErrorToleranceX, kErrorToleranceY);
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

void PrintGrid(float *temps) {
  for (int i = 1; i <= AMG88xx_PIXEL_ARRAY_SIZE; ++i) {
    float temp = temps[i - 1];
    Serial.print((int)temp);
    Serial.print(", ");
    if (i % 8 == 0) {
      Serial.println();
    }
  }
  Serial.println();
}

uint64_t DeltaT() {
  uint64_t cur = millis();
  uint64_t d_t = cur - last_millis;
  last_millis = cur;
  return d_t == 0 ? 1 : d_t;
}

// The sensor is rotated 90 degrees clockwise. This function transforms the
// error coordinates to match the turret's orientation. A point(x, y) in the
// sensor's frame becomes (y, -x) in the turret's frame.
void RotateError90Cw(Point<float> &error) {
  float temp_x = error.x;
  error.x = error.y;
  error.y = -temp_x;
}

void TurretLoop() {
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
