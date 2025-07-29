//////////////////////////////////////////////////
//  LIBRARIES  //
//////////////////////////////////////////////////
#include "turret.h"
#include "PinDefinitionsAndMore.h"
#include "bitmask_set.h"
#include "ring_buffer_queue.h"
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

//////////////////////////////////////////////////
//  PINS AND PARAMETERS  //
//////////////////////////////////////////////////

constexpr bool kDebug = true;

Adafruit_AMG88xx heat_sensor;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

Servo yawServo;
Servo pitchServo;
Servo rollServo;

// [0, 180], defines the angle of the servo, with 0 being down.
int pitchServoVal = 100;
// yaw and roll servos are continuous. Input values are positive/negative speeds
// with 90 as the zero value [0, 180].
int yawServoVal = 90;
int rollServoVal = 90;

int lastYawServoVal = 90;
int lastPitchServoVal = 90;
int lastRollServoVal = 90;

int pitchMoveSpeed = 8;
int yawMoveSpeed = 90;
int yawStopSpeed = 90;
int rollMoveSpeed = 90;
int rollStopSpeed = 90;

int yawPrecision = 150;
int rollPrecision = 158;

constexpr int kPitchMax = 175;
constexpr int kPitchMin = 15;

//////////////////////////////////////////////////
// Variables for tracking //
//////////////////////////////////////////////////

bool isTracking = true;

constexpr int kGridSize = 8;
constexpr int kNeighborSize = 8;
constexpr float kSensorFov = 31.5;
constexpr float kGridMidPt = (kGridSize - 1) / 2;
constexpr float kGridToAngle = kSensorFov / kGridMidPt;

constexpr float kP = 5.0;
constexpr float kD = 0.0;
constexpr float kI = 0.0;

// TODO: Eventually find these dynamically on startup and re-eval periodically.
constexpr float kBackgroundTemp = 21.5;
constexpr float kTempThreashold = 1.5;

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
Point<float> last_error = {.x = 0, .y = 0};

//////////////////////////////////////////////////
//  SETUP  //
//////////////////////////////////////////////////

void homeServos() {
  yawServo.write(yawStopSpeed);
  delay(20);
  rollServo.write(rollStopSpeed);
  delay(100);
  pitchServo.write(100);
  delay(100);
  pitchServoVal = 100; // store the pitch servo value
}

void turretSetup() {
  Serial.begin(9600);
  Serial.println(F("AMG88xx test"));

  bool status;

  // default settings
  status = heat_sensor.begin();
  if (!status) {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1)
      ;
  }

  Serial.println("-- Thermistor Test --");

  Serial.println();

  delay(100); // let sensor boot up

  yawServo.attach(10);   // attach YAW servo to pin 3
  pitchServo.attach(11); // attach PITCH servo to pin 4
  rollServo.attach(12);  // attach ROLL servo to pin 5

  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin
  // from the internal boards definition as default feedback LED
  IrReceiver.begin(9, ENABLE_LED_FEEDBACK);

  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin " STR(9)));

  last_millis = millis();

  homeServos();
}

////////////////////////////////////////////////
//  L O O P  //
////////////////////////////////////////////////

template <typename T> T clamp(T val, T min, T max) {
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

int Col(int index) { return index % kGridSize; }

int Row(int index) { return floor(index / kGridSize); }

Point<int> ToPoint(int index) { return {.x = Col(index), .y = Row(index)}; }

int Index(Point<int> p) { return p.x * kGridSize + p.y; }

bool InBounds(Point<int> p) {
  return (p.x < kGridSize && p.x >= 0) && (p.y < kGridSize && p.y >= 0);
}

bool InBounds(int index) {
  return index >= 0 && index < AMG88xx_PIXEL_ARRAY_SIZE;
}

int Move(Point<int> move, int index) {
  Point<int> current = ToPoint(index);
  current += move;
  return InBounds(current) ? Index(current) : -1;
}

// These sets can't be bigger than the search space.
BitmaskSet<AMG88xx_PIXEL_ARRAY_SIZE> visited;
RingBufferQueue<int, AMG88xx_PIXEL_ARRAY_SIZE> q;

int *GetNeighbors(float *temps, int index) {
  int neighbors[kNeighborSize] = {
      // up left
      Move({.x = -1, .y = -1}, index),
      // up
      InBounds(index - kGridSize) ? index - kGridSize : -1,
      // up right
      Move({.x = 1, .y = -1}, index),
      // right
      Move({.x = 1, .y = 0}, index),
      // down right
      Move({.x = 1, .y = 1}, index),
      // down
      InBounds(index + kGridSize) ? index + kGridSize : -1,
      // down left
      Move({.x = -1, .y = 1}, index),
      // left
      Move({.x = -1, .y = 0}, index),
  };
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
    return {.x = -1.0, .y = -1.0};
  }

  // BFS
  bool success = q.Enqueue(max_i);
  if (!success) {
    return {.x = -1.0, .y = -1.0};
  }

  visited.Set(max_i);

  float x_total = 0;
  float y_total = 0;
  float temp_total = 0;

  while (!q.Empty()) {
    int current = q.Dequeue();

    // Sums for weighted average
    x_total += temps[current] * Col(current);
    y_total += temps[current] * Row(current);
    temp_total += temps[current];

    int *neighbors = GetNeighbors(temps, current);
    for (int i = 0; i < kNeighborSize; ++i) {
      if (!visited.Contains(i)) {
        visited.Set(i);
        q.Enqueue(i);
      }
    }
  }

  return {GridToAngle(x_total / temp_total), GridToAngle(y_total / temp_total)};
}

void setYawSpeed(int speed) { yawServo.write(speed); }

void setPitchSpeed(int speed) {
  // take the requested speed [0, 180] with 90 as stopped, like a continuous
  // servo. Translate this speed to a combination of movement direction and gaps
  // between mvmt.
  pitchSpeed = clamp(speed, 0, 180);
  int norm = clamp(abs(speed - 90), 0, 90);
  int pitchStepSize = kMaxPitchStep - (norm / 90);
}

// Check if the pitch servo should be moving, and move it
void movePitch() {
  if (pitchSpeed = 90) {
    return;
  }
  if (millis() > lastPitchMoveMs + pitchStepSize &&
      pitchServoVal != kPitchMax && pitchServoVal != kPitchMin) {
    if (pitchSpeed > 90) {
      pitchServoVal++;
    } else {
      pitchServoVal--;
    }
  }
}

void leftMove(int moves) {
  for (int i = 0; i < moves; i++) {
    yawServo.write(yawStopSpeed + yawMoveSpeed);
    delay(yawPrecision);
    yawServo.write(yawStopSpeed);
    delay(5);
    Serial.println("LEFT");
  }
}

void rightMove(int moves) {
  for (int i = 0; i < moves; i++) {
    yawServo.write(yawStopSpeed - yawMoveSpeed);
    delay(yawPrecision);
    yawServo.write(yawStopSpeed);
    delay(5);
    Serial.println("RIGHT");
  }
}

void upMove(int moves) {
  for (int i = 0; i < moves; i++) {
    if (pitchServoVal > kPitchMin) {
      pitchServoVal = pitchServoVal -
                      pitchMoveSpeed; // decrement the current angle and update
      pitchServo.write(pitchServoVal);
      delay(50);
      Serial.println("UP");
    }
  }
}

void downMove(int moves) {
  for (int i = 0; i < moves; i++) {
    if (pitchServoVal <
        kPitchMax) { // make sure the servo is within rotation
                     // limits (less than 175 degrees by default)
      pitchServoVal = pitchServoVal +
                      pitchMoveSpeed; // increment the current angle and update
      pitchServo.write(pitchServoVal);
      delay(50);
      Serial.println("DOWN");
    }
  }
}

void fire() { // function for firing a single dart
  rollServo.write(rollStopSpeed + rollMoveSpeed); // start rotating the servo
  delay(rollPrecision); // time for approximately 60 degrees of rotation
  rollServo.write(rollStopSpeed); // stop rotating the servo
  delay(5);                       // delay for smoothness
}

void fireAll() { // function to fire all 6 darts at once
  rollServo.write(rollStopSpeed + rollMoveSpeed); // start rotating the servo
  delay(rollPrecision * 6);       // time for 360 degrees of rotation
  rollServo.write(rollStopSpeed); // stop rotating the servo
  delay(5);                       // delay for smoothness
}

void toggleTracking() { isTracking = !isTracking; }

void handleCommand(int command) {
  if ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    Serial.println("DEBOUNCING REPEATED NUMBER - IGNORING INPUT");
    return;
  }

  switch (command) {
  case up:
    if (!isTracking) {
      return;
    }
    upMove(1);
    break;

  case down:
    if (!isTracking) {
      return;
    }
    downMove(1);
    break;

  case left:
    if (!isTracking) {
      return;
    }
    leftMove(1);
    break;

  case right:
    if (!isTracking) {
      return;
    }
    rightMove(1);
    break;

  case ok:
    if (!isTracking) {
      return;
    }
    fire();
    break;

  case star:
    toggleTracking();
    break;

  default:
    // Unknown command, do nothing
    Serial.println("Command Read Failed or Unknown, Try Again");
    break;
  }
}

const char COLOR_0[13] = "\033[48;5;12m";
const char COLOR_19[13] = "\033[48;5;12m";
const char END[9] = "\033[0m";

const char COLOR[20][13] = {
    "\033[48;5;12m",  "\033[48;5;21m",  "\033[48;5;33m",  "\033[48;5;51m",
    "\033[48;5;48m",  "\033[48;5;119m", "\033[48;5;155m", "\033[48;5;191m",
    "\033[48;5;226m", "\033[48;5;222m", "\033[48;5;220m", "\033[48;5;214m",
    "\033[48;5;208m", "\033[48;5;202m", "\033[48;5;198m", "\033[48;5;197m",
    "\033[48;5;196m", "\033[48;5;9m",   "\033[48;5;160",  "\033[48;5;124m",
};
const int MIN_TEMP = 15;

const char *getTermColor(int temp) {
  if (temp < 15)
    return COLOR_0;
  if (temp >= 35)
    return COLOR_19;
  int i = temp - MIN_TEMP;
  return COLOR[i];
}

void PrintGrid(float *temps) {
  for (int i = 1; i <= AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    float temp = temps[i - 1];
    Serial.print(getTermColor(temp));
    Serial.print((int)temp);
    Serial.print(END);
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
  return d_t;
}

void turretLoop() {
  if (IrReceiver.decode()) {
    int command = IrReceiver.decodedIRData.command;
    IrReceiver.resume();
    handleCommand(command);
  }
  delay(5);

  Point<float> current_error = {0, 0};
  int out_x = 0;
  int out_y = 0;
  if (isTracking) {
    // read all the pixels
    heat_sensor.readPixels(pixels);

    current_error = FindHeatCenter(pixels, AMG88xx_PIXEL_ARRAY_SIZE);

    // PID to get to center
    Point<float> derivative_error = (current_error - last_error) / DeltaT();
    last_error = current_error;

    Point<float> output = current_error * kP + derivative_error * kD;
    out_x =
        map(long(clamp(int(round(output.y)), -255, 255)), -255, 255, 0, 180);
    out_y =
        map(long(clamp(int(round(output.y)), -255, 255)), -255, 255, 0, 180);
    setPitchSpeed(out_y);
    setYawSpeed(out_x);
    movePitch();
  }

  if (kDebug) {
    PrintGrid(pixels);
    Serial.print("last error x: ");
    Serial.print(last_error.x);
    Serial.print("\n");

    Serial.print("last error y: ");
    Serial.print(last_error.y);
    Serial.print("\n");

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
  }
}
