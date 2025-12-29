#ifndef TURRET
#define TURRET

#include <Adafruit_AMG88xx.h>
#ifdef GMOCK_FLAG
#include <cstddef>
#endif

void TurretSetup();
void TurretLoop();

// Turn on debug logging. introduces latency.
constexpr bool kDebug = true;

// new pitch servo with esp32 nano
constexpr int kPitchMax = 115;
constexpr int kPitchMin = 35;
constexpr int kPitchInit = (kPitchMax + kPitchMin) / 2 + kPitchMin;

constexpr int kStopSpeed = 90;
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

// Grid constants, based on the sensors in the termal grid.
constexpr int kGridSize = 8;
// This should never change.
constexpr int kNeighborSize = 8;
// Coarsly measured
constexpr float kSensorFov = 31.5;
constexpr float kGridMidPt = (kGridSize - 1) / 2.0f;
constexpr float kGridToAngle = kSensorFov / kGridMidPt;

// TODO: Eventually find these dynamically on startup and re-eval periodically.
// Temperature (C) considered "background", varies by time of year.
constexpr float kBackgroundTemp = 21.5;
// Degrees above "background" to consider for aquisition.
constexpr float kTempThreashold = 1.5;

template <typename T> struct Point {
  T x;
  T y;

  Point operator+(Point other) { return {.x = x + other.x, .y = y + other.y}; }
  Point operator-(Point other) { return {.x = x - other.x, .y = y - other.y}; }
  Point operator*(const T rhs) { return {.x = x * rhs, .y = y * rhs}; }
  Point operator/(const T divisor) {
    return {.x = x / divisor, .y = y / divisor};
  }
  void operator+=(Point other) {
    x += other.x;
    y += other.y;
  }
  void operator-=(Point other) {
    x -= other.x;
    y -= other.y;
  }

  bool operator==(Point other) { return x == other.x && y == other.y; }
  bool operator!=(Point other) { return x != other.x || y != other.y; }
};

Point<float> FindHeatCenter(float *temps, size_t size);

template <typename T> Point<T> ApplyTolerance(const Point<T> &error) {
  if (abs(error.x) > kErrorToleranceX && abs(error.y) > kErrorToleranceY) {
    return error;
  }

  T adjusted_x = error.x;
  T adjusted_y = error.y;

  if (abs(error.x) < kErrorToleranceX) {
    adjusted_x = T(0);
  }
  if (abs(error.y) < kErrorToleranceY) {
    adjusted_y = T(0);
  }
  return {
      .x = adjusted_x,
      .y = adjusted_y,
  };
}

template <typename T> T Clamp(T val, T min, T max) {
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

#endif // TURRET
