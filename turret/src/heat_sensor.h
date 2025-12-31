#ifndef TURRET_HEAT_SENSOR_H_
#define TURRET_HEAT_SENSOR_H_

#include "bit_mask_set.h"
#include "grid.h"
#include "point.h"
#include "ring_buffer_queue.h"
#include <Adafruit_AMG88xx.h>

namespace turret {

// TODO: Do we need separate FOVs for x and y?
class HeatSensor {
public:
  HeatSensor(float background_temp, float temp_threshold, float sensor_fov)
      : background_temp_(background_temp), temp_threshold_(temp_threshold),
        sensor_fov_(sensor_fov),
        // Because rows == cols
        grid_to_angle_(sensor_fov_ / grid_.kRowMidPt) {}

  void Init() {
    bool status = sensor_.begin();
    if (!status) {
      Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    }
  }

  void Read() { sensor_.readPixels(grid_.data()); }

  Point<float> FindHeatCenter();

  int size() { return AMG88xx_PIXEL_ARRAY_SIZE; }

private:
  float background_temp_;
  float temp_threshold_;
  float sensor_fov_;
  const float grid_to_angle_;

  Adafruit_AMG88xx sensor_;
  Grid2d<float, int, /*W=*/8, /*H=*/8> grid_;
  BitmaskSet<AMG88xx_PIXEL_ARRAY_SIZE> visited_;
  RingBufferQueue<Point<int>, AMG88xx_PIXEL_ARRAY_SIZE> q_;

  // Convert an error in grid space to real world sensor space.
  Point<float> ToSensorAngle(Point<float> grid_error) {
    Point<float> norm = grid_error - grid_.kRowMidPt;
    return norm * grid_to_angle_;
  }
};

Point<float> HeatSensor::FindHeatCenter() {
  visited_.Clear();
  q_.Clear();

  // Get the index with the highest temp
  int max_i = 0;
  float max = background_temp_;
  float current = 0.0;
  for (int i = 0; i < grid_.kSize; ++i) {
    current = grid_.data()[i];
    if (current > background_temp_ + temp_threshold_ && current > max) {
      max_i = i;
      max = current;
    }
  }

  // No temps above threshold
  if (max == background_temp_) {
    return {.x = 0.0, .y = 0.0};
  }

  // BFS
  q_.Enqueue(grid_.PointFrom(max_i));

  visited_.Set(max_i);

  float x_total = 0;
  float y_total = 0;
  float temp_total = 0;

  while (!q_.Empty()) {
    Point<int> current = q_.Dequeue();
    float cur_temp = grid_.At(current);

    // Sums for weighted average
    x_total += cur_temp * current.x;
    y_total += cur_temp * current.y;
    temp_total += cur_temp;

    for (const auto &neighbor : grid_.Neighbors(current)) {
      float neighbor_temp = grid_.At(neighbor);
      size_t neighbor_idx = grid_.IndexOf(neighbor);
      if (!visited_.Contains(neighbor_idx) &&
          neighbor_temp > background_temp_ + temp_threshold_) {
        visited_.Set(neighbor_idx);
        q_.Enqueue(neighbor);
      }
    }
  }

  if (temp_total == 0) {
    return {.x = 0.0, .y = 0.0};
  }

  return {
      ToSensorAngle({.x = x_total / temp_total, .y = y_total / temp_total})};
}

} // namespace turret

#endif // TURRET_HEAT_SENSOR_H
