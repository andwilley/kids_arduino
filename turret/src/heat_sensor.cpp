#include "heat_sensor.h"
#include "point.h"
#include <Adafruit_AMG88xx.h>

namespace turret {

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
    return ToSensorAngle(kMiddle);
  }

  // BFS
  q_.Enqueue(grid_.PointFrom(max_i));

  visited_.Set(max_i);

  float x_total = 0;
  float y_total = 0;
  float temp_total = 0;

  while (!q_.Empty()) {
    Point<int> current = q_.Dequeue();
    if (!grid_.InBounds(current)) {
      return ToSensorAngle(kMiddle);
    }
    float cur_temp = grid_.At(current);

    // Sums for weighted average
    x_total += cur_temp * current.x;
    y_total += cur_temp * current.y;
    temp_total += cur_temp;

    for (const auto &neighbor : grid_.Neighbors(current)) {
      // Neighbors() checks bounds
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
    return ToSensorAngle(kMiddle);
  }

  return {
      ToSensorAngle({.x = x_total / temp_total, .y = y_total / temp_total})};
}

} // namespace turret
