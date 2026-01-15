#include "heat_sensor.h"
#include "point.h"
#include <Adafruit_AMG88xx.h>

namespace heat_sensor {

geometry::Point<float>
HeatSensor::FindHeatCenter(const geometry::Point<float> &last_center) {
  visited_.Clear();
  q_.Clear();

  const geometry::Point<int> max_point = grid_.MaxPoint();
  float max_temp = grid_.At(max_point);
  size_t max_i = grid_.IndexOf(max_point);

  if (max_temp < background_temp_ + seed_temp_threshold_) {
    return kMiddle;
  }

  q_.Enqueue(grid_.PointFrom(max_i));

  visited_.Set(max_i);

  float x_total = 0;
  float y_total = 0;
  float temp_total = 0;
  geometry::Point<int> current;
  while (q_.PopFront(current)) {
    // Point comes from grid method
    const float cur_temp = grid_.At(current);

    // Sums for weighted average
    x_total += cur_temp * current.x;
    y_total += cur_temp * current.y;
    temp_total += cur_temp;

    for (const auto &neighbor : grid_.Neighbors(current)) {
      // Neighbors() checks bounds
      const float neighbor_temp = grid_.At(neighbor);
      const size_t neighbor_idx = grid_.IndexOf(neighbor);
      if (!visited_.Contains(neighbor_idx) &&
          neighbor_temp > background_temp_ + search_temp_threshold_) {
        visited_.Set(neighbor_idx);
        q_.Enqueue(neighbor);
      }
    }
  }

  if (temp_total == 0) {
    return kMiddle;
  }

  const geometry::Point<float> center =
      ToSensorAngle({.x = x_total / temp_total, .y = y_total / temp_total});

  return last_center + (center - last_center) * alpha_;
}

} // namespace heat_sensor
