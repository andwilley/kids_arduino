#ifndef TURRET_HEAT_SENSOR_H_
#define TURRET_HEAT_SENSOR_H_

#include "bitmask_set.h"
#include "grid.h"
#include "point.h"
#include "ring_buffer_queue.h"
#include <Adafruit_AMG88xx.h>

namespace turret {

constexpr float kSensorFovX = 60.0;
constexpr float kSensorFovY = 60.0;

class HeatSensor {
public:
  HeatSensor(float background_temp, float seed_temp_threshold,
             float search_temp_threshold, float alpha)
      : background_temp_(background_temp),
        seed_temp_threshold_(seed_temp_threshold),
        search_temp_threshold_(search_temp_threshold), alpha_(alpha),
        grid_to_angle_{.x = kSensorFovX / grid_.kCols,
                       .y = kSensorFovY / grid_.kRows} {}

  void Init() {
    const bool status = sensor_.begin();
    if (!status) {
      Log.Log(kWarning, "Could not find a valid AMG88xx sensor, check wiring!");
    }
  }

  void Read() { sensor_.readPixels(grid_.data()); }

  Point<float> FindHeatCenter(const Point<float> &last_center);

  int size() const { return AMG88xx_PIXEL_ARRAY_SIZE; }

  const Point<float> kMiddle = {.x = 0.0, .y = 0.0};

  void Print() const { grid_.Print(); }

private:
  Point<float> grid_to_angle_;
  float background_temp_;
  float seed_temp_threshold_;
  float search_temp_threshold_;
  float alpha_;

  Adafruit_AMG88xx sensor_;
  Grid2d<float, int, /*W=*/8, /*H=*/8> grid_;
  BitmaskSet<AMG88xx_PIXEL_ARRAY_SIZE> visited_;
  RingBufferQueue<Point<int>, AMG88xx_PIXEL_ARRAY_SIZE> q_;

  // Convert an error in grid space to real world sensor space.
  Point<float> ToSensorAngle(const Point<float> &grid_error) {
    Point<float> norm = grid_error - grid_.kMidPt;
    return norm * grid_to_angle_;
  }
};

} // namespace turret

#endif // TURRET_HEAT_SENSOR_H
