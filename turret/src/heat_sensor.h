#ifndef TURRET_HEAT_SENSOR_H_
#define TURRET_HEAT_SENSOR_H_

#include "bitmask_set.h"
#include "grid.h"
#include "point.h"
#include "ring_buffer_queue.h"
#include <Adafruit_AMG88xx.h>

namespace turret {

class HeatSensor {
public:
  HeatSensor(float background_temp, float temp_threshold, float sensor_fov_x,
             float sensor_fov_y)
      : background_temp_(background_temp), temp_threshold_(temp_threshold),
        grid_to_angle_{.x = sensor_fov_x / grid_.kCols,
                       .y = sensor_fov_y / grid_.kRows} {}

  void Init() {
    bool status = sensor_.begin();
    if (!status) {
      Log.Log(kWarning, "Could not find a valid AMG88xx sensor, check wiring!");
    }
  }

  void Read() { sensor_.readPixels(grid_.data()); }

  Point<float> FindHeatCenter();

  int size() { return AMG88xx_PIXEL_ARRAY_SIZE; }

  const Point<float> kMiddle = {.x = grid_.kColMidPt, .y = grid_.kRowMidPt};

  void Print() { grid_.Print(); }

private:
  float background_temp_;
  float temp_threshold_;
  const Point<float> grid_to_angle_;

  Adafruit_AMG88xx sensor_;
  Grid2d<float, int, /*W=*/8, /*H=*/8> grid_;
  BitmaskSet<AMG88xx_PIXEL_ARRAY_SIZE> visited_;
  RingBufferQueue<Point<int>, AMG88xx_PIXEL_ARRAY_SIZE> q_;

  // Convert an error in grid space to real world sensor space.
  Point<float> ToSensorAngle(Point<float> grid_error) {
    Point<float> norm =
        grid_error - Point<float>{.x = grid_.kColMidPt, .y = grid_.kRowMidPt};
    return norm * grid_to_angle_;
  }
};

} // namespace turret

#endif // TURRET_HEAT_SENSOR_H
