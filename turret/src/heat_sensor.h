#ifndef HEAT_SENSOR_H_
#define HEAT_SENSOR_H_

#include "bitmask_set.h"
#include "grid.h"
#include "point.h"
#include "ring_buffer_queue.h"
#include <Adafruit_AMG88xx.h>

namespace heat_sensor {

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
      logger::Log.Log(logger::kWarning,
                      "Could not find a valid AMG88xx sensor, check wiring!");
    }
  }

  void Read() { sensor_.readPixels(grid_.data()); }

  geometry::Point<float>
  FindHeatCenter(const geometry::Point<float> &last_center);

  int size() const { return AMG88xx_PIXEL_ARRAY_SIZE; }

  const geometry::Point<float> kMiddle = {.x = 0.0, .y = 0.0};

  void Print() const { grid_.Print(); }

private:
  geometry::Point<float> grid_to_angle_;
  float background_temp_;
  float seed_temp_threshold_;
  float search_temp_threshold_;
  float alpha_;

  Adafruit_AMG88xx sensor_;
  grid::Grid2d<float, int, /*W=*/8, /*H=*/8> grid_;
  bitmask_set::BitmaskSet<AMG88xx_PIXEL_ARRAY_SIZE> visited_;
  ring_buffer_queue::RingBufferQueue<geometry::Point<int>,
                                     AMG88xx_PIXEL_ARRAY_SIZE>
      q_;

  // Convert an error in grid space to real world sensor space.
  geometry::Point<float>
  ToSensorAngle(const geometry::Point<float> &grid_error) {
    geometry::Point<float> norm = grid_error - grid_.kMidPt;
    return norm * grid_to_angle_;
  }
};

} // namespace heat_sensor

#endif // HEAT_SENSOR_H
