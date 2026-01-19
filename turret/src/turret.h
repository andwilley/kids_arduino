#ifndef TURRET_H_
#define TURRET_H_

#include <Adafruit_AMG88xx.h>

namespace turret {

constexpr int kIrReceiverPin = 5;

// Turn on debug logging. introduces latency.
constexpr bool kDebug = true;

void Setup();
void Loop();

} // namespace turret

#endif // TURRET_H_
