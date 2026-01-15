#include "movement.h"
#include "remote.h"
#ifdef GMOCK_FLAG
#include "Arduino.h"
#include <cmath>
#endif // GMOCK_FLAG

#include "logger.h"
#include "turret.h"
#include <Arduino.h>

namespace turret {

remote::Remote remote(kIrReceiverPin);
Movement mvmt(/*is_tracking=*/true);

uint64_t last_micros = 0;
int64_t HandleRemoteCommands(uint16_t *commands, size_t last_index) {
  uint16_t command = commands[0];
  switch (command) {
  case remote::kUp:
    if (mvmt.IsTracking()) {
      return -1;
    }
    mvmt.UpMove();
  case remote::kDown:
    if (mvmt.IsTracking()) {
      return -1;
    }
    mvmt.DownMove();
    break;
  case remote::kLeft:
    if (mvmt.IsTracking()) {
      return -1;
    }
    mvmt.LeftMove();
    break;
  case remote::kRight:
    if (mvmt.IsTracking()) {
      return -1;
    }
    mvmt.RightMove();
    break;
  case remote::kOk:
    mvmt.Fire();
    break;
  case remote::kStar:
    mvmt.ToggleTracking();
    break;
  default:
    logger::Log.Log(logger::kWarning, "unknown command");
    break;
  }
  return -1;
};

void Setup() {
  Serial.begin(9600);
  logger::Log.SetDebug(kDebug);
  logger::Log.Init();
  logger::Log.ClearLogs();
  last_micros = micros();

  mvmt.Init(last_micros);
  remote.Init();
  remote.RegisterHandler(HandleRemoteCommands);
}

void Loop() {
  uint64_t current_micros = micros();
  uint64_t dt = current_micros - last_micros;
  last_micros = current_micros;

  remote.DoRemote(current_micros);
  mvmt.Track(dt);
  mvmt.Update(current_micros);
}

} // namespace turret
