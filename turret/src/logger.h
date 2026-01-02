#ifndef TURRET_LOGGER_H_
#define TURRET_LOGGER_H_

#include "Arduino.h"
#include <FS.h>
#include <LittleFS.h>
#include <cstring>

namespace turret {

enum LogLevel {
  kInfo,
  kWarning,
  kSevere,
};

class Logger {
public:
  Logger(bool debug = true) : debug_(debug) {}

  Logger(const Logger &) = delete;
  void operator=(const Logger &) = delete;

  void Init();

  void Log(LogLevel level, const char *format, ...)
      __attribute__((format(printf, 3, 4)));

  static constexpr int kMaxLogLength = 128;
  static constexpr int kLevelLength = 5;
  // 1 is the null termination
  static constexpr int kEffectiveLogLength = kMaxLogLength - kLevelLength - 1;

private:
  bool debug_;
  QueueHandle_t queue_;

  static void WriterTask_(void *param);
};

extern Logger Log;

} // namespace turret

#endif // TURRET_LOGGER_H_
