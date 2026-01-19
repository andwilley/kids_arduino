#ifndef LOGGER_H_
#define LOGGER_H_

#include "Arduino.h"
#include <FS.h>
#include <LittleFS.h>
#include <cstring>

namespace logger {

enum LogLevel {
  kInfo,
  kWarning,
  kSevere,
};

class Logger {
public:
  Logger() {}

  Logger(const Logger &) = delete;
  void operator=(const Logger &) = delete;

  void SetDebug(bool debug) { debug_ = debug; }

  void Init();

  void Log(LogLevel level, const char *format, ...)
      __attribute__((format(printf, 3, 4)));

  void ClearLogs();

  static constexpr int kMaxLogLength = 128;
  static constexpr int kLevelLength = 5;
  // 1 is the null termination
  static constexpr int kEffectiveLogLength = kMaxLogLength - kLevelLength - 1;

private:
  bool debug_ = true;
  QueueHandle_t queue_;

  static void WriterTask_(void *param);
};

extern Logger Log;

} // namespace logger

#endif // LOGGER_H_
