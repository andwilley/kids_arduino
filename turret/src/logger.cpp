#include "logger.h"
#include "Arduino.h"
#include <FS.h>
#include <LittleFS.h>
#include <cstring>

namespace logger {

// Global logger instance definition (declared as extern in logger.h)
Logger Log;

void Logger::Init() {
  if (!LittleFS.begin()) {
    return;
  }

  queue_ = xQueueCreate(50, kMaxLogLength);
  xTaskCreatePinnedToCore(WriterTask_, "LogWriter",
                          /* stack_size= */ 8192, this,
                          /* priority= */ 1,
                          /* task_handle= */ NULL,
                          /* core= */ 0);
}

void Logger::WriterTask_(void *param) {
  Logger *self = (Logger *)param;
  String msg;
  char buffer[kMaxLogLength];

  File f = LittleFS.open("/log.txt", "a");

  while (true) {
    if (xQueueReceive(self->queue_, &buffer, portMAX_DELAY)) {
      if (!f) {
        f = LittleFS.open("/log.txt", "a");
      }
      if (f) {
        f.println(buffer);
        // Consider batching flushes
        f.flush();
      }
    }
  }
}

void Logger::Log(LogLevel level, const char *format, ...) {
  if (!debug_ && level != kSevere) {
    return;
  }
  char buffer[kMaxLogLength];

  // MUST not exceed kLevelLength
  char *level_prefix;
  switch (level) {
  case kInfo:
    level_prefix = "INFO ";
    break;
  case kWarning:
    level_prefix = "WARN ";
    break;
  case kSevere:
    level_prefix = "CRIT ";
    break;
  default:
    level_prefix = "UNKN ";
    break;
  }

  memcpy(buffer, level_prefix, kLevelLength);

  va_list args;
  va_start(args, format);
  vsnprintf(buffer + kLevelLength, kMaxLogLength - kLevelLength, format, args);
  va_end(args);

  xQueueSend(queue_, &buffer, 0);
}

void Logger::ClearLogs() {
  if (LittleFS.remove("/log.txt")) {
    Log(kInfo, "Log file cleared");
  }
}

} // namespace logger
