#ifndef TURRET_REMOTE_H_
#define TURRET_REMOTE_H_

#include "ring_buffer_queue.h"
#include <Arduino.h>
#include <IRremote.hpp>
#include <cstddef>
#include <functional>

// IR transmission type
#define DECODE_NEC

namespace remote {

constexpr int left = 0x8;
constexpr int right = 0x5A;
constexpr int up = 0x52;
constexpr int down = 0x18;
constexpr int ok = 0x1C;
constexpr int cmd1 = 0x45;
constexpr int cmd2 = 0x46;
constexpr int cmd3 = 0x47;
constexpr int cmd4 = 0x44;
constexpr int cmd5 = 0x40;
constexpr int cmd6 = 0x43;
constexpr int cmd7 = 0x7;
constexpr int cmd8 = 0x15;
constexpr int cmd9 = 0x9;
constexpr int cmd0 = 0x19;
constexpr int star = 0x16;
constexpr int hashtag = 0xD;

class Remote {
public:
  static constexpr int kMaxRegistrations = 8;
  static constexpr int32_t kMaxWaitMicros = 2000000;
  static constexpr size_t kCommandBufferSize = 8;

  static Remote &Instance() {
    static Remote instance;
    return instance;
  }

  Remote(const Remote &r) = delete;
  Remote &operator=(const Remote &r) = delete;

  void Init(uint32_t max_next_command_timeout_micros = 2000000) {
    IrReceiver.begin(5, ENABLE_LED_FEEDBACK);
    max_next_command_timeout_micros_ = max_next_command_timeout_micros;
  }

  // A function that takes an array of bytes, does some work based on the
  // content of those bytes, and returns a timeout for when the next code is
  // considered part of this buffer. RegisterHandler() returns the handler index
  // for this registrar. Returns -1 if registration is full.
  // Consider returning optional or doing a bool with output param.
  // `handler` should return an the micros after the current update that the
  // next update should be considered part of the same buffer. Returning -1 uses
  // the default.
  int RegisterHandler(std::function<uint32_t(uint64_t *, size_t)> handler);

  // Add new code to the ring buffer. For each handler, check the last timeout
  // T. If less than T micros have passed since the last command or if T is none
  // undefined
  void DoRemote(uint64_t current_micros);

private:
  Remote() {}

  int32_t GetCommandDelay(size_t handler_idx) {
    if (handler_idx > registrations_ - 1) {
      return kMaxWaitMicros;
    }

    return next_command_timeout_[handler_idx] == -1
               ? kMaxWaitMicros
               : next_command_timeout_[handler_idx];
  }

  // The timestamp at which the last command was received.
  uint64_t last_command_micros_ = 0;
  // Indexed by handler. The actual handler method to call.
  std::function<uint64_t(uint64_t *, size_t)> handlers_[kMaxRegistrations];
  // Indexed by handler, value is the number of micors after the last command to
  // treat the next command as part of the same series.
  int32_t next_command_timeout_[kMaxRegistrations]{-1};
  // The current number of registrations.
  int registrations_ = 0;
  int32_t max_next_command_timeout_micors_ = 2000000;
  // Ring buffer to track commands.
  turret::RingBufferQueue<unit16_t, kCommandBufferSize> buffer_;
  // The N most recent IR commands. Trimmed if no handlers need the last
  // command(s).
  uint16_t current_commands_[kCommandBufferSize]{};
  // Indexed by handler, value is the index of the deepest command to include.
  size_t command_depth_[kMaxRegistrations]{};
};

} // namespace remote

#endif // TURRET_REMOTE_H_
