#ifndef REMOTE_H_
#define REMOTE_H_

#include "ring_buffer_queue.h"
#include <Arduino.h>
#include <cstddef>
#include <functional>

// IR transmission type
#define DECODE_NEC

namespace remote {

constexpr int kLeft = 0x8;
constexpr int kRight = 0x5A;
constexpr int kUp = 0x52;
constexpr int kDown = 0x18;
constexpr int kOk = 0x1C;
constexpr int kCmd1 = 0x45;
constexpr int kCmd2 = 0x46;
constexpr int kCmd3 = 0x47;
constexpr int kCmd4 = 0x44;
constexpr int kCmd5 = 0x40;
constexpr int kCmd6 = 0x43;
constexpr int kCmd7 = 0x7;
constexpr int kCmd8 = 0x15;
constexpr int kCmd9 = 0x9;
constexpr int kCmd0 = 0x19;
constexpr int kStar = 0x16;
constexpr int kHashtag = 0xD;

class Remote {
public:
  static constexpr int kMaxRegistrations = 8;
  static constexpr int32_t kMaxWaitMicros = 2000000;
  static constexpr size_t kCommandBufferSize = 8;

  Remote(int pin, uint32_t max_next_command_timeout_micros = 2000000)
      : max_next_command_timeout_micros_(max_next_command_timeout_micros),
        pin_(pin) {}

  Remote(const Remote &r) = delete;
  Remote &operator=(const Remote &r) = delete;

  void Init();

  // A function that takes an array of bytes, does some work based on the
  // content of those bytes, and returns a timeout for when the next code is
  // considered part of this buffer. RegisterHandler() returns the handler index
  // for this registrar. Returns -1 if registration is full.
  //
  // `handler` should return the micros after the current update that the
  // next update should be considered part of the same buffer. Returning -1 from
  // the handler uses the default.
  int RegisterHandler(std::function<int64_t(uint16_t *, size_t)> handler);

  // Add new code to the ring buffer. For each handler, check the last timeout
  // T. If less than T micros have passed since the last command or if T is none
  // undefined
  void DoRemote(uint64_t current_micros);

private:
  int32_t GetCommandDelay(size_t handler_idx) {
    if (handler_idx > registrations_ - 1) {
      return kMaxWaitMicros;
    }

    return next_command_timeout_[handler_idx] == -1
               ? kMaxWaitMicros
               : next_command_timeout_[handler_idx];
  }

  int pin_;
  // The timestamp at which the last command was received.
  uint64_t last_command_micros_ = 0;
  // Indexed by handler. The actual handler method to call.
  std::function<int32_t(uint16_t *, size_t)> handlers_[kMaxRegistrations];
  // Indexed by handler, value is the number of micors after the last command to
  // treat the next command as part of the same series.
  int32_t next_command_timeout_[kMaxRegistrations]{-1};
  // The current number of registrations.
  int registrations_ = 0;
  int32_t max_next_command_timeout_micros_ = 2000000;
  // Ring buffer to track commands.
  ring_buffer_queue::RingBufferQueue<uint16_t, kCommandBufferSize> buffer_;
  // The N most recent IR commands. Trimmed if no handlers need the last
  // command(s).
  uint16_t current_commands_[kCommandBufferSize]{};
  // Indexed by handler, value is the index of the deepest command to include.
  size_t command_depth_[kMaxRegistrations]{};
};

} // namespace remote

#endif // REMOTE_H_
