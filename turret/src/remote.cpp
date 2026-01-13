#include "remote.h"

#include <IRremote.hpp>

namespace remote {

int Remote::RegisterHandler(
    std::function<uint32_t(uint64_t *, size_t)> handler) {
  if (registrations_ == kMaxRegistrations - 1) {
    return -1;
  }

  handlers_[registrations_] = handler;
  return registrations_++;
}

void Remote::DoRemote(uint64_t current_micros) {
  if (!IrReceiver.decode() ||
      (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    return;
  }

  uint16_t command = IrReceiver.decodedIRData.command;
  buffer_.Enqueue(command);
  size_t elems = buffer_.Stack(current_commands_);
  for (size_t handler_idx = 0; handler_idx < registrations_; ++handler_idx) {
    uint64_t command_micros = GetCommandDelay(handler_idx);
    if (last_command_micros_ + command_micros > current_micros) {
      command_depth_[handler_idx] = 0;
    } else {
      ++command_depth_[handler_idx];
    }
    next_command_timeout_[handler_idx] =
        handlers_[handler_idx](current_commands_, command_depth_[handler_idx]);
  }
  IrReceiver.resume();
}

} // namespace remote
