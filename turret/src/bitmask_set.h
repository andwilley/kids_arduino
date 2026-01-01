#ifndef TURRET_BITMASK_SET_H_
#define TURRET_BITMASK_SET_H_

#include <cstring>
#include <stddef.h>
#include <stdint.h>

namespace turret {

template <size_t S> class BitmaskSet {
public:
  bool Contains(size_t index);

  void Set(size_t index, bool high = true);

  void Clear() { memset(mask, 0, sizeof(mask)); }

private:
  uint64_t mask[(S + 63) / 64] = {};
};

template <size_t S> bool BitmaskSet<S>::Contains(size_t index) {
  if (index >= S) {
    return false;
  }
  return (mask[index / 64] & 1ULL << (index % 64)) != 0;
}

template <size_t S> void BitmaskSet<S>::Set(size_t index, bool high) {
  if (index >= S) {
    return;
  }
  uint64_t &word = mask[index / 64];
  if (high) {
    word |= 1ULL << (index % 64);
  } else {
    word &= ~(1ULL << (index % 64));
  }
}

} // namespace turret

#endif // TURRET_BITMASK_SET_H_
