#ifndef BITMASK_SET_H_
#define BITMASK_SET_H_

#include <cstring>
#include <stddef.h>
#include <stdint.h>

namespace bitmask_set {

template <size_t S> class BitmaskSet {
public:
  bool Contains(size_t index) const;

  void Set(size_t index, bool high = true);

  void Clear() { memset(mask_, 0, sizeof(mask_)); }

private:
  uint64_t mask_[(S + 63) / 64] = {};
};

template <size_t S> bool BitmaskSet<S>::Contains(size_t index) const {
  if (index >= S) {
    return false;
  }
  return (mask_[index / 64] & 1ULL << (index % 64)) != 0;
}

template <size_t S> void BitmaskSet<S>::Set(size_t index, bool high) {
  if (index >= S) {
    return;
  }
  uint64_t &word = mask_[index / 64];
  if (high) {
    word |= 1ULL << (index % 64);
  } else {
    word &= ~(1ULL << (index % 64));
  }
}

} // namespace bitmask_set

#endif // BITMASK_SET_H_
