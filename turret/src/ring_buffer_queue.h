#ifndef RING_BUFFER_QUEUE_H_
#define RING_BUFFER_QUEUE_H_

#include <cstring>
#include <stddef.h>

namespace ring_buffer_queue {

// A ring buffer queue. If Enqueue is used when the queue if full, data will be
// overwritten.
template <typename T, size_t S> class RingBufferQueue {
public:
  RingBufferQueue() : head_(0), tail_(0) {};

  // If called when the queue is full, this will overwrite the current head.
  // Returns true if head was overwritten.
  bool Enqueue(T item);

  // Returns false if the buffer is empty.
  bool PopFront(T &item);

  bool Empty() const;

  bool Full() const;

  void Clear();

  // Fills dest with the buffer elements starting from tail. Returns the actual
  // number of elements copied.
  size_t Stack(T *dest) const;

private:
  // First item in the queue, occupied unless the queue is empty
  size_t head_ = 0;
  // The next empty slot in the queue
  size_t tail_ = 0;
  size_t count_ = 0;
  T buffer_[S]{};

  size_t Wrap(size_t index) const { return (index % S + S) % S; }
};

template <typename T, size_t S>
size_t RingBufferQueue<T, S>::Stack(T *dest) const {
  if (count_ == 0) {
    return 0;
  }

  size_t current_idx = (tail_ == 0) ? S - 1 : tail_ - 1;

  for (size_t i = 0; i < count_; ++i) {
    dest[i] = buffer_[current_idx];
    current_idx = Wrap(current_idx - 1);
  }
  return count_;
}

template <typename T, size_t S> bool RingBufferQueue<T, S>::Enqueue(T item) {
  bool overwrite = false;
  if (Full()) {
    head_ = Wrap(head_ + 1);
    overwrite = true;
  } else {
    ++count_;
  }
  buffer_[tail_] = item;
  tail_ = Wrap(tail_ + 1);
  return overwrite;
}

template <typename T, size_t S> bool RingBufferQueue<T, S>::PopFront(T &item) {
  if (count_ == 0) {
    return false;
  }
  item = buffer_[head_];
  head_ = Wrap(head_ + 1);
  --count_;
  return true;
}

template <typename T, size_t S> void RingBufferQueue<T, S>::Clear() {
  head_ = 0;
  tail_ = 0;
  count_ = 0;
}

template <typename T, size_t S> bool RingBufferQueue<T, S>::Empty() const {
  return count_ == 0;
}

template <typename T, size_t S> bool RingBufferQueue<T, S>::Full() const {
  return count_ == S;
}

} // namespace ring_buffer_queue

#endif // RING_BUFFER_QUEUE_H_
