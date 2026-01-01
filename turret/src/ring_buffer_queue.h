#ifndef TURRET_RING_BUFFER_QUEUE_H_
#define TURRET_RING_BUFFER_QUEUE_H_

#include <stddef.h>

namespace turret {

// A ring buffer de/queue.
template <typename T, size_t S> class RingBufferQueue {
public:
  RingBufferQueue() : head_(0), tail_(0) {};

  bool Enqueue(T item);

  T Dequeue();

  bool Empty();

  bool Full();

  void Clear();

private:
  // First item in the queue, occupied unless the queue is empty
  size_t head_;
  // The next empty slot in the queue
  size_t tail_;
  T queue_[S];

  size_t wrap(size_t index) { return index % S; }
};

template <typename T, size_t S> bool RingBufferQueue<T, S>::Enqueue(T item) {
  if (Full()) {
    return false;
  }
  queue_[tail_] = item;
  tail_ = wrap(tail_ + 1);
  return true;
}

// UB if the queue is empty, caller must verify.
template <typename T, size_t S> T RingBufferQueue<T, S>::Dequeue() {
  T item = queue_[head_];
  head_ = wrap(head_ + 1);
  return item;
}

template <typename T, size_t S> void RingBufferQueue<T, S>::Clear() {
  head_ = 0;
  tail_ = 0;
}

template <typename T, size_t S> bool RingBufferQueue<T, S>::Empty() {
  return head_ == tail_;
}

template <typename T, size_t S> bool RingBufferQueue<T, S>::Full() {
  return wrap(tail_ + 1) == head_;
}

} // namespace turret

#endif // TURRET_RING_BUFFER_QUEUE_H_
