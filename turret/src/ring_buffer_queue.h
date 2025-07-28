#ifndef RING_BUFFER_QUEUE
#define RING_BUFFER_QUEUE

#include <stddef.h>

// A ring buffer de/queue.
template <typename T, size_t S> class RingBufferQueue {
public:
  RingBufferQueue() : head(T(0)), tail(T(0)){};

  bool Enqueue(T item);

  T Dequeue();

  bool Empty();

  bool Full();

  void Clear();

private:
  size_t head;
  size_t tail;
  T queue[S];

  size_t wrap(size_t index) { return index % S; }
};

template <typename T, size_t S> bool RingBufferQueue<T, S>::Enqueue(T item) {
  if (Full()) {
    return false;
  }
  size_t next_tail = wrap(tail + 1);
  queue[tail] = item;
  tail = next_tail;
  return true;
}

// UB if the queue is empty, caller must verify.
template <typename T, size_t S> T RingBufferQueue<T, S>::Dequeue() {
  T item = queue[head];
  head = wrap(head + 1);
  return item;
}

template <typename T, size_t S> void RingBufferQueue<T, S>::Clear() {
  head = 0;
  tail = 0;
}

template <typename T, size_t S> bool RingBufferQueue<T, S>::Empty() {
  return head == tail;
}

template <typename T, size_t S> bool RingBufferQueue<T, S>::Full() {
  return wrap(tail + 1) == head;
}

#endif // RING_BUFFER_QUEUE
