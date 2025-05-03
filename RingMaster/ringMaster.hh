#pragma once
#include <atomic>
#include <cstddef>
#include <memory>

/**
 * @brief Lock-free ring buffer implementation for high-performance
 * producer/consumer workflows
 *
 * This header defines the RingMaster template class, providing a
 * fixed-capacity, cache-aligned, lock-free circular buffer. It targets
 * single-producer/single-consumer use cases, minimizing overhead by avoiding
 * locks and employing atomic operations with appropriate memory ordering to
 * guarantee thread safety.
 *
 * @section Assumptions
 * - Capacity is compile-time constant and a power of two for efficient indexing.
 * - Only one thread calls push(), and only one thread calls pop().
 * - Type Q_TYPE supports nothrow move construction and assignment; copying is avoided.
 *
 * @section Usage
 * @code
 * RingMaster<MyType> buffer;
 * MyType item;
 * if (buffer.push(std::move(item))) {
 *     // pushed successfully
 * }
 * MyType out;
 * if (buffer.pop(out)) {
 *     // popped successfully
 * }
 * @endcode
 *
 * @tparam Q_TYPE Element type stored in the ring; must be movable without throwing.
 */

#define CAPACITY                                                               \
  8 /**< Capacity of the ring buffer; must be a power of two for efficient indexing. */
#define MASK                                                                 \
  (CAPACITY - 1) /**< Mask for wrapping indices efficiently via bitwise AND. */

/**
 * @class RingMaster
 * @brief Lock-free circular buffer for single-producer/single-consumer patterns
 *
 * RingMaster provides constant-time push and pop operations with minimal latency.
 * Internally, it uses two atomic counters (head_ and tail_) padded to
 * cache lines to prevent false sharing. The buffer array is also aligned
 * to 64-byte boundaries.
 *
 * @note This class is NOT safe for multiple concurrent producers or consumers.
 * @warning clear() is not thread-safe; only call when no push/pop is in flight.
 */
template <typename Q_TYPE>
class RingMaster {
private:
  /**
   * @struct PaddedAtomic
   * @brief Cache-aligned atomic counter to avoid false sharing
   *
   * Each atomic variable is padded to occupy its own cache line,
   * preventing contention between threads operating on head_ and tail_.
   */
  struct alignas(64) PaddedAtomic {
    std::atomic<size_t> var; /**< Atomic index counter (head or tail) */
    char pad[64 - sizeof(std::atomic<size_t>)]; /**< Padding to complete cache line size */
  };

  PaddedAtomic head_{0}; /**< Producer index (next write position) */
  PaddedAtomic tail_{0}; /**< Consumer index (next read position) */
  alignas(64) Q_TYPE buffer_[CAPACITY]; /**< Storage for elements, cache-line aligned */

public:
  /**
   * @brief Default constructor initializes indices
   *
   * head_ and tail_ start at zero, indicating an empty buffer.
   */
  RingMaster() = default;

  /**
   * @brief Destructor cleans up resources
   *
   * No dynamic allocation is used; default behavior suffices.
   */
  ~RingMaster();

  /**
   * @brief Push an element into the ring buffer if space is available
   *
   * Uses atomic operations to update head position after storing the value.
   * Employs perfect forwarding to accept lvalues or rvalues efficiently.
   *
   * @tparam ENQ_TYPE Type deduced for insertion (should match Q_TYPE or convertible)
   * @param value Element to insert (forwarded)
   * @return true if insertion succeeded, false if buffer was full
   */
  template <typename ENQ_TYPE>
  bool push(ENQ_TYPE&& value);

  /**
   * @brief Pop the oldest element from the buffer
   *
   * Retrieves the element at tail_, moves it into `out`, then advances tail_.
   *
   * @param out Reference where the popped element is stored
   * @return true if an element was available, false if buffer was empty
   */
  bool pop(Q_TYPE& out);

  /**
   * @brief Discard up to n oldest elements without retrieval
   *
   * Advances tail_ by up to n positions, effectively removing elements.
   *
   * @param n Maximum number of elements to remove
   * @return Actual number of elements removed
   */
  size_t remove(size_t n);

  /**
   * @brief Reset buffer to empty state
   *
   * Sets both head_ and tail_ back to zero.
   *
   * @warning Not thread-safe. Only call when no concurrent push/pop operations.
   */
  void clear();

  /**
   * @brief Check if buffer is empty
   *
   * Compares head_ and tail_ atomically; may be slightly stale under concurrency.
   *
   * @return true if no elements are in the buffer, false otherwise.
   */
  bool isEmpty() const noexcept;

  /**
   * @brief Check if buffer is full
   *
   * True when advancing head_ would collide with tail_.
   *
   * @return true if no additional elements can be pushed, false otherwise.
   */
  bool isFull() const noexcept;

  /**
   * @brief Get approximate count of elements in buffer
   *
   * Computes difference between head_ and tail_; may be stale under heavy concurrency.
   *
   * @return Number of elements currently held in buffer
   */
  size_t size() const noexcept;
};

/*-------------------------------------------------
 * Implementation below (template definitions)
 *-------------------------------------------------*/

template <typename Q_TYPE>
RingMaster<Q_TYPE>::~RingMaster() {
  clear();
}

template <typename Q_TYPE>
template <typename ENQ_TYPE>
bool RingMaster<Q_TYPE>::push(ENQ_TYPE&& value) {
  const size_t head = head_.var.load(std::memory_order_relaxed);
  const size_t tail = tail_.var.load(std::memory_order_acquire);

  if ((head - tail) >= CAPACITY) {
    return false; // buffer full
  }

  buffer_[head & MASK] = std::forward<ENQ_TYPE>(value);
  head_.var.store(head + 1, std::memory_order_release);
  return true;
}

template <typename Q_TYPE>
bool RingMaster<Q_TYPE>::pop(Q_TYPE& out) {
  const size_t tail = tail_.var.load(std::memory_order_relaxed);
  const size_t head = head_.var.load(std::memory_order_acquire);

  if (tail == head) {
    return false; // buffer empty
  }

  out = std::move(buffer_[tail & MASK]);
  tail_.var.store(tail + 1, std::memory_order_release);
  return true;
}

template <typename Q_TYPE>
size_t RingMaster<Q_TYPE>::remove(size_t n) {
  const size_t tail = tail_.var.load(std::memory_order_relaxed);
  const size_t head = head_.var.load(std::memory_order_acquire);
  const size_t available = head - tail;
  const size_t toRemove = (n > available) ? available : n;

  if (toRemove > 0) {
    tail_.var.store(tail + toRemove, std::memory_order_release);
  }
  return toRemove;
}

template <typename Q_TYPE>
void RingMaster<Q_TYPE>::clear() {
  head_.var.store(0, std::memory_order_relaxed);
  tail_.var.store(0, std::memory_order_relaxed);
}

template <typename Q_TYPE>
bool RingMaster<Q_TYPE>::isEmpty() const noexcept {
  const size_t head = head_.var.load(std::memory_order_acquire);
  const size_t tail = tail_.var.load(std::memory_order_acquire);
  return head == tail;
}

template <typename Q_TYPE>
bool RingMaster<Q_TYPE>::isFull() const noexcept {
  const size_t head = head_.var.load(std::memory_order_acquire);
  const size_t tail = tail_.var.load(std::memory_order_acquire);
  return (head - tail) >= CAPACITY;
}

template <typename Q_TYPE>
size_t RingMaster<Q_TYPE>::size() const noexcept {
  const size_t head = head_.var.load(std::memory_order_acquire);
  const size_t tail = tail_.var.load(std::memory_order_acquire);
  return head - tail;
}

