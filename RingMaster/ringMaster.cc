/**
 * @file ringMaster.cc
 * @brief Implementation of RingMaster lock-free ring buffer
 *
 * Contains method implementations for the thread-safe ring buffer
 * with appropriate memory ordering constraints for correct concurrent
 * behavior.
 */
#include "ringMaster.hh"

/**
 * @brief Clean up resources
 *
 * @tparam Q_TYPE Element type stored in the buffer
 */
template <typename Q_TYPE>
RingMaster<Q_TYPE>::~RingMaster() {
  clear();
}

/**
 * @brief Add element to buffer if not full
 *
 * Uses relaxed load for head (producer-only) and acquire load for tail
 * to synchronize with consumer. Forwards the value to support efficient
 * move semantics when available. Uses release store to publish changes
 * to the consumer.
 *
 * @tparam Q_TYPE Element type stored in the buffer
 * @tparam ENQ_TYPE Input parameter type
 * @param value Element to insert
 * @return true if insertion succeeded, false if buffer full
 */
template <typename Q_TYPE>
template <typename ENQ_TYPE>
bool RingMaster<Q_TYPE>::push(ENQ_TYPE&& value) {
  const size_t head = head_.var.load(std::memory_order_relaxed);
  const size_t tail = tail_.var.load(std::memory_order_acquire);

  if ((head - tail) >= CAPACITY) {
    return false;  // Buffer full
  }

  buffer_[head & mask] = std::forward<ENQ_TYPE>(value);
  head_.var.store(head + 1, std::memory_order_release);
  return true;
}

/**
 * @brief Remove oldest element from buffer
 *
 * Uses relaxed load for tail (consumer-only) and acquire load for head
 * to synchronize with producer. Uses move semantics when retrieving
 * element to improve performance when Q_TYPE supports it. Uses release
 * store to publish changes to the producer.
 *
 * @tparam Q_TYPE Element type stored in the buffer
 * @param popOut Reference to store the retrieved element
 * @return true if element retrieved, false if buffer empty
 */
template <typename Q_TYPE>
bool RingMaster<Q_TYPE>::pop(Q_TYPE& popOut) {
  const size_t tail = tail_.var.load(std::memory_order_relaxed);
  const size_t head = head_.var.load(std::memory_order_acquire);

  if (tail == head) {
    return false;  // Buffer empty
  }

  popOut = std::move(buffer_[tail & mask]);
  tail_.var.store(tail + 1, std::memory_order_release);
  return true;
}

/**
 * @brief Remove multiple elements at once
 *
 * Calculates available elements and removes at most n elements.
 * Uses appropriate memory ordering for thread safety.
 *
 * @tparam Q_TYPE Element type stored in the buffer
 * @param n Maximum number of elements to remove
 * @return Actual number of elements removed
 */
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

/**
 * @brief Reset buffer to empty state
 *
 * Sets both head and tail indices to zero. Uses relaxed memory
 * ordering as this method is not intended for concurrent use.
 *
 * @warning Not thread-safe - only call when no concurrent access
 *
 * @tparam Q_TYPE Element type stored in the buffer
 */
template <typename Q_TYPE>
void RingMaster<Q_TYPE>::clear() {
  head_.var.store(0, std::memory_order_relaxed);
  tail_.var.store(0, std::memory_order_relaxed);
}

/**
 * @brief Check if buffer contains no elements
 *
 * A buffer is empty when head equals tail. Uses acquire memory
 * ordering to ensure visibility of concurrent operations.
 *
 * @tparam Q_TYPE Element type stored in the buffer
 * @return true if empty, false otherwise
 */
template <typename Q_TYPE>
bool RingMaster<Q_TYPE>::isEmpty() const noexcept {
  return (head_.var.load(std::memory_order_acquire) ==
          tail_.var.load(std::memory_order_acquire));
}

/**
 * @brief Check if buffer cannot accept more elements
 *
 * A buffer is full when the difference between head and tail
 * equals or exceeds capacity. Uses acquire memory ordering
 * to ensure visibility of concurrent operations.
 *
 * @tparam Q_TYPE Element type stored in the buffer
 * @return true if full, false otherwise
 */
template <typename Q_TYPE>
bool RingMaster<Q_TYPE>::isFull() const noexcept {
  return (head_.var.load(std::memory_order_acquire) -
          tail_.var.load(std::memory_order_acquire)) >= CAPACITY;
}

/**
 * @brief Get current element count
 *
 * Calculates difference between head and tail indices.
 * Uses acquire memory ordering to ensure visibility of
 * concurrent operations.
 *
 * @tparam Q_TYPE Element type stored in the buffer
 * @return Number of elements in buffer
 */
template <typename Q_TYPE>
size_t RingMaster<Q_TYPE>::size() const noexcept {
  return (head_.var.load(std::memory_order_acquire) -
          tail_.var.load(std::memory_order_acquire));
}
