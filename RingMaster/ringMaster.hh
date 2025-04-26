/**
 * @file ringMaster.hh
 * @brief Lock-free ring buffer for concurrent producer/consumer scenarios
 *
 * Provides a fixed-size thread-safe ring buffer implementation using atomic
 * operations instead of locks. Optimized for single-producer/single-consumer
 * patterns with appropriate memory ordering guarantees.
 */
#pragma once
#include <atomic>
#include <cstddef>
#include <memory>

/** @brief Buffer capacity (must be power of 2) */
#define CAPACITY (8)
/** @brief Bit mask for fast modulo operations */
#define mask ((CAPACITY) - 1)

// Verify CAPACITY is power of 2 for efficient wrap-around
static_assert((CAPACITY & (CAPACITY - 1)) == 0,
              "CAPACITY must be a power of 2");

/**
 * @class RingMaster
 * @brief Thread-safe lock-free circular buffer with atomic operations
 *
 * Implements a fixed-size ring buffer with O(1) insertion and removal
 * operations. Uses atomic operations with proper memory ordering to ensure
 * thread safety without locks or mutexes. Uses power-of-2 sizing for
 * efficient modulo via bit masking.
 *
 * @tparam Q_TYPE Must be movable and support non-throwing move operations.
 *                Copy operations are not used for performance reasons.
 */
template <typename Q_TYPE>
class RingMaster {
 private:
  /**
   * @struct PaddedAtomic
   * @brief Cache-aligned atomic counter to prevent false sharing
   *
   * Ensures atomic variables occupy separate cache lines to avoid
   * performance degradation from false sharing between threads.
   */
  struct alignas(64) PaddedAtomic {
    /** @brief Atomic index counter */
    std::atomic<size_t> var;
    /** @brief Padding to fill cache line */
    char pad[64 - sizeof(std::atomic<size_t>)];
  };

  /** @brief Write position index (producer) */
  PaddedAtomic head_{0};
  /** @brief Read position index (consumer) */
  PaddedAtomic tail_{0};
  /** @brief Data storage array */
  alignas(64) Q_TYPE buffer_[CAPACITY];

 public:
  /**
   * @brief Initialize empty ring buffer
   */
  RingMaster() = default;

  /**
   * @brief Clean up resources
   */
  ~RingMaster();

  /**
   * @brief Add element to buffer if not full
   *
   * Thread-safe insertion operation that uses perfect forwarding
   * to support both lvalue and rvalue references.
   *
   * @tparam ENQ_TYPE Input parameter type (deduced automatically)
   * @param value Element to insert
   * @return true if insertion succeeded, false if buffer full
   */
  template <typename ENQ_TYPE>
  bool push(ENQ_TYPE&& value);

  /**
   * @brief Remove oldest element from buffer
   *
   * Thread-safe removal operation that extracts the oldest element
   * if available.
   *
   * @param out Reference to store the retrieved element
   * @return true if element retrieved, false if buffer empty
   */
  bool pop(Q_TYPE& out);

  /**
   * @brief Remove multiple elements at once
   *
   * Discards up to n elements from the buffer without retrieving them.
   * Thread-safe operation that handles partial availability.
   *
   * @param n Maximum number of elements to remove
   * @return Actual number of elements removed
   */
  size_t remove(size_t n);

  /**
   * @brief Reset buffer to empty state
   *
   * Resets head and tail indices to zero, effectively emptying the buffer.
   *
   * @warning Not thread-safe - only call when no concurrent access
   */
  void clear();

  /**
   * @brief Check if buffer contains no elements
   *
   * Thread-safe check that may return stale result during concurrent
   * operations due to race conditions.
   *
   * @return true if empty, false otherwise
   */
  bool isEmpty() const noexcept;

  /**
   * @brief Check if buffer cannot accept more elements
   *
   * Thread-safe check that may return stale result during concurrent
   * operations due to race conditions.
   *
   * @return true if full, false otherwise
   */
  bool isFull() const noexcept;

  /**
   * @brief Get current element count
   *
   * Thread-safe operation that may return stale result during concurrent
   * operations due to race conditions.
   *
   * @return Number of elements in buffer
   */
  size_t size() const noexcept;
};
