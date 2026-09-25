#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <utility>

namespace common {

/**
 * @brief A thread-safe triple buffer implementation for SPSC (Single Producer
 * Single Consumer) usage.
 *
 * This class manages three buffers:
 * 1. Write buffer: Owned by the producer.
 * 2. Read buffer: Owned by the consumer.
 * 3. Shared buffer: Holds the latest committed data.
 *
 * This ensures that the producer never blocks waiting for the consumer,
 * and the consumer always has access to the most recent data without tearing.
 */
template <typename T> class TripleBuffer {
public:
  TripleBuffer()
      : write_index_(0), read_index_(1), shared_index_(2), new_data_(false) {}

  explicit TripleBuffer(const T &initial_value)
      : write_index_(0), read_index_(1), shared_index_(2), new_data_(false) {
    buffers_[0] = initial_value;
    buffers_[1] = initial_value;
    buffers_[2] = initial_value;
  }

  /**
   * @brief Get pointer to the current write buffer.
   * Only the producer thread should call this.
   * @return Pointer to the write buffer.
   */
  T *get_write_buffer() { return &buffers_[write_index_]; }

  /**
   * @brief Publish the content of the write buffer.
   * This makes the data available to the consumer.
   */
  void push() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::swap(write_index_, shared_index_);
    new_data_.store(true, std::memory_order_release);
  }

  /**
   * @brief Write a value and publish it immediately.
   */
  void write(const T &value) {
    buffers_[write_index_] = value;
    push();
  }

  /**
   * @brief Check if new data is available without consuming it.
   */
  bool has_new_data() const {
    return new_data_.load(std::memory_order_acquire);
  }

  /**
   * @brief Update the read buffer if new data is available.
   * Only the consumer thread should call this.
   * @return true if the read buffer was updated (new data), false otherwise.
   */
  bool pull() {
    if (!new_data_.load(std::memory_order_acquire)) {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    // Check again inside lock
    if (!new_data_.load(std::memory_order_relaxed)) {
      return false;
    }

    std::swap(read_index_, shared_index_);
    new_data_.store(false, std::memory_order_release);
    return true;
  }

  /**
   * @brief Get pointer to the current read buffer.
   * Only the consumer thread should call this.
   * @return Pointer to the read buffer.
   */
  const T *get_read_buffer() const { return &buffers_[read_index_]; }

  /**
   * @brief Read the current data.
   * Updates the internal read buffer if new data is available.
   * @param value[out] Destination to copy the data to.
   * @return true if new data was fetched, false if using old data.
   */
  bool read(T &value) {
    bool updated = pull();
    value = buffers_[read_index_];
    return updated;
  }

private:
  std::array<T, 3> buffers_;

  // Index management
  int write_index_;  // Accessed only by producer
  int read_index_;   // Accessed only by consumer
  int shared_index_; // Protected by mutex

  std::atomic<bool> new_data_;
  std::mutex mutex_; // Protects shared_index_ and ensures atomic swap
};

} // namespace common
