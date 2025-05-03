#pragma once
#include <chrono>  // For timestamp
#include <cstdint>
#include <vector>

/**
 * @brief RGB frame from camera
 */
struct Frame {
  uint32_t width;
  uint32_t height;
  uint64_t sequenceNumber;
  std::chrono::microseconds timestamp;
  std::vector<uint8_t> rawData;
};

/**
 * @brief JPEG frame for streaming
 */
struct jpegFrame_ {
  std::vector<uint8_t> jpegData;
  size_t jpegSize;
  uint64_t frameNumber;
};
