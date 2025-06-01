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

/**
 * @brief MJPEG frame with metadata
 */
struct MjpegFrame {
  std::vector<uint8_t> jpegData;          /**< Compressed JPEG data */
  uint32_t width;                         /**< Frame width in pixels */
  uint32_t height;                        /**< Frame height in pixels */
  uint64_t sequenceNumber;                /**< Unique frame sequence number */
  std::chrono::microseconds timestamp;    /**< Timestamp of capture */
};
