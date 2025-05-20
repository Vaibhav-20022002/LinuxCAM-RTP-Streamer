#pragma once
/**
 * @brief Threaded JPEG converter using TurboJPEG for high-performance
 * compression of RGB frames.
 *
 * This header provides the TurboSnap class, a background threaded converter
 * that fetches raw RGB frames from an input queue, compresses them to JPEG, and
 * dispatches them into an output queue.
 */

#include <turbojpeg.h> /**< TurboJPEG library providing high-performance JPEG compression/decompression APIs */

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

// ----------------------------------------------------
// [Definitions]: Macros for pixel format and compression flags
// ----------------------------------------------------

/**
 * @def YUV420
 * @brief Alias for TurboJPEG YUV 4:2:0 subsampling format
 */
#define YUV420 TJSAMP_420

/**
 * @def RGB888
 * @brief Alias for TurboJPEG 24-bit RGB input pixel format
 */
#define RGB888 TJPF_RGB

/**
 * @def COMP_FLAGS
 * @brief Compression mode flags for TurboJPEG compressor
 *
 * Use FASTDCT (fast DCT algorithm) for speed at slight quality cost.
 */
#define COMP_FLAGS (TJFLAG_FASTDCT)

// ----------------------------------------------------
// [Class]: TurboSnap
// ----------------------------------------------------

/**
 * @class TurboSnap
 * @brief Threaded converter from RGB frames to JPEG format using TurboJPEG
 *
 * This templated class sets up a background thread to continuously fetch RGB
 * frames from an input queue `IN_QUE`, compress them to JPEG using TurboJPEG,
 * and dispatch resulting JPEG frames into an output queue `OUT_QUE`.
 *
 * @tparam IN_QUE  Type of input queue; must support `bool pop(Frame&)`
 * @tparam OUT_QUE Type of output queue; must support `bool push(const
 * jpegFrame_&)`
 */
template <typename IN_QUE, typename OUT_QUE>
class TurboSnap {
 private:
  // --------------------
  // Member variables
  // --------------------
  IN_QUE& inRGB_;    /**< Reference to input RGB queue */
  OUT_QUE& outJPEG_; /**< Reference to output JPEG queue */
  uint32_t width_;   /**< Expected width of frames */
  uint32_t height_;  /**< Expected height of frames */
  uint16_t quality_; /**< JPEG quality factor (0-100) */

  std::thread worker_;          /**< Worker thread running conversion loop */
  std::atomic<bool> isRunning_; /**< Thread control flag */

  tjhandle compressor_; /**< TurboJPEG compression handle */

  std::vector<unsigned char> rgbBuffer_;  /**< Staging buffer for RGB frames */
  std::vector<unsigned char> jpegBuffer_; /**< Output buffer for JPEG data */

  /**
   * @brief Internal thread loop that performs RGB -> JPEG compression.
   */
  void conversionLoop_();

  /**
   * @struct rgbFrame_
   * @brief Helper struct for staging RGB frame data (unused directly in current
   * implementation).
   */
  struct rgbFrame_ {
    uint32_t width;
    uint32_t height;
    uint64_t frameNumber;
    std::vector<unsigned char> rgbData;
    size_t rgbSize;
  };

 public:
  // --------------------
  // Constructor / Destructor
  // --------------------

  /**
   * @brief Constructor.
   * @param rgbIn Input RGB frame queue (must support pop()).
   * @param jpegOut Output JPEG frame queue (must support push()).
   * @param width Expected width of frames.
   * @param height Expected height of frames.
   * @param quality JPEG quality (default: 75).
   * @throws std::runtime_error if TurboJPEG initialization fails.
   */
  TurboSnap(IN_QUE& rgbIn, OUT_QUE& jpegOut, uint32_t width, uint32_t height,
            uint16_t quality = 75);

  /**
   * @brief Destructor.
   * Signals the background thread to stop and cleans up resources.
   */
  ~TurboSnap();
};

// --------------------
// [Implementation]: TurboSnap
// --------------------

template <typename IN_QUE, typename OUT_QUE>
TurboSnap<IN_QUE, OUT_QUE>::TurboSnap(IN_QUE& rgbIn, OUT_QUE& jpegOut,
                                      uint32_t width, uint32_t height,
                                      uint16_t quality)
    : inRGB_(rgbIn),
      outJPEG_(jpegOut),
      width_(width),
      height_(height),
      quality_(quality),
      isRunning_(true) {
  compressor_ = tjInitCompress();
  if (!compressor_) {
    throw std::runtime_error("TurboJPEG initialization failed: " +
                             std::string(tjGetErrorStr()));
  }

  rgbBuffer_.resize(3ULL * width_ * height_);  ///< RGB 3 bytes per pixel
  jpegBuffer_.resize(tjBufSize(
      width_, height_, YUV420));  ///< Estimated worst case JPEG buffer size

  worker_ = std::thread(&TurboSnap::conversionLoop_, this);
}

template <typename IN_QUE, typename OUT_QUE>
TurboSnap<IN_QUE, OUT_QUE>::~TurboSnap() {
  isRunning_.store(false, std::memory_order_release);
  if (worker_.joinable()) {
    worker_.join();
  }
  tjDestroy(compressor_);
}

template <typename IN_QUE, typename OUT_QUE>
void TurboSnap<IN_QUE, OUT_QUE>::conversionLoop_() {
  while (isRunning_.load(std::memory_order_acquire)) {
    Frame input;

    if (inRGB_.pop(input)) {
      // Frame dimension check
      if (input.width != width_ || input.height != height_) {
        std::cerr << "Frame resolution mismatch: input=" << input.width << "x"
                  << input.height << ", expected=" << width_ << "x" << height_
                  << "\n";
        continue;
      }

      size_t jpegOutSize = jpegBuffer_.size();
      unsigned char* jpegBufPtr = jpegBuffer_.data();

      int ret = tjCompress2(compressor_, input.rawData.data(), width_, 0,
                            height_, RGB888, &jpegBufPtr, &jpegOutSize, YUV420,
                            quality_, COMP_FLAGS);

      if (ret != 0) {
        std::cerr << "TurboJPEG compression error: " << tjGetErrorStr() << "\n";
        continue;
      }

      jpegFrame_ output;
      output.frameNumber = input.sequenceNumber;
      output.jpegSize = jpegOutSize;
      output.jpegData.assign(jpegBuffer_.begin(),
                             jpegBuffer_.begin() + jpegOutSize);

      // Retry push if queue is full
      while (isRunning_.load() && !outJPEG_.push(output)) {
        std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
    } else {
      // No input: Yield to avoid busy spin
      std::this_thread::yield();
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  }
}
