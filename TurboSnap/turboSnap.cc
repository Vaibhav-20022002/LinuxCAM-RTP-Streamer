#include "turboSnap.hh"

/**
 * @brief Constructs and initializes the TurboSnap converter
 *
 * Initializes the TurboJPEG compression engine, allocates memory for the input
 * and output buffers, and launches the background worker thread that processes
 * RGB frames into JPEG format. All resources are allocated during construction
 * to avoid runtime allocations during conversion.
 *
 * @tparam IN_QUE Queue type for RGB input frames
 * @tparam OUT_QUE Queue type for JPEG output frames
 * @param rgbIn Reference to the queue providing RGB frames for conversion
 * @param jpegOut Reference to the queue where converted JPEG frames will be
 * placed
 * @param width Width of the frames in pixels
 * @param height Height of the frames in pixels
 * @param quality JPEG compression quality factor (0-100), higher values yield
 * better quality at larger sizes
 *
 * @throws std::runtime_error If TurboJPEG compressor initialization fails
 */
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
  // Create the TurboJPEG compressor instance
  compressor_ = tjInitCompress();
  if (!compressor_) {
    throw std::runtime_error("TurboJPEG init failed: " +
                             std::string(tjGetErrorStr()));
  }

  // Pre-allocate buffers once to avoid runtime allocations during processing
  rgbBuffer_.resize(3 * width_ * height_);  // RGB = 3 bytes per pixel
  jpegBuffer_.resize(tjBufSize(
      width_, height_,
      YUV420));  // Conservative allocation based on TurboJPEG requirements

  // Launch the worker thread for background processing
  worker_ = std::thread(&TurboSnap::conversionLoop_, this);
}

/**
 * @brief Destructor that cleans up resources and ensures proper shutdown
 *
 * Signals the worker thread to stop, waits for pending operations to complete,
 * then releases the TurboJPEG compressor resources.
 *
 * @tparam IN_QUE Queue type for RGB input frames
 * @tparam OUT_QUE Queue type for JPEG output frames
 */
template <typename IN_QUE, typename OUT_QUE>
TurboSnap<IN_QUE, OUT_QUE>::~TurboSnap() {
  // Signal worker thread to stop
  isRunning_.store(false, std::memory_order_release);

  // Wait for worker thread to finish any pending operations
  if (worker_.joinable()) {
    worker_.join();
  }

  // Free TurboJPEG resources
  tjDestroy(compressor_);
}

/**
 * @brief Main processing loop that converts RGB frames to JPEG
 *
 * This method runs in a dedicated thread and continuously:
 *
 * 1. Retrieves RGB frames from the input queue
 *
 * 2. Validates frame dimensions
 *
 * 3. Compresses the RGB data to JPEG format using TurboJPEG
 *
 * 4. Places the compressed JPEG data into the output queue
 *
 * The method implements backoff strategies when queues are empty or full,
 * and handles error conditions during compression.
 *
 * @tparam IN_QUE Queue type for RGB input frames
 * @tparam OUT_QUE Queue type for JPEG output frames
 */
template <typename IN_QUE, typename OUT_QUE>
void TurboSnap<IN_QUE, OUT_QUE>::conversionLoop_() {
  while (isRunning_.load(std::memory_order_acquire)) {
    rgbFrame_ input;
    if (inRGB_.pop(input)) {
      // Validate frame dimensions match expected configuration
      if (input.width != width_ || input.height != height_) {
        std::cerr << "Frame resolution mismatch\n";
        continue;
      }

      // Track the output size of compressed JPEG
      size_t jpegOutSize = jpegBuffer_.size();

      // Perform the actual RGB to JPEG compression using TurboJPEG
      if (0 != tjCompress2(compressor_, input.rgbData.data(), width_,
                           0 /* pitch */, height_, RGB888, jpegBuffer_.data(),
                           &jpegOutSize, YUV420, quality_, COMP_FLAGS)) {
        // Log error but continue processing other frames
        std::cerr << "Compression failed: " << tjGetErrorStr() << "\n";
        continue;
      }

      // Prepare output structure with compressed data
      jpegFrame_ output;
      output.jpegData.assign(jpegBuffer_.begin(),
                             jpegBuffer_.begin() + jpegOutSize);
      output.jpegSize = jpegOutSize;
      output.frameNumber = input.frameNumber;

      // Enqueue to JPEG output buffer with retry logic if the queue is full
      while (isRunning_.load(std::memory_order_relaxed) &&
             !outJPEG_.push(output)) {
        // Back off using thread yield and short sleep to reduce CPU usage
        std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
    } else {
      // No input available; implement backoff strategy
      std::this_thread::yield();
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  }
}
