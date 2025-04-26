// turboSnap.hh

#include <turbojpeg.h> /**< TurboJPEG library providing high-performance JPEG compression/decompression APIs */

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <thread>

/**
 * @def YUV420
 * @brief YUV420 color space format alias for TurboJPEG
 * Represents the YUV 4:2:0 planar color space where chroma components are
 * subsampled by 2x horizontally and vertically
 */
#define YUV420 TJSAMP_420

/**
 * @def RGB888
 * @brief RGB color space format alias for TurboJPEG
 * Represents 24-bit RGB color space with 8 bits per component in R-G-B order
 */
#define RGB888 TJPF_RGB

/**
 * @def COMP_FLAGS
 * @brief Compression flags for TurboJPEG
 * Uses FASTDCT flag for faster but slightly less accurate DCT calculations in
 * compression
 */
#define COMP_FLAGS (TJFLAG_FASTDCT)

/**
 * @class TurboSnap
 * @brief High-performance RGB to JPEG conversion using TurboJPEG
 *
 * Implements a worker thread-based RGB frame to JPEG conversion pipeline using
 * the TurboJPEG library for optimal performance. The class manages memory
 * buffers, implements queue handling for both input and output, and provides
 * thread-safe operation.
 *
 * @tparam IN_QUE Queue type for RGB input frames (must support push/pop
 * operations)
 * @tparam OUT_QUE Queue type for JPEG output frames (must support push/pop
 * operations)
 */
template <typename IN_QUE, typename OUT_QUE>
class TurboSnap {
 private:
  // Variables:

  IN_QUE& inRGB_; /**< Reference to input queue containing RGB frames */
  OUT_QUE&
      outJPEG_;     /**< Reference to output queue for compressed JPEG frames */
  uint32_t width_;  /**< Width of input/output frame in pixels */
  uint32_t height_; /**< Height of input/output frame in pixels */
  uint16_t quality_;   /**< JPEG compression quality parameter (range: [0, 100],
                          higher = better quality) */
  std::thread worker_; /**< Worker thread handling the RGB to JPEG conversion
                          pipeline */
  std::atomic<bool> isRunning_;   /**< Thread synchronization flag indicating
                                     active processing state */
  tjhandle compressor_ = nullptr; /**< TurboJPEG compressor handle */

  // Pre-allocated Buffers:

  std::vector<unsigned char> rgbBuffer_; /**< Pre-allocated buffer for RGB frame
                                            data (size: 3 * width_ * height_) */
  std::vector<unsigned char>
      jpegBuffer_; /**< Pre-allocated buffer for compressed JPEG data (size: >=
                      tjBufSize(...)) */

  /**
   * @brief Main processing thread function for RGB to JPEG conversion
   *
   * Continuously dequeues RGB frames from input queue, performs TurboJPEG
   * compression, and enqueues the resulting JPEG data to the output queue.
   * Implements back-off strategy when queues are empty or full.
   */
  void conversionLoop_();

  /**
   * @struct rgbFrame_
   * @brief Container for RGB frame data and associated metadata
   *
   * Holds raw RGB pixel data along with frame dimensions and sequence
   * information
   */
  struct rgbFrame_ {
    uint32_t width;       /**< Frame width in pixels */
    uint32_t height;      /**< Frame height in pixels */
    uint64_t frameNumber; /**< Sequential frame identifier */
    std::vector<unsigned char>
        rgbData;    /**< Raw RGB pixel data (size: 3 * width * height bytes) */
    size_t rgbSize; /**< Total size of RGB data in bytes */
  };

  /**
   * @struct jpegFrame_
   * @brief Container for compressed JPEG data and associated metadata
   *
   * Holds compressed JPEG data along with size information and the original
   * frame sequence number
   */
  struct jpegFrame_ {
    std::vector<unsigned char> jpegData; /**< Compressed JPEG binary data */
    size_t jpegSize;      /**< Size of the compressed JPEG data in bytes */
    uint64_t frameNumber; /**< Original frame sequence number (preserved from
                             input) */
  };

 public:
  /**
   * @brief Constructs a TurboSnap converter with the specified parameters
   *
   * Initializes TurboJPEG compressor, allocates required memory buffers for RGB
   * and JPEG data, and launches the worker thread for background conversion
   * processing.
   *
   * @param rgbIn   Reference to the input queue for RGB frames
   * @param jpegOut Reference to the output queue for compressed JPEG frames
   * @param width   Frame width in pixels
   * @param height  Frame height in pixels
   * @param quality JPEG compression quality (0-100, default: 75)
   *                Higher values provide better image quality at the cost of
   * larger file size
   *
   * @throws std::runtime_error If TurboJPEG initialization fails
   */
  TurboSnap(IN_QUE& rgbIn, OUT_QUE& jpegOut, uint32_t width, uint32_t height,
            uint16_t quality = 75);

  /**
   * @brief Destructor for TurboSnap
   *
   * Gracefully stops the worker thread, waits for any pending operations to
   * complete, and frees all TurboJPEG resources.
   */
  ~TurboSnap();

};  // end of TurboSnap class
