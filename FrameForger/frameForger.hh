/**
 * @brief Camera frame acquisition and processing system
 *
 * @details The FrameForger class provides a high-level interface for camera
 * operations, including initialization, configuration, frame capture, and
 * buffer management. It uses libcamera to interface with camera hardware and
 * provides processed frames through a ring buffer mechanism.
 */
#pragma once

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/stream.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "frameTypes.hh"
#include "ringMaster.hh"

/**
 * @class FrameForger
 * @brief Main camera handling class responsible for frame acquisition and
 * processing
 *
 * @details The FrameForger class encapsulates the entire camera workflow:
 * - Camera discovery and selection
 * - Stream configuration and setup
 * - Buffer allocation and memory management
 * - Frame capture and processing
 * - Delivery of processed frames through a ring buffer
 *
 * It uses libcamera as the backend for camera operations and provides a
 * simplified interface for application code to access camera frames.
 */
class FrameForger {
 public:
  /**
   * @enum ErrorCode
   * @brief Error codes returned by FrameForger methods
   *
   * @details These error codes indicate the specific stage of the
   * initialization or capture process where a failure occurred, allowing for
   * precise error handling and diagnostics.
   */
  enum ErrorCode {
    SUCCESS = 0,                   /**< Operation completed successfully */
    ERR_CAMERA_MANAGER_START = -1, /**< Failed to start camera manager */
    ERR_NO_CAMERAS = -2,           /**< No cameras available on the system */
    ERR_CAMERA_ACQUIRE = -3,    /**< Failed to acquire camera (may be in use) */
    ERR_CONFIG_GENERATION = -4, /**< Failed to generate camera configuration */
    ERR_CONFIG_VALIDATION = -5, /**< Camera configuration validation failed */
    ERR_CONFIG_APPLICATION = -6, /**< Failed to apply camera configuration */
    ERR_BUFFER_ALLOCATION = -7,  /**< Failed to allocate frame buffers */
    ERR_REQUEST_CREATION = -8,   /**< Failed to create capture request */
    ERR_REQUEST_BUFFER = -9,     /**< Failed to add buffer to request */
    ERR_CAMERA_START = -10,      /**< Failed to start camera capture */
    ERR_QUEUE_REQUEST = -11      /**< Failed to queue capture request */
  };

  /**
   * @brief Constructor that initializes the FrameForger with an external buffer
   *
   * @param rgbBuffer Reference to a ring buffer for storing captured frames
   *
   * @details Initializes the camera manager but does not start any camera
   * operations. The full initialization sequence must be triggered by calling
   * initialize().
   */
  explicit FrameForger(RingMaster<Frame>& rgbBuffer);

  /**
   * @brief Destructor that ensures proper cleanup of all resources
   *
   * @details Automatically stops any ongoing capture and releases all allocated
   * resources, including camera hardware, memory mappings, and buffer
   * allocations.
   */
  ~FrameForger();

  /**
   * @brief Initializes the camera system in preparation for capture
   *
   * @return ErrorCode indicating success or the specific failure point
   *
   * @details This method performs a complete initialization sequence:
   * 1. Sets up the camera manager
   * 2. Selects and acquires a camera
   * 3. Configures the camera stream
   * 4. Allocates frame buffers
   * 5. Creates capture requests
   *
   * Each step must succeed for the initialization to complete successfully.
   */
  int initialize();

  /**
   * @brief Starts the frame capture process
   *
   * @return ErrorCode indicating success or the specific failure point
   *
   * @details This method:
   * 1. Connects the frame completion callback
   * 2. Starts the camera hardware
   * 3. Queues the initial capture requests
   *
   * Once started, frames will be continuously captured and processed until
   * stopCapture() is called.
   */
  int startCapture();

  /**
   * @brief Stops the frame capture process
   *
   * @details Safely stops the camera and clears all pending requests.
   * This method can be called even if capture is not active.
   */
  void stopCapture();

  /**
   * @brief Checks if frame capture is currently active
   *
   * @return true if capture is active, false otherwise
   */
  bool isCapturing() const { return capturing_.load(); }

  /**
   * @brief Retrieves the next available frame from the frame buffer
   *
   * @param frame Reference to a Frame object that will be filled with the next
   * frame data
   * @return true if a frame was retrieved, false if capture was stopped before
   * a frame became available
   *
   * @details This method blocks until either:
   * - A new frame becomes available in the buffer
   * - The capture process is stopped
   *
   * It uses a yield/sleep mechanism to avoid consuming CPU resources while
   * waiting.
   */
  bool getNextFrame(Frame& frame);

 private:
  /** @brief Type alias for camera manager pointer for readability */
  using CameraManagerPtr = std::unique_ptr<libcamera::CameraManager>;

  /** @brief Type alias for camera configuration pointer for readability */
  using CameraConfigurationPtr =
      std::unique_ptr<libcamera::CameraConfiguration>;

  /** @brief Type alias for request pointer for readability */
  using RequestPtr = std::unique_ptr<libcamera::Request>;

  CameraManagerPtr cameraManager_;            /**< Camera manager instance */
  std::shared_ptr<libcamera::Camera> camera_; /**< Selected camera instance */
  CameraConfigurationPtr cameraConfig_;       /**< Camera configuration */
  libcamera::Stream* stream_ = nullptr;       /**< Selected video stream */
  std::unique_ptr<libcamera::FrameBufferAllocator>
      bufferAllocator_; /**< Frame buffer allocator */

  bool cameraAcquired_ = false; /**< Flag indicating if camera is acquired */
  std::atomic<bool> capturing_{
      false}; /**< Flag indicating active capture state */
  unsigned int buffersAllocated_ = 0; /**< Number of allocated frame buffers */
  std::vector<RequestPtr> requests_;  /**< Collection of capture requests */
  std::unordered_map<int, std::pair<void*, size_t>>
      fdMappings_; /**< Memory mappings for frame buffer file descriptors */
  size_t frameSize_ = 0;    /**< Size of a single frame in bytes */
  uint32_t width_;          /**< Frame width in pixels */
  uint32_t height_;         /**< Frame height in pixels */
  uint64_t seqCounter_ = 0; /**< Sequence counter for captured frames */

  /** @brief Reference to the external ring buffer for processed frames */
  RingMaster<Frame>& rgbBuffer_;

  /**
   * @brief Sets up the camera manager to discover available cameras
   *
   * @return ErrorCode indicating success or failure
   *
   * @details Initializes the libcamera framework and discovers connected
   * cameras.
   */
  int setupCameraManager();

  /**
   * @brief Selects the first available camera and acquires it
   *
   * @return ErrorCode indicating success or failure
   *
   * @details Finds and acquires exclusive access to the first available camera.
   */
  int selectCamera();

  /**
   * @brief Configures the camera stream with resolution and format settings
   *
   * @return ErrorCode indicating success or failure
   *
   * @details Creates and applies a stream configuration with specific
   * resolution and pixel format.
   */
  int configureStream();

  /**
   * @brief Allocates memory buffers for frame capture
   *
   * @return ErrorCode indicating success or failure
   *
   * @details Allocates frame buffers and maps them into application memory
   * space.
   */
  int allocateBuffers();

  /**
   * @brief Creates capture requests and attaches buffers
   *
   * @return ErrorCode indicating success or failure
   *
   * @details Creates request objects that will be used during capture.
   */
  int createRequests();

  /**
   * @brief Performs complete cleanup of all allocated resources
   *
   * @details Releases camera hardware, unmaps memory, and frees all resources.
   */
  void cleanup();

  /**
   * @brief Releases the acquired camera back to the system
   *
   * @details Releases exclusive access to the camera hardware.
   */
  void releaseCamera();

  /**
   * @brief Handles completed capture requests
   *
   * @param request Pointer to the completed request
   *
   * @details Processes captured frame data and pushes it to the output buffer.
   * This method is called by libcamera when a frame capture completes.
   */
  void requestComplete(libcamera::Request* request);

  /**
   * @brief Static callback function for libcamera request completion
   *
   * @param request Pointer to the completed request
   * @param user_data Pointer to the FrameForger instance
   *
   * @details Static bridge function that routes callbacks to the appropriate
   * object instance.
   */
  static void requestCallback(libcamera::Request* request, void* user_data);
};
