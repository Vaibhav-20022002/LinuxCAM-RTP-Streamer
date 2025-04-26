// frameForger.hh
#pragma once

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/stream.h>
#include <sys/mman.h>  // mmap, PROT_*, MAP_*
#include <unistd.h>    // close, lseek

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <csignal>  // For signal handling
#include <cstring>  // Added for std::memcpy
#include <iomanip>  // For std::setw, std::setfill formatting
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>  // For sleep
#include <unordered_map>
#include <utility>
#include <vector>

/**
 * @class FrameForger
 * @brief Manages camera initialization, buffer allocation, and frame capture
 * using libcamera.
 *
 * @details This class provides a comprehensive interface for camera operations
 * using the libcamera framework. It encapsulates the complexity of camera
 * initialization, stream configuration, buffer management, and frame capture
 * while providing a simple API for client applications.
 *
 * Key responsibilities include:
 * - Camera device discovery and selection
 * - Stream and format configuration
 * - Frame buffer allocation and memory mapping
 * - Request queue management
 * - Frame capture and metadata extraction
 *
 * The class uses a producer-consumer pattern where libcamera produces frames
 * in callback threads, and the application consumes them via getNextFrame().
 *
 * @note This class is not thread-safe for initialization and capture start/stop
 * operations, but thread-safe for frame retrieval.
 *
 * @see libcamera::Camera
 * @see libcamera::CameraManager
 * @see libcamera::Stream
 * @see libcamera::Request
 *
 * @version 1.0
 * @author Original Author
 */
class FrameForger {
 public:
  /**
   * @brief Error codes returned by FrameForger methods.
   *
   * @details These error codes provide specific information about where
   * failures occurred during operation. Client code should check return values
   * against these codes to determine appropriate actions.
   */
  enum ErrorCode {
    SUCCESS = 0,                   /**< Operation completed successfully */
    ERR_CAMERA_MANAGER_START = -1, /**< Failed to start camera manager */
    ERR_NO_CAMERAS = -2,           /**< No cameras found in the system */
    ERR_CAMERA_ACQUIRE = -3, /**< Failed to acquire camera (possibly in use) */
    ERR_CONFIG_GENERATION = -4, /**< Failed to generate camera configuration */
    ERR_CONFIG_VALIDATION = -5, /**< Camera configuration validation failed */
    ERR_CONFIG_APPLICATION =
        -6, /**< Failed to apply camera configuration to hardware */
    ERR_BUFFER_ALLOCATION = -7, /**< Failed to allocate frame buffers */
    ERR_REQUEST_CREATION = -8,  /**< Failed to create capture request */
    ERR_REQUEST_BUFFER = -9,    /**< Failed to add buffer to request */
    ERR_CAMERA_START = -10,     /**< Failed to start camera streaming */
    ERR_QUEUE_REQUEST = -11     /**< Failed to queue capture request */
  };

  /**
   * @brief Constructs a FrameForger object and initializes the CameraManager.
   *
   * @details Creates a new FrameForger instance and initializes the underlying
   * libcamera::CameraManager, which is responsible for camera device discovery.
   * The constructor does not start the camera manager or acquire any cameras.
   *
   * @note After construction, call initialize() to complete setup.
   */
  explicit FrameForger();

  /**
   * @brief Destroys the FrameForger, stopping capture and cleaning up
   * resources.
   *
   * @details Performs a clean shutdown sequence:
   * 1. Stops any active capture (if running)
   * 2. Releases memory mappings
   * 3. Deallocates frame buffers
   * 4. Releases camera resources
   * 5. Stops the camera manager
   *
   * This ensures that all hardware resources are properly released, preventing
   * resource leaks and allowing other applications to access the camera.
   */
  ~FrameForger();

  /**
   * @brief Initializes camera manager, selects camera, configures stream,
   * allocates buffers, and creates requests.
   *
   * @details This method performs the complete camera initialization sequence:
   * 1. Starts the camera manager and discovers available cameras
   * 2. Selects and acquires the first available camera
   * 3. Configures the camera stream with appropriate resolution and format
   * 4. Allocates memory for frame buffers
   * 5. Creates capture requests and attaches buffers
   *
   * This method must be called before startCapture().
   *
   * @return ErrorCode indicating success or specific failure point.
   *
   * @see ErrorCode
   */
  int initialize();

  /**
   * @brief Starts capturing frames by queueing requests and connecting
   * callbacks.
   *
   * @details This method begins the frame capture process:
   * 1. Registers the request completion callback
   * 2. Starts the camera hardware
   * 3. Queues initial capture requests
   *
   * Once started, the camera will continuously capture frames until
   * stopCapture() is called or the object is destroyed.
   *
   * @pre initialize() must be called successfully before this method.
   * @post Frames will be available via getNextFrame() if successful.
   *
   * @return ErrorCode indicating success or specific failure point.
   *
   * @see ErrorCode
   * @see getNextFrame()
   * @see stopCapture()
   */
  int startCapture();

  /**
   * @brief Stops frame capture and clears queued requests.
   *
   * @details This method halts the capture process:
   * 1. Stops the camera hardware
   * 2. Clears any pending requests
   * 3. Disconnects callback handlers
   *
   * After this call, no new frames will be produced, but existing frames in the
   * queue will remain available for consumption via getNextFrame().
   *
   * @post capturing_ flag is set to false
   * @post completedRequests_ queue is cleared
   *
   * @see startCapture()
   * @see getNextFrame()
   */
  void stopCapture();

  /**
   * @brief Queries whether capture is currently active.
   *
   * @details This method provides a thread-safe way to check if the camera
   * is currently capturing frames. It can be used to determine if
   * startCapture() has been called and stopCapture() has not yet been called.
   *
   * @return true if capturing is active; false otherwise.
   *
   * @see startCapture()
   * @see stopCapture()
   */
  bool isCapturing() const { return capturing_; }

  /**
   * @brief Container for frame metadata information.
   *
   * @details This structure holds timing and other metadata associated with
   * a captured frame, separate from the actual pixel data. This separation
   * allows for efficient processing and metadata-only operations.
   */
  struct FrameMetaData {
    /**
     * @brief Timestamp when the frame was captured by the sensor.
     *
     * @details This timestamp represents the moment when the sensor finished
     * capturing the frame, measured in microseconds since an arbitrary
     * reference point. The timestamp is useful for synchronization and
     * framerate calculations.
     */
    std::chrono::microseconds timestamp;

    /**
     * @todo Add more fields according to the TurboJPEG-lib requirements
     * This would include fields such as exposure time, gain, color temperature,
     * etc.
     */
  };

  /**
   * @brief Container for complete frame data and associated metadata.
   *
   * @details This structure combines the raw pixel data with the metadata
   * for a single frame. It serves as the primary interface for applications
   * to receive complete frame information. The data is stored as a vector
   * of bytes in the format specified during stream configuration.
   */
  struct Frame {
    /**
     * @brief Raw pixel data for the frame.
     *
     * @details Contains the actual frame content as a sequence of bytes.
     * The format is determined by the stream configuration (e.g., RGB888),
     * and the size matches the configured resolution.
     */
    std::vector<uint8_t> data;

    /**
     * @brief Associated metadata for this frame.
     *
     * @details Contains timing and other information about the frame.
     * @see FrameMetaData
     */
    FrameMetaData metadata;
  };

  /**
   * @brief Retrieves the next available frame from the internal queue.
   *
   * @details This method provides the primary mechanism for consuming captured
   * frames. It attempts to retrieve the oldest frame from the internal queue.
   * If a frame is available, it moves the frame data and metadata into the
   * provided Frame structure and returns true. If no frame is available,
   * it returns false immediately without waiting.
   *
   * This method is thread-safe and can be called concurrently from multiple
   * threads.
   *
   * @param[out] frame Reference to a Frame structure to receive the data
   *
   * @return true if a frame was retrieved successfully; false if no frame was
   * available
   *
   * @note This method does not block. For blocking behavior, implement a wait
   * loop.
   *
   * @see Frame
   * @see FrameMetaData
   */
  bool getNextFrame(Frame& frame);

 private:
  /**
   * @brief Type alias for camera manager smart pointer.
   * @details Provides memory management for the CameraManager instance.
   */
  using CameraManagerPtr = std::unique_ptr<libcamera::CameraManager>;

  /**
   * @brief Type alias for camera configuration smart pointer.
   * @details Provides memory management for the CameraConfiguration instance.
   */
  using CameraConfigurationPtr =
      std::unique_ptr<libcamera::CameraConfiguration>;

  /**
   * @brief Type alias for request smart pointer.
   * @details Provides memory management for Request instances.
   */
  using RequestPtr = std::unique_ptr<libcamera::Request>;

  // Core components
  /**
   * @brief Manager for camera discovery and access.
   * @details The central entry point for libcamera operations, responsible
   * for enumerating and providing access to available camera devices.
   */
  CameraManagerPtr cameraManager_;

  /**
   * @brief Reference to the selected camera device.
   * @details The camera instance being used for capture operations.
   * Shared ownership is used as libcamera maintains its own references.
   */
  std::shared_ptr<libcamera::Camera> camera_;

  /**
   * @brief Configuration for the camera capture parameters.
   * @details Contains the stream configurations including resolution,
   * pixel format, and other capture parameters.
   */
  CameraConfigurationPtr cameraConfig_;

  /**
   * @brief Pointer to the active video stream.
   * @details Non-owning reference to the configured stream instance.
   * Stream lifetime is managed by the CameraConfiguration.
   */
  libcamera::Stream* stream_ = nullptr;

  /**
   * @brief Allocator for frame buffers.
   * @details Manages the allocation and deallocation of memory for
   * storing captured frames.
   */
  std::unique_ptr<libcamera::FrameBufferAllocator> bufferAllocator_;

  // State management flags
  /**
   * @brief Flag indicating if a camera has been successfully acquired.
   * @details Used during cleanup to determine if camera release is needed.
   */
  bool cameraAcquired_ = false;

  /**
   * @brief Flag indicating if capture is currently active.
   * @details Used to prevent duplicate start/stop operations.
   */
  bool capturing_ = false;

  /**
   * @brief Count of allocated frame buffers.
   * @details Tracks the number of buffers allocated for the stream.
   */
  unsigned int buffersAllocated_ = 0;

  // Request and memory management
  /**
   * @brief Collection of capture request objects.
   * @details Owns the Request objects that are reused during capture.
   */
  std::vector<RequestPtr> requests_;

  /**
   * @brief Queue of completed capture requests pending processing.
   * @details Non-owning queue of requests that have completed and need
   * to be processed. Access must be protected by requestMutex_.
   * @warning This queue is not thread-safe on its own.
   */
  std::queue<libcamera::Request*> completedRequests_;

  /**
   * @brief Mutex for protecting access to the completedRequests_ queue.
   * @details Ensures thread-safe operations on the shared request queue.
   */
  mutable std::mutex requestMutex_;

  /**
   * @brief Condition variable for signaling request completion.
   * @details Used to notify waiting threads about changes in the
   * completedRequests_ queue.
   */
  std::condition_variable requestCv_;

  /**
   * @brief Queue of processed frames ready for consumption.
   * @details Thread-safe queue of frames available for getNextFrame().
   */
  std::queue<Frame> frameBuffer_;

  /**
   * @brief Mutex for protecting access to the frameBuffer_ queue.
   * @details Ensures thread-safe operations on the shared frame queue.
   */
  mutable std::mutex frameBufferMutex_;

  /**
   * @brief Condition variable for signaling frame availability.
   * @details Used to notify waiting threads about changes in the frameBuffer_
   * queue.
   */
  std::condition_variable frameBufferCv_;

  /**
   * @brief Mapping of file descriptors to memory mappings.
   * @details Maps file descriptors to memory address and size for
   * accessing frame buffer memory.
   */
  std::unordered_map<int, std::pair<void*, size_t>> fdMappings_;

  /**
   * @brief Size of frames in bytes.
   * @details Stored size of frames based on resolution and pixel format.
   */
  size_t frameSize_;

  /**
   * @brief Starts the CameraManager.
   *
   * @details This initializes the underlying libcamera infrastructure,
   * discovering available cameras and preparing them for use. It's the
   * first step in the initialization sequence.
   *
   * @return ErrorCode::SUCCESS or ERR_CAMERA_MANAGER_START
   *
   * @see libcamera::CameraManager::start()
   */
  int setupCameraManager();

  /**
   * @brief Selects the first available camera and acquires it.
   *
   * @details This method:
   * 1. Retrieves the list of available cameras from the camera manager
   * 2. Selects the first camera in the list
   * 3. Acquires exclusive access to the camera
   *
   * If no cameras are available or acquisition fails (e.g., camera in use by
   * another process), an appropriate error code is returned.
   *
   * @pre setupCameraManager() must be called successfully before this method.
   * @post cameraAcquired_ is set to true if successful
   *
   * @return ErrorCode::SUCCESS, ERR_NO_CAMERAS, or ERR_CAMERA_ACQUIRE
   *
   * @see libcamera::Camera::acquire()
   */
  int selectCamera();

  /**
   * @brief Generates and validates the stream configuration, then applies it.
   *
   * @details This method configures the video stream:
   * 1. Generates a configuration for video recording
   * 2. Sets the desired resolution and pixel format
   * 3. Validates the configuration against hardware capabilities
   * 4. Applies the validated configuration to the camera
   *
   * @pre selectCamera() must be called successfully before this method.
   * @post stream_ is initialized with the configured stream
   * @post frameSize_ is set to the calculated frame size
   *
   * @return ErrorCode::SUCCESS, ERR_CONFIG_GENERATION, ERR_CONFIG_VALIDATION,
   * or ERR_CONFIG_APPLICATION
   *
   * @see libcamera::Camera::generateConfiguration()
   * @see libcamera::CameraConfiguration::validate()
   * @see libcamera::Camera::configure()
   */
  int configureStream();

  /**
   * @brief Allocates frame buffers for the configured stream.
   *
   * @details This method:
   * 1. Creates a FrameBufferAllocator for the selected camera
   * 2. Allocates buffers for the configured stream
   * 3. Maps buffer memory for direct access
   * 4. Stores mappings for later use
   *
   * @pre configureStream() must be called successfully before this method.
   * @post buffersAllocated_ is set to the number of allocated buffers
   * @post fdMappings_ contains mappings for all allocated buffer memory
   *
   * @return ErrorCode::SUCCESS or ERR_BUFFER_ALLOCATION
   *
   * @see libcamera::FrameBufferAllocator::allocate()
   * @see mmap()
   */
  int allocateBuffers();

  /**
   * @brief Creates capture requests and attaches buffers.
   *
   * @details This method:
   * 1. Creates a Request object for each allocated buffer
   * 2. Attaches a buffer to each request
   * 3. Stores the requests for later use
   *
   * These requests will be queued during startCapture() to initiate the
   * frame capture process.
   *
   * @pre allocateBuffers() must be called successfully before this method.
   * @post requests_ contains a Request for each allocated buffer
   *
   * @return ErrorCode::SUCCESS, ERR_REQUEST_CREATION, or ERR_REQUEST_BUFFER
   *
   * @see libcamera::Camera::createRequest()
   * @see libcamera::Request::addBuffer()
   */
  int createRequests();

  /**
   * @brief Handles completed requests by processing frame data and metadata.
   *
   * @details This callback function is invoked by libcamera when a capture
   * request completes. It:
   * 1. Extracts frame data from the mapped memory
   * 2. Retrieves metadata such as timestamp
   * 3. Creates a Frame object with the data and metadata
   * 4. Adds the Frame to the frameBuffer_ queue
   * 5. Reuses and requeues the request for continuous capture
   *
   * This function runs in libcamera's internal thread context and
   * should perform minimal processing to avoid blocking.
   *
   * @param request Pointer to the completed Request object.
   *
   * @pre The request must contain a valid FrameBuffer for stream_.
   * @post A new Frame is added to frameBuffer_ if successful.
   * @post The request is requeued for continuous capture.
   *
   * @see libcamera::Request
   * @see Frame
   * @see libcamera::Camera::queueRequest()
   */
  void requestComplete(libcamera::Request* request);

  /**
   * @brief Static adapter for request completion callback.
   *
   * @details This static function serves as an adapter between libcamera's
   * C-style callback mechanism and the object-oriented design of FrameForger.
   * It casts the user_data pointer to a FrameForger instance and delegates
   * to that instance's requestComplete method.
   *
   * This function is registered as the callback for the requestCompleted
   * signal.
   *
   * @param request Pointer to the completed Request object.
   * @param user_data Pointer to the FrameForger instance.
   *
   * @see requestComplete()
   * @see libcamera::Signal
   */
  static void requestCallback(libcamera::Request* request, void* user_data);

  /**
   * @brief Releases all resources and cleans up.
   *
   * @details This method performs a complete cleanup sequence:
   * 1. Unmaps all memory mappings
   * 2. Clears the frame buffer queue
   * 3. Releases the camera if acquired
   * 4. Stops the camera manager
   *
   * This method is called by the destructor and can also be called
   * explicitly for early cleanup.
   *
   * @see releaseCamera()
   * @see munmap()
   */
  void cleanup();

  /**
   * @brief Releases the acquired camera if owned.
   *
   * @details This method releases exclusive access to the camera
   * if it was previously acquired. It checks the cameraAcquired_
   * flag to avoid attempting to release an unacquired camera.
   *
   * @post cameraAcquired_ is set to false if successful
   *
   * @see libcamera::Camera::release()
   */
  void releaseCamera();
};
