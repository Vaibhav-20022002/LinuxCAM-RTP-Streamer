#pragma once

#include "frameForger.hh"

#include <sys/mman.h>

#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>

#include "frameTypes.hh"

/**
 * @brief Global flag indicating if the program should continue running
 *
 * @details This flag is modified by the signal handler to gracefully
 * terminate the program when a termination signal is received.
 */
volatile sig_atomic_t running = 1;

/**
 * @brief Signal handler for graceful termination
 *
 * @param sig Signal number (unused but required for signal handler signature)
 *
 * @details Sets the running flag to 0, signaling the main loop to exit cleanly.
 */
void signalHandler(int /*sig*/) { running = 0; }

/**
 * @brief Constructs a FrameForger object with the specified RGB buffer
 *
 * @param rgbBuffer Reference to the ring buffer that will store processed
 * frames
 *
 * @details Initializes the camera manager and stores a reference to the
 * external ring buffer. The camera is not yet initialized or started at this
 * point.
 */
FrameForger::FrameForger(RingMaster<Frame>& rgbBuffer)
    : cameraManager_(std::make_unique<libcamera::CameraManager>()),
      bufferAllocator_(nullptr),
      rgbBuffer_(rgbBuffer) {}

/**
 * @brief Destructor that ensures proper cleanup of all resources
 *
 * @details Stops any active capture and performs comprehensive cleanup
 * of all allocated resources to prevent leaks.
 */
FrameForger::~FrameForger() {
  stopCapture();
  cleanup();
}

/**
 * @brief Retrieves the next available frame from the frame buffer
 *
 * @param frame Reference to a Frame object that will be filled with the next
 * frame data
 * @return true if a frame was retrieved, false if capture was stopped before a
 * frame became available
 *
 * @details This method blocks until either:
 * - A new frame becomes available in the buffer
 * - The capture process is stopped
 *
 * It uses a yield/sleep mechanism to avoid consuming CPU resources while
 * waiting.
 */
bool FrameForger::getNextFrame(Frame& frame) {
  // Block until a frame is available or capture stops
  while (capturing_.load()) {
    if (rgbBuffer_.pop(frame)) return true;
    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  return false;
}

/**
 * @brief Processes a completed frame capture request
 *
 * @param request Pointer to the completed request
 *
 * @details This method is called by libcamera when a frame capture is complete.
 * It processes the captured frame data by:
 * 1. Checking if the request was cancelled
 * 2. Locating the frame buffer for the configured stream
 * 3. Copying frame data from mapped memory to a Frame object
 * 4. Adding metadata like timestamp and sequence number
 * 5. Pushing the frame to the output ring buffer
 * 6. Recycling the request for continuous capture
 *
 * The method handles memory mapping and proper buffer management to maintain
 * continuous frame capture.
 */
void FrameForger::requestComplete(libcamera::Request* request) {
  if (request->status() == libcamera::Request::RequestCancelled) return;

  // Locate the FrameBuffer for our stream:
  auto const& bufs = request->buffers();
  auto it = bufs.find(stream_);
  if (it == bufs.end()) return;
  libcamera::FrameBuffer* fb = it->second;

  // Build our shared Frame
  Frame frame;
  frame.rawData.resize(frameSize_);
  size_t copied = 0;
  for (auto& plane : fb->planes()) {
    int fd = plane.fd.get();
    auto mapIt = fdMappings_.find(fd);
    if (mapIt == fdMappings_.end()) continue;
    uint8_t* src = static_cast<uint8_t*>(mapIt->second.first) + plane.offset;
    size_t toCopy = std::min<size_t>(plane.length, frameSize_ - copied);
    std::memcpy(frame.rawData.data() + copied, src, toCopy);
    copied += toCopy;
  }

  // Fill metadata
  auto ts = request->metadata().get(libcamera::controls::SensorTimestamp);
  if (ts) frame.timestamp = std::chrono::microseconds(*ts);
  frame.width = width_;
  frame.height = height_;
  // Sequence ID
  frame.sequenceNumber = seqCounter_++;

  // Push into the external ring, back off if full
  while (!rgbBuffer_.push(std::move(frame))) {
    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::microseconds(20));
  }

  // Reuse & requeue
  request->reuse();
  if (request->addBuffer(stream_, fb) < 0) return;
  camera_->queueRequest(request);
}

/**
 * @brief Static callback function for libcamera request completion
 *
 * @param request Pointer to the completed request
 * @param user_data Pointer to the FrameForger instance
 *
 * @details Static bridge function that routes callbacks to the appropriate
 * object instance. This function is necessary because libcamera requires a
 * static callback function.
 */
void FrameForger::requestCallback(libcamera::Request* request,
                                  void* user_data) {
  static_cast<FrameForger*>(user_data)->requestComplete(request);
}

/**
 * @brief Initializes the entire camera capture pipeline.
 *
 * @details Performs a sequential initialization process:
 * 1. Sets up the camera manager to discover available cameras
 * 2. Selects and acquires a camera device
 * 3. Configures the stream with appropriate resolution and pixel format
 * 4. Allocates memory for frame buffers
 * 5. Creates capture requests and attaches buffers
 *
 * If any step fails, the initialization is aborted and an error code is
 * returned.
 *
 * @return ErrorCode indicating success or the specific point of failure
 *
 * @see setupCameraManager()
 * @see selectCamera()
 * @see configureStream()
 * @see allocateBuffers()
 * @see createRequests()
 */
int FrameForger::initialize() {
  int ret = setupCameraManager();
  if (ret != ErrorCode::SUCCESS) return ret;

  ret = selectCamera();
  if (ret != ErrorCode::SUCCESS) return ret;

  ret = configureStream();
  if (ret != ErrorCode::SUCCESS) return ret;

  ret = allocateBuffers();
  if (ret != ErrorCode::SUCCESS) return ret;

  return createRequests();
}

/**
 * @brief Initializes the camera manager to discover available cameras.
 *
 * @details Starts the libcamera camera manager, which discovers camera devices
 * connected to the system and makes them available for use. This is the first
 * step in the camera initialization sequence.
 *
 * @return ErrorCode::SUCCESS if the camera manager starts successfully,
 *         ERR_CAMERA_MANAGER_START if it fails to start
 *
 * @see libcamera::CameraManager::start()
 */
int FrameForger::setupCameraManager() {
  if (cameraManager_->start()) {
    std::cerr << "Failed to start camera manager" << std::endl;
    return ErrorCode::ERR_CAMERA_MANAGER_START;
  }
  return ErrorCode::SUCCESS;
}

/**
 * @brief Selects the first available camera and acquires it for exclusive use.
 *
 * @details This method:
 * 1. Retrieves the list of cameras from the camera manager
 * 2. Verifies that at least one camera is available
 * 3. Selects the first camera in the list
 * 4. Acquires exclusive access to the camera
 *
 * @return ErrorCode::SUCCESS if a camera was successfully acquired,
 *         ERR_NO_CAMERAS if no cameras were found,
 *         ERR_CAMERA_ACQUIRE if the camera could not be acquired (possibly in
 * use)
 *
 * @see libcamera::CameraManager::cameras()
 * @see libcamera::Camera::acquire()
 */
int FrameForger::selectCamera() {
  auto cameras = cameraManager_->cameras();
  if (cameras.empty()) {
    std::cerr << "No cameras available" << std::endl;
    return ErrorCode::ERR_NO_CAMERAS;
  }

  camera_ = cameras.front();
  if (camera_->acquire()) {
    std::cerr << "Failed to acquire camera" << std::endl;
    return ErrorCode::ERR_CAMERA_ACQUIRE;
  }
  cameraAcquired_ = true;
  return ErrorCode::SUCCESS;
}

/**
 * @brief Configures the camera stream with specific resolution and format.
 *
 * @details This method performs stream configuration:
 * 1. Generates a camera configuration for video recording
 * 2. Sets the stream resolution to 640x480
 * 3. Sets the pixel format to RGB888
 * 4. Validates the configuration against hardware capabilities
 * 5. Applies the validated configuration to the camera hardware
 * 6. Stores a reference to the configured stream
 * 7. Stores the frame size for later buffer allocation
 *
 * @return ErrorCode::SUCCESS if configuration succeeds,
 *         ERR_CONFIG_GENERATION if configuration generation fails,
 *         ERR_CONFIG_VALIDATION if validation fails,
 *         ERR_CONFIG_APPLICATION if applying the configuration fails
 *
 * @see libcamera::Camera::generateConfiguration()
 * @see libcamera::CameraConfiguration::validate()
 * @see libcamera::Camera::configure()
 */
int FrameForger::configureStream() {
  cameraConfig_ =
      camera_->generateConfiguration({libcamera::StreamRole::VideoRecording});
  if (!cameraConfig_ || cameraConfig_->empty()) {
    std::cerr << "Failed to generate camera configuration" << std::endl;
    return ErrorCode::ERR_CONFIG_GENERATION;
  }

  auto& streamConfig = cameraConfig_->at(0);
  streamConfig.size = {640, 480};
  width_ = streamConfig.size.width;
  height_ = streamConfig.size.height;
  streamConfig.pixelFormat = libcamera::formats::BGR888;

  // Validate and apply configuration
  if (cameraConfig_->validate() == libcamera::CameraConfiguration::Invalid) {
    std::cerr << "Failed to validate configuration" << std::endl;
    return ErrorCode::ERR_CONFIG_VALIDATION;
  }

  if (camera_->configure(cameraConfig_.get())) {
    std::cerr << "Failed to apply camera configuration" << std::endl;
    return ErrorCode::ERR_CONFIG_APPLICATION;
  }

  // Store direct stream reference and frame size
  stream_ = streamConfig.stream();
  frameSize_ = streamConfig.frameSize;

  return ErrorCode::SUCCESS;
}

/**
 * @brief Allocates frame buffers for the configured stream and maps them to
 * memory.
 *
 * @details This method:
 * 1. Creates a FrameBufferAllocator for the camera
 * 2. Allocates buffers for the configured stream
 * 3. Collects file descriptors and memory requirements for each buffer
 * 4. Maps buffer memory into process address space using mmap
 * 5. Stores mappings for later access
 * 6. Tracks the number of buffers allocated
 *
 * @return ErrorCode::SUCCESS if allocation succeeds,
 *         ERR_BUFFER_ALLOCATION if buffer allocation or memory mapping fails
 *
 * @see libcamera::FrameBufferAllocator::allocate()
 * @see mmap()
 */
int FrameForger::allocateBuffers() {
  bufferAllocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
  if (bufferAllocator_->allocate(stream_) < 0) {
    std::cerr << "Failed to allocate frame buffers" << std::endl;
    return ErrorCode::ERR_BUFFER_ALLOCATION;
  }

  // Collect all FDs and max lengths
  std::unordered_map<int, size_t> maxLens;
  for (const auto& buffer : bufferAllocator_->buffers(stream_)) {
    for (const auto& plane : buffer->planes()) {
      int fd = plane.fd.get();
      size_t reqLen = plane.offset + plane.length;
      if (reqLen > maxLens[fd]) {
        maxLens[fd] = reqLen;
      }
    }
  }

  // Map memory for each FD
  for (const auto& [fd, len] : maxLens) {
    void* addr = mmap(nullptr, len, PROT_READ, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
      std::cerr << "MMAP Failed" << std::endl;
      // Cleanup exisiting mappings
      for (const auto& [mapped_fd, mapping] : fdMappings_) {
        munmap(mapping.first, mapping.second);
      }
      fdMappings_.clear();
      return ErrorCode::ERR_BUFFER_ALLOCATION;
    }
    fdMappings_[fd] = {addr, len};
  }

  buffersAllocated_ = bufferAllocator_->buffers(stream_).size();
  return ErrorCode::SUCCESS;
}

/**
 * @brief Creates capture requests and attaches buffers for frame capture.
 *
 * @details For each allocated buffer, this method:
 * 1. Creates a capture request from the camera
 * 2. Adds the buffer to the request
 * 3. Stores the request for later use
 *
 * These requests will be queued during startCapture() to begin
 * the frame capture process.
 *
 * @return ErrorCode::SUCCESS if all requests are created successfully,
 *         ERR_REQUEST_CREATION if request creation fails,
 *         ERR_REQUEST_BUFFER if adding a buffer to a request fails
 *
 * @see libcamera::Camera::createRequest()
 * @see libcamera::Request::addBuffer()
 */
int FrameForger::createRequests() {
  for (auto& buffer :
       bufferAllocator_->buffers(stream_)) {  // Use stored pointer
    auto request = camera_->createRequest();
    if (!request) {
      std::cerr << "Failed to create request" << std::endl;
      return ErrorCode::ERR_REQUEST_CREATION;
    }

    if (request->addBuffer(stream_, buffer.get()) < 0) {  // Use active stream
      std::cerr << "Failed to add buffer to request" << std::endl;
      return ErrorCode::ERR_REQUEST_BUFFER;
    }

    requests_.push_back(std::move(request));
  }
  return ErrorCode::SUCCESS;
}

/**
 * @brief Starts the continuous frame capture process
 *
 * @return ErrorCode indicating success or failure
 *
 * @details This method:
 * 1. Checks if capture is already active
 * 2. Connects the frame completion callback
 * 3. Starts the camera hardware
 * 4. Queues all capture requests to begin continuous capture
 * 5. Sets the capturing flag to true
 *
 * Once started, frames will be continuously captured and processed until
 * stopCapture() is called.
 *
 * @see libcamera::Camera::start()
 * @see libcamera::Camera::queueRequest()
 */
int FrameForger::startCapture() {
  if (capturing_.load()) return ErrorCode::SUCCESS;

  // Connect the libcamera signal to our member callback
  camera_->requestCompleted.connect(this, &FrameForger::requestComplete);

  if (camera_->start()) return ErrorCode::ERR_CAMERA_START;

  for (auto& req : requests_) {
    if (camera_->queueRequest(req.get()) < 0)
      return ErrorCode::ERR_QUEUE_REQUEST;
  }

  capturing_.store(true);
  return ErrorCode::SUCCESS;
}

/**
 * @brief Stops the frame capture process
 *
 * @details This method:
 * 1. Atomically sets the capturing flag to false
 * 2. Stops the camera hardware
 * 3. Clears all capture requests
 *
 * The method is safe to call even if capture is not active.
 *
 * @see libcamera::Camera::stop()
 */
void FrameForger::stopCapture() {
  if (!capturing_.exchange(false)) return;
  camera_->stop();
  requests_.clear();
}

/**
 * @brief Performs cleanup of all allocated resources.
 *
 * @details This method releases all resources in a specific order:
 * 1. Unmaps all memory-mapped buffer regions
 * 2. Clears the frame buffer queue
 * 3. Releases the camera if acquired
 * 4. Stops the camera manager
 *
 * This comprehensive cleanup ensures that all resources are properly
 * released, preventing memory leaks and allowing hardware to be
 * used by other applications.
 *
 * @see releaseCamera()
 * @see munmap()
 */
void FrameForger::cleanup() {
  // Unmap all MMAPed buffers
  for (auto& [fd, mapping] : fdMappings_) {
    munmap(mapping.first, mapping.second);
  }
  fdMappings_.clear();

  // Release camera and stop manager
  releaseCamera();
  if (cameraManager_) cameraManager_->stop();
}

/**
 * @brief Releases the acquired camera resource.
 *
 * @details This method:
 * 1. Checks if a camera is currently acquired
 * 2. If so, releases the camera back to the system
 * 3. Updates the cameraAcquired_ flag
 *
 * This allows other applications to use the camera after
 * this application is done with it.
 *
 * @see libcamera::Camera::release()
 */
void FrameForger::releaseCamera() {
  if (cameraAcquired_) {
    camera_->release();
    cameraAcquired_ = false;
  }
}
