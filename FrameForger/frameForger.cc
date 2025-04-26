// frameForger.cc
#include "frameForger.hh"

/**
 * Global flag for graceful termination of the capture process.
 * Set to 0 by signal handlers to indicate that the application should stop.
 * @see signalHandler
 */
volatile sig_atomic_t running = 1;

/**
 * @brief Signal handler for graceful termination of the application.
 *
 * @details This function is registered as a handler for SIGINT and SIGTERM
 * signals. When these signals are received, it sets the global 'running' flag
 * to 0, allowing the main loop to exit cleanly.
 *
 * @param signal The signal number that was received (e.g., SIGINT or SIGTERM).
 *
 * @see running
 */
void signalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    running = 0;
  }
}

/**
 * @brief Constructs a new FrameForger object with initialized camera manager.
 *
 * @details Creates a camera manager instance using smart pointers to ensure
 * proper resource cleanup. The buffer allocator is initialized to nullptr and
 * will be created during the initialization process.
 *
 * @note This constructor does not start the camera manager or acquire any
 * cameras. The initialize() method must be called to complete the setup
 * process.
 *
 * @see initialize()
 */
FrameForger::FrameForger()
    : cameraManager_(std::make_unique<libcamera::CameraManager>()),
      bufferAllocator_(nullptr) {}

/**
 * @brief Destroys the FrameForger instance, releasing all associated resources.
 *
 * @details Performs a clean shutdown sequence by stopping any active capture
 * session and cleaning up all allocated resources, including:
 * - Memory mappings for frame buffers
 * - Camera resources
 * - Buffer allocator
 * - Camera manager
 *
 * This ensures that all hardware resources are properly released, preventing
 * resource leaks and allowing other applications to access the camera.
 *
 * @see stopCapture()
 * @see cleanup()
 */
FrameForger::~FrameForger() {
  stopCapture();
  cleanup();
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
  streamConfig.pixelFormat = libcamera::formats::RGB888;

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
 * @brief Starts the frame capture process.
 *
 * @details This method begins the continuous frame capture process:
 * 1. Verifies that capture is not already active
 * 2. Connects the requestCompleted signal to the request handler
 * 3. Starts the camera hardware
 * 4. Queues all prepared requests to begin frame capture
 *
 * Once started, the camera will continuously capture frames until
 * stopCapture() is called or the object is destroyed.
 *
 * @return ErrorCode::SUCCESS if capture starts successfully,
 *         ERR_CAMERA_START if starting the camera fails,
 *         ERR_QUEUE_REQUEST if queueing requests fails
 *
 * @see requestComplete()
 * @see libcamera::Camera::start()
 * @see libcamera::Camera::queueRequest()
 */
int FrameForger::startCapture() {
  if (capturing_) return ErrorCode::SUCCESS;

  camera_->requestCompleted.connect(
      this, [this](libcamera::Request* req) { this->requestComplete(req); });

  if (camera_->start()) {
    std::cerr << "Failed to start camera" << std::endl;
    return ErrorCode::ERR_CAMERA_START;
  }

  for (auto& request : requests_) {
    if (camera_->queueRequest(request.get()) < 0) {
      std::cerr << "Failed to queue request" << std::endl;
      return ErrorCode::ERR_QUEUE_REQUEST;
    }
  }

  capturing_ = true;
  return ErrorCode::SUCCESS;
}

/**
 * @brief Stops the frame capture process.
 *
 * @details This method halts the continuous frame capture:
 * 1. Verifies that capture is currently active
 * 2. Stops the camera hardware
 * 3. Sets the capturing flag to false
 * 4. Clears the completed requests queue
 * 5. Clears all pending requests
 *
 * After stopping, any frames already in the frame buffer remain
 * available for retrieval.
 *
 * @see startCapture()
 * @see libcamera::Camera::stop()
 */
void FrameForger::stopCapture() {
  if (!capturing_) return;

  camera_->stop();
  capturing_ = false;

  {
    std::lock_guard<std::mutex> lock(requestMutex_);
    while (!completedRequests_.empty()) {
      completedRequests_.pop();
    }
  }

  requests_.clear();
}

/**
 * @brief Retrieves the next available frame from the internal queue.
 *
 * @details This method provides the primary mechanism for consuming captured
 * frames:
 * 1. Acquires a lock on the frame buffer mutex
 * 2. Checks if any frames are available
 * 3. If available, moves the oldest frame to the output parameter
 * 4. Removes the frame from the queue
 *
 * This method is thread-safe and can be called from multiple threads.
 *
 * @param[out] frame Reference to a Frame structure to receive the frame data
 *
 * @return true if a frame was successfully retrieved, false if no frames were
 * available
 *
 * @note This method does not block when no frames are available.
 *
 * @see Frame
 */
bool FrameForger::getNextFrame(Frame& frame) {
  std::lock_guard<std::mutex> lock(frameBufferMutex_);
  if (frameBuffer_.empty()) return false;

  frame = std::move(frameBuffer_.front());
  frameBuffer_.pop();
  return true;
}

/**
 * @brief Processes a completed capture request and prepares the next one.
 *
 * @details This callback function is invoked when a capture request completes:
 * 1. Checks if the request was cancelled
 * 2. Retrieves the frame buffer from the request
 * 3. Copies frame data from mapped memory to the frame buffer
 * 4. Extracts metadata such as timestamp
 * 5. Places the complete frame in the frame buffer queue
 * 6. Reuses the request for continuous capture
 * 7. Requeues the request with the camera
 *
 * This implements a continuous capture loop where each completed request
 * is immediately reused and requeued.
 *
 * @param request Pointer to the completed capture request
 *
 * @see libcamera::Request
 * @see Frame
 * @see libcamera::Camera::queueRequest()
 */
void FrameForger::requestComplete(libcamera::Request* request) {
  if (request->status() == libcamera::Request::RequestCancelled) return;

  auto const& buffers = request->buffers();
  auto itr = buffers.find(stream_);
  if (itr == buffers.end()) {
    std::cerr << "No buffer for stream!" << std::endl;
    return;
  }
  libcamera::FrameBuffer* fb = itr->second;

  // Prepare frame data
  Frame frame;
  frame.data.resize(frameSize_);
  size_t copied = 0;

  // Copy data from mapped memory
  for (const auto& plane : fb->planes()) {
    int fd = plane.fd.get();
    auto mapping = fdMappings_.find(fd);
    if (mapping == fdMappings_.end()) {
      std::cerr << "No mapping for FD " << fd << std::endl;
      continue;
    }

    uint8_t* src = static_cast<uint8_t*>(mapping->second.first) + plane.offset;
    size_t toCopy =
        std::min(static_cast<size_t>(plane.length), frameSize_ - copied);
    std::memcpy(frame.data.data() + copied, src, toCopy);
    copied += toCopy;
  }

  // Get the timestamp from metadata
  auto ts = request->metadata().get(libcamera::controls::SensorTimestamp);
  if (ts) frame.metadata.timestamp = std::chrono::microseconds(*ts);

  // Store in queue
  {
    std::lock_guard<std::mutex> lock(frameBufferMutex_);
    frameBuffer_.push(std::move(frame));
  }
  frameBufferCv_.notify_one();

  // Reuse request and re-add buffer
  request->reuse();
  if (request->addBuffer(stream_, fb) < 0) {
    std::cerr << "Failed to add buffer to reused request" << std::endl;
    return;
  }

  // Requeue request with proper error checking
  if (camera_->queueRequest(request) < 0) {
    std::cerr << "Failed to requeue request" << std::endl;
    return;
  }

  // Existing queue management...
  std::lock_guard<std::mutex> lock(requestMutex_);
  completedRequests_.push(request);
  requestCv_.notify_one();
}

/**
 * @brief Static callback wrapper for request completion.
 *
 * @details This static function adapts between libcamera's callback system
 * and the object-oriented design of FrameForger. It retrieves the FrameForger
 * instance from the user_data pointer and forwards the request to the
 * instance's requestComplete method.
 *
 * @param request Pointer to the completed capture request
 * @param userData Pointer to the FrameForger instance (cast to void*)
 *
 * @see requestComplete()
 * @see libcamera::Signal
 */
void FrameForger::requestCallback(libcamera::Request* request, void* userData) {
  FrameForger* self = static_cast<FrameForger*>(userData);
  self->requestComplete(request);
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
  // Unmap all memory
  for (auto& [fd, mapping] : fdMappings_) {
    munmap(mapping.first, mapping.second);
  }
  fdMappings_.clear();

  // Clear frame buffer - Implemented direct clear
  {
    std::lock_guard<std::mutex> lock(frameBufferMutex_);
    std::queue<Frame> empty;
    std::swap(frameBuffer_, empty);
  }

  releaseCamera();
  if (cameraManager_) {
    cameraManager_->stop();
  }
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

/**
 * @brief Main function demonstrating FrameForger usage.
 *
 * @details This function provides a complete example of using the FrameForger
 * class:
 * 1. Sets up signal handlers for graceful termination
 * 2. Creates and initializes a FrameForger instance
 * 3. Starts frame capture
 * 4. Processes frames in a loop until interrupted
 * 5. For each frame, displays metadata and basic statistics
 * 6. Stops capture and cleans up when interrupted
 *
 * @return 0 on successful completion, or an error code on failure
 *
 * @see FrameForger::initialize()
 * @see FrameForger::startCapture()
 * @see FrameForger::getNextFrame()
 * @see FrameForger::stopCapture()
 */
int main() {
  // Set up signal handling for graceful termination
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  std::cout << "Starting FrameForger demo..." << std::endl;

  // Initialize FrameForger
  FrameForger forger;
  int result = forger.initialize();
  if (result != FrameForger::ErrorCode::SUCCESS) {
    std::cerr << "Failed to initialize FrameForger: error code " << result
              << std::endl;
    return result;
  }

  // Start capturing frames
  result = forger.startCapture();
  if (result != FrameForger::ErrorCode::SUCCESS) {
    std::cerr << "Failed to start capture: error code " << result << std::endl;
    return result;
  }

  std::cout << "Capture started successfully. Press Ctrl+C to exit."
            << std::endl;
  std::cout << "---------------------------------------------------"
            << std::endl;

  // Frame counter
  unsigned int frameCount = 0;

  // Process frames until interrupted
  while (running) {
    FrameForger::Frame frame;
    if (forger.getNextFrame(frame)) {
      frameCount++;

      // Print frame metadata
      std::cout << "Frame #" << std::setw(6) << std::setfill('0') << frameCount
                << std::endl;
      std::cout << "  Timestamp: " << frame.metadata.timestamp.count() << " μs"
                << std::endl;
      std::cout << "  Data size: " << frame.data.size() << " bytes"
                << std::endl;

      // Calculate and print basic frame statistics
      if (!frame.data.empty()) {
        // Calculate average pixel intensity (simple metric)
        uint64_t sum = 0;
        for (const auto& byte : frame.data) {
          sum += byte;
        }
        double average = static_cast<double>(sum) / frame.data.size();

        std::cout << "  Average intensity: " << std::fixed
                  << std::setprecision(2) << average << std::endl;

        // Print first few RGB values as a sample
        std::cout << "  Sample RGB values (first 3 pixels): ";
        for (size_t i = 0; i < 9 && i < frame.data.size(); i += 3) {
          if (i > 0) std::cout << ", ";
          std::cout << "(" << static_cast<int>(frame.data[i]) << ","
                    << static_cast<int>(frame.data[i + 1]) << ","
                    << static_cast<int>(frame.data[i + 2]) << ")";
        }
        std::cout << std::endl;
      }

      std::cout << "---------------------------------------------------"
                << std::endl;
    } else {
      // No frame available, sleep a bit to prevent CPU spinning
      std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 fps
    }
  }

  std::cout << "Capture stopped. Processed " << frameCount << " frames."
            << std::endl;

  // Stop capture and clean up
  forger.stopCapture();

  return 0;
}
