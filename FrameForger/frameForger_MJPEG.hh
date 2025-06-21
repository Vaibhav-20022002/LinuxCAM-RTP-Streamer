/**
 * @brief Camera frame acquisition and processing system
 * @details The FrameForger_MJPEG class provides a high-level interface for camera
 * operations using libcamera. It handles initialization, configuration,
 * frame capture, and buffer management.
 */
#pragma once

#include "frameTypes.hh"
#include "../main.hh"
#include "ringMaster.hh"
#include <chrono>
#include <iostream>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/pixel_format.h>
#include <libcamera/stream.h>
#include <memory>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

class FrameForger_MJPEG {
public:
  enum ErrorCode {
    SUCCESS                  = 0,
    ERR_CAMERA_MANAGER_START = -1,
    ERR_NO_CAMERAS           = -2,
    ERR_CAMERA_ACQUIRE       = -3,
    ERR_CONFIG_GENERATION    = -4,
    ERR_CONFIG_VALIDATION    = -5,
    ERR_CONFIG_APPLICATION   = -6,
    ERR_BUFFER_ALLOCATION    = -7,
    ERR_REQUEST_CREATION     = -8,
    ERR_REQUEST_BUFFER       = -9,
    ERR_CAMERA_START         = -10,
    ERR_QUEUE_REQUEST        = -11
  };

  /**
   * @brief Construct a new FrameForger_MJPEG object
   * @param mjpegBuffer Reference to ring buffer for storing frames
   * @param config Camera configuration parameters
   */
  explicit FrameForger_MJPEG(RingMaster<MjpegFrame> &mjpegBuffer, const CameraConfig &config);

  ~FrameForger_MJPEG();

  bool isInitialized() const { return initialized_; }
  int  initialize();
  int  startCapture();
  void stopCapture();
  bool isCapturing() const { return capturing_.load(); }
  bool getNextFrame(MjpegFrame &frame);

private:
  using CameraManagerPtr       = std::unique_ptr<libcamera::CameraManager>;
  using CameraConfigurationPtr = std::unique_ptr<libcamera::CameraConfiguration>;
  using RequestPtr             = std::unique_ptr<libcamera::Request>;

  CameraManagerPtr                                 cameraManager_;
  std::shared_ptr<libcamera::Camera>               camera_;
  CameraConfigurationPtr                           cameraConfig_;
  libcamera::Stream                               *stream_ = nullptr;
  std::unique_ptr<libcamera::FrameBufferAllocator> bufferAllocator_;

  bool                                               cameraAcquired_ = false;
  std::atomic<bool>                                  capturing_{false};
  unsigned int                                       buffersAllocated_ = 0;
  std::vector<RequestPtr>                            requests_;
  std::unordered_map<int, std::pair<void *, size_t>> fdMappings_;
  size_t                                             frameSize_ = 0;
  uint32_t                                           width_;
  uint32_t                                           height_;
  uint64_t                                           seqCounter_ = 0;

  bool                initialized_ = false;
  const CameraConfig &config_;

  RingMaster<MjpegFrame> &rgbBuffer_;

  int         setupCameraManager();
  int         selectCamera();
  int         configureStream();
  int         allocateBuffers();
  int         createRequests();
  void        cleanup();
  void        releaseCamera();
  void        requestComplete(libcamera::Request *request);
  static void requestCallback(libcamera::Request *request, void *user_data);
};