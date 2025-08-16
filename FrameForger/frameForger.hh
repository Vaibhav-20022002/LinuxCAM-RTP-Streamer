#pragma once

#include "defines.hh"

#if defined(__linux__)
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <atomic>
#include <cerrno>
#include <csignal> // For sig_atomic_t
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string_view>
#include <vector>

// Signal flag (set by an async-signal-safe handler in main).
extern volatile sig_atomic_t g_sigint_received;

///\class FrameForger
///\brief A RAII-compliant class for capturing video frames using V4L2.
///
/// This class uses memory-mapped I/O and a zero-copy callback mechanism.
/// Resource management is handled automatically via the constructor and destructor.
class FrameForger {
public:
  ///\brief Constructs and initializes the V4L2 device.
  ///\param devicePath A view of the path to the video device (e.g., "/dev/video0").
  ///\param width The desired frame width.
  ///\param height The desired frame height.
  ///\param format The desired V4L2 pixel format (e.g., V4L2_PIX_FMT_MJPEG).
  ///\throws std::runtime_error if any critical initialization step fails.
  FrameForger(std::string_view devicePath, uint32_t width, uint32_t height, uint32_t format);

  ///\brief Destructor. Automatically stops streaming, unmaps buffers, and closes the device.
  ~FrameForger();

  // **--- Deleted copy and move semantics to prevent resource mismanagement ---**
  // This class manages a unique hardware resource (the camera), so copying it doesn't make sense.
  // A move constructor could be implemented, but is omitted for simplicity and safety.
  FrameForger(const FrameForger &)            = delete;
  FrameForger &operator=(const FrameForger &) = delete;

  ///\brief Starts the video stream.
  ///\note This must be called before starting the streaming loop.
  void startStream();

  ///\brief The main capture loop.
  /// This function blocks and continuously captures frames until stop() is called.
  ///\param frameHandler A callback function that will be invoked for each captured frame.
  ///                    It receives the frame via move semantics.
  void streamingLoop(std::function<void(Frame &&)> frameHandler);

  ///\brief Signals the streaming loop to stop gracefully.
  ///\note This function is thread-safe and can be called from a signal handler.
  void stop();

  // **------ Getter Functions ------**

  ///\brief Gets the camera's file descriptor.
  ///\return The integer file descriptor, or -1 if not open.
  int getFD() const { return fd; }

  ///\brief Gets the configured frame width.
  int getWidth() const { return width; }

  ///\brief Gets the configured frame height.
  int getHeight() const { return height; }

private:
  ///\struct BufferInfo
  ///\brief A simple struct to hold information about a single memory-mapped buffer.
  struct BufferInfo {
    void  *start;  ///< Pointer to the start of the mapped memory region.
    size_t length; ///< The total size of the buffer in bytes.
  };

  // **--- Member Variables ---**
  int      fd = -1; ///< The file descriptor for the video device.
  uint32_t width;   ///< Configured frame width.
  uint32_t height;  ///< Configured frame height.

  std::vector<BufferInfo> buffers;            ///< A vector holding info for each mmap'd buffer.
  std::atomic<bool>       isStreaming{false}; ///< Atomic flag to control the main loop.
  std::atomic<bool> streamOn{false}; ///< Tracks whether STREAMON succeeded and needs STREAMOFF.

  // **--- Private Helper Methods for Initialization and Control ---**

  ///\brief A simple static wrapper function to handle ioctl errors robustly.
  /// It automatically retries the call if it's interrupted by a signal (EINTR).
  ///\param fd The file descriptor.
  ///\param request The ioctl request code.
  ///\param arg A pointer to the argument for the ioctl call.
  ///\throws std::runtime_error if the ioctl call fails for a reason other than EINTR.
  static void xioctl(int fd, int request, void *arg);

  ///\brief Opens the device file and sets the file descriptor.
  void openDevice(std::string_view devicePath);

  ///\brief Queries device capabilities and sets the desired video format.
  void initializeDevice(uint32_t format);

  ///\brief Requests, queries, and memory-maps the V4L2 buffers.
  void initializeBuffers(uint8_t reqBufferCount = 4);

  ///\brief Queues all mapped buffers to make them available to the driver.
  void queueAllBuffers();

  ///\brief Stops the video stream if it is running.
  void stopStream();
};