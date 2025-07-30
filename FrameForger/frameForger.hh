#pragma once

#include "../defines.hh"

#if defined(__linux__)
    #include <linux/videodev2.h>
    #include <sys/ioctl.h>
    #include <sys/types.h>
    #include <unistd.h>
#endif

#include <cstddef>
#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>

///\brief A simple function to handle ioctl errors
inline void xioctl(int fd, int request, void *arg) {
  int ret;
  // Keep trying the ioctl call if it's interrupted by a signal
  do {
    ret = ioctl(fd, request, arg);
  } while (-1 == ret && EINTR == errno);

  if (-1 == ret) {
    FAIL_MSG("ioctl request failed");
    exit(EXIT_FAILURE);
  }
}

class FrameForger {
private:
  std::string device;
  int         fd;

  // Structs to handle device settings and supports
  struct v4l2_capability capa;
  struct v4l2_format     fmt;
  struct v4l2_requestbuffers buffReq;

  // Mapped buffer pointers
  void **buffPtrs;

  ///\brief Query the camera capture capabilities
  ///\return true if the device supports video capture, false otherwise
  ///\note This function should be called after opening the device.
  bool queryCaptureCapa();

  ///\brief Query the camera streaming capabilities
  ///\return true if the device supports streaming, false otherwise
  ///\note This function should be called after opening the device.
  bool queryStreamCapa();

  ///\brief Set the format for the video stream
  ///\return true if the format was set successfully, false otherwise
  ///\note This function should be called after opening the device and querying capabilities.
  bool setFormat();

public:
  bool isStreamSupported = true; ///< Flag to indicate if streaming is supported
  bool isCaptureSupported = true; ///< Flag to indicate if video capture is supported

  int width; ///< Width of the selected configuration
  int height; ///< width of the selected configuration
  std::string pixelFormatStr; ///< Selected Pixel format string
  uint32_t pixelFormat; ///< Selected V4L2 Pixel format code
  uint32_t pixelField; ///< Selected V4L2 Pixel field

  uint8_t noOfBuffers; ///< Number of allocated & mapped buffers

  ///\brief Constructor
  FrameForger(char *devicePath, int flags);

  ///\brief Destructor
  ~FrameForger();

  ///\brief Open the device and sets device name along with file descriptor.
  ///\param devicePath Path to the device (e.g., "/dev/video0")
  ///\param flags Flags to control the opening behaviour
  ///\note Cannot be used for concurrent device
  void openDevice(int flags);

  // **------ Getter Functions ------**

  ///\brief Get camera file descriptor
  ///\note It may return -1 if file descriptor is unset, use with proper handling.
  int getFD();

  ///\brief Get Device name
  ///\note It may return empty string, check & handle it manually.
  std::string getDevice();

  // **------ Setter Functions ------**

  ///\brief Query the device capabilities
  ///\return true if the device supports video capture and streaming, false otherwise
  ///\note This function should be called after opening the device.
  ///\note This function will set the `capa` struct with the device capabilities.
  void queryCapabilities();

  ///\brief Try to set the given format, width & height
  ///\return true if given configuration is set and false upon failure along with error log
  bool setFormat(std::string format, uint32_t width, uint32_t height);

  ///\brief Allocate and map buffers between V4L2 driver and program memory.
  ///\return Number of buffers allocated & mapped.
  ///\note If no count for requested buffer given it by default set to 4.
  ///\note It should be greater than or equal to 2 always. If given it going to allocate 4 buffers without any error.
  ///\note If not able to allocated it returns zero with error log.
  uint8_t allocateBuffers(int reqBufferCount = 4);

  ///\brief Query and Map the allocated buffers
  ///\return true if all buffers are mapped successfully, else false along with error message.
  ///\note If all the buffers are able to get mapped, it returns false and require immediate exit
  bool mapBuffers();

  ///\brief Queue every mapped buffer
  void queueBuffers();
};
