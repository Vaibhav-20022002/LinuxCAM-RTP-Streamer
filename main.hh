#pragma once

#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>

#include <algorithm>
#include <csignal>
#include <iostream>
#include <vector>

/**
 * @brief Global flag for signal handling
 * @details Using volatile sig_atomic_t ensures atomic access across signal
 * handlers
 */
inline volatile sig_atomic_t g_running = 1;

/**
 * @brief Signal handler for graceful shutdown
 * @param signal Signal number received
 */
inline void handleSignal(int signal) {
  printf("\n[INFO] Signal %d received, shutting down...\n", signal);
  g_running = 0;  // Set flag instead of calling exit()
}

/**
 * @brief Register signal handlers for SIGINT, SIGTERM, SIGHUP
 */
inline void registerSignals() {
  struct sigaction sa;
  sa.sa_handler = handleSignal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;

  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
  sigaction(SIGHUP, &sa, nullptr);
}

/**
 * @brief Get current timestamp for logging
 * @return Formatted timestamp string
 */
inline const char* getTimestamp() {
  static char buffer[24];
  time_t now = time(nullptr);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&now));
  return buffer;
}

/**
 * @brief Log messages with timestamp (variadic template)
 * @param format Format string
 * @param args Variable arguments to format string
 */
template <typename... Args>
inline void log(const char* format, Args... args) {
  printf("[%s] ", getTimestamp());
  printf(format, args...);
  printf("\n");
}

/**
 * @brief Log simple string messages
 * @param message Message to log
 */
inline void log(const char* message) {
  printf("[%s] %s\n", getTimestamp(), message);
}

/**
 * @brief Format information structure
 */
struct FormatInfo {
  uint32_t pixelFormat;     ///< V4L2 pixel format code
  const char* description;  ///< Format description
};

/**
 * @brief Format priority list (descending order)
 * @details Static to avoid recreating this vector each time
 */
static const std::vector<uint32_t> priorityFormats = {
    V4L2_PIX_FMT_MJPEG,   // Compressed formats first
    V4L2_PIX_FMT_YUV420,  // Common planar YUV
    V4L2_PIX_FMT_YUYV,    // Packed YUV
    V4L2_PIX_FMT_RGB24    // Direct RGB
};

/**
 * @brief Best configuration found during device querying
 */
struct bestConfig {
  uint32_t format = 0;           ///< V4L2 pixel format code
  const char* description = "";  ///< Format description
  uint32_t width = 0;            ///< Frame width
  uint32_t height = 0;           ///< Frame height
  v4l2_fract interval = {0, 0};  ///< Frame interval (for FPS calculation)
};

// Global variables
extern int fd;       ///< File descriptor for video device
extern int errCode;  ///< Last error code

/**
 * @brief Wrapper for ioctl() which retries on interrupt
 * @param fd File descriptor
 * @param request Request code
 * @param arg Request argument
 * @return Result code (negative on error)
 */
int xioctl(int fd, int request, void* arg);

/**
 * @brief Opens a video device with specified flags
 * @param device Device path
 * @param flag Open flags
 * @return File descriptor or -1 on error
 */
int openDevice(const char* device, int flag = O_RDWR);

/**
 * @brief Queries device capabilities
 * @return Capability structure or empty structure on error
 */
v4l2_capability queryDeviceCapa();

/**
 * @brief Gets supported formats and descriptions
 * @return Vector of supported format information
 */
std::vector<FormatInfo> getFormatsAndDesc();

/**
 * @brief Gets the best supported configuration based on priority
 * @param formatInfos Reference to format information vector
 * @return Best configuration found
 */
bestConfig getSupportedPriorityConfig(
    const std::vector<FormatInfo>& formatInfos);

/**
 * @brief Applies frame rate configuration to device
 * @param bestPriorityConfig Reference to best configuration
 * @return Stream parameters or empty structure on error
 */
v4l2_streamparm applyFrameRateConfig(const bestConfig& bestPriorityConfig);