#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <unordered_map>
#include <vector>

// Signal handler for graceful shutdown
inline void handleSignal(int signal) {
  printf("\n[INFO] Signal %d received, shutting down...\n", signal);
  exit(signal);
}

// Register signal handlers for SIGINT, SIGTERM, SIGHUP
inline void registerSignals() {
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
  std::signal(SIGHUP, handleSignal);
}

// Get current timestamp for logging
inline const char* getTimestamp() {
  static char buffer[24];
  time_t now = time(nullptr);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&now));
  return buffer;
}

// Log messages with timestamp (variadic template)
template <typename... Args>
inline void log(const char* format, Args... args) {
  printf("[%s] ", getTimestamp());
  printf(format, args...);
  printf("\n");
}

// Overload for simple string messages
inline void log(const char* message) {
  printf("[%s] %s\n", getTimestamp(), message);
}

// Wrapper for ioctl with retry on interrupt
int xioctl(int fd, int request, void* arg) {
  int ret;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret == -1 && errno == EINTR);
  return ret;
}

struct FormatInfo {
  uint32_t pixelFormat;
  std::string description;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    log("Usage: %s <video_device>", argv[0]);
    return EXIT_FAILURE;
  }
  const char* device = argv[1];

  registerSignals();

  // Open video device
  log("Opening device: %s", device);
  int fd = open(device, O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    log("Open failed: %s (%d)", strerror(errno), errno);
    return EXIT_FAILURE;
  }

  // Query device capabilities
  v4l2_capability caps = {};
  if (xioctl(fd, VIDIOC_QUERYCAP, &caps) == -1) {
    log("Query capabilities failed: %s", strerror(errno));
    close(fd);
    return EXIT_FAILURE;
  }

  log("Driver: %s", caps.driver);
  log("Card: %s", caps.card);

  // Validate required capabilities
  if (!(caps.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    log("Device lacks video capture support");
    close(fd);
    return EXIT_FAILURE;
  }
  if (!(caps.capabilities & V4L2_CAP_STREAMING)) {
    log("Device lacks streaming support");
    close(fd);
    return EXIT_FAILURE;
  }

  // Gather supported pixel formats with descriptions
  std::vector<FormatInfo> formatInfos;
  for (uint32_t idx = 0;; ++idx) {
    v4l2_fmtdesc fmtDesc = {};
    fmtDesc.index = idx;
    fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_ENUM_FMT, &fmtDesc) == -1) {
      if (errno == EINVAL) break;  // Normal enumeration end
      log("Format enumeration failed: %s", strerror(errno));
      close(fd);
      return EXIT_FAILURE;
    }
    formatInfos.push_back({fmtDesc.pixelformat,
                           reinterpret_cast<const char*>(fmtDesc.description)});
  }

  // Format priority list (descending order)
  const std::vector<uint32_t> priorityFormats = {
      V4L2_PIX_FMT_MJPEG,   // Compressed formats first
      V4L2_PIX_FMT_YUV420,  // Common planar YUV
      V4L2_PIX_FMT_YUYV,    // Packed YUV
      V4L2_PIX_FMT_RGB24    // Direct RGB
  };

  // Track best configuration found
  struct {
    uint32_t format = 0;
    std::string description;
    uint32_t width = 0;
    uint32_t height = 0;
    v4l2_fract interval = {0, 0};
  } bestConfig;

  // Find highest priority supported format
  for (auto fmt : priorityFormats) {
    bool formatSupported = false;
    std::string fmtDescription;
    for (const auto& info : formatInfos) {
      if (info.pixelFormat == fmt) {
        formatSupported = true;
        fmtDescription = info.description;
        break;
      }
    }
    if (!formatSupported) continue;

    // Find maximum resolution for this format
    uint32_t maxWidth = 0, maxHeight = 0;
    for (uint32_t sizeIdx = 0;; ++sizeIdx) {
      v4l2_frmsizeenum frmSize = {};
      frmSize.index = sizeIdx;
      frmSize.pixel_format = fmt;
      if (xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmSize) == -1) {
        if (errno == EINVAL) break;
        log("Frame size enumeration failed: %s", strerror(errno));
        close(fd);
        return EXIT_FAILURE;
      }

      if (frmSize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        const uint64_t res = frmSize.discrete.width * frmSize.discrete.height;
        if (res > maxWidth * maxHeight) {
          maxWidth = frmSize.discrete.width;
          maxHeight = frmSize.discrete.height;
        }
      } else if (frmSize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
        maxWidth = frmSize.stepwise.max_width;
        maxHeight = frmSize.stepwise.max_height;
        break;
      }
    }

    if (maxWidth == 0 || maxHeight == 0) continue;

    // Find best frame interval for chosen resolution
    v4l2_fract bestInterval = {0, 0};
    for (uint32_t intvIdx = 0;; ++intvIdx) {
      v4l2_frmivalenum frmIntv = {};
      frmIntv.index = intvIdx;
      frmIntv.pixel_format = fmt;
      frmIntv.width = maxWidth;
      frmIntv.height = maxHeight;
      if (xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmIntv) == -1) {
        if (errno == EINVAL) break;
        log("Interval enumeration failed: %s", strerror(errno));
        close(fd);
        return EXIT_FAILURE;
      }

      if (frmIntv.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
        if (frmIntv.discrete.denominator == 0) continue;

        const double currentFps =
            static_cast<double>(frmIntv.discrete.denominator) /
            frmIntv.discrete.numerator;
        const double bestFps =
            (bestInterval.denominator == 0)
                ? 0.0
                : static_cast<double>(bestInterval.denominator) /
                      bestInterval.numerator;

        if (bestInterval.denominator == 0 || currentFps > bestFps) {
          bestInterval = frmIntv.discrete;
        }
      } else if (frmIntv.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
        if (frmIntv.stepwise.min.denominator == 0) continue;

        const double maxFps =
            static_cast<double>(frmIntv.stepwise.min.denominator) /
            frmIntv.stepwise.min.numerator;
        const double bestFps =
            (bestInterval.denominator == 0)
                ? 0.0
                : static_cast<double>(bestInterval.denominator) /
                      bestInterval.numerator;

        if (bestInterval.denominator == 0 || maxFps > bestFps) {
          bestInterval = frmIntv.stepwise.min;
        }
      }
    }

    if (bestInterval.denominator == 0) continue;

    // Update best configuration
    bestConfig.format = fmt;
    bestConfig.description = fmtDescription;
    bestConfig.width = maxWidth;
    bestConfig.height = maxHeight;
    bestConfig.interval = bestInterval;
    break;  // Found highest priority format
  }

  if (bestConfig.format == 0) {
    log("No supported configuration found");
    close(fd);
    return EXIT_FAILURE;
  }

  // Apply frame rate configuration
  v4l2_streamparm streamParams = {};
  streamParams.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  streamParams.parm.capture.timeperframe = bestConfig.interval;
  if (xioctl(fd, VIDIOC_S_PARM, &streamParams) == -1) {
    log("Failed to set frame rate: %s", strerror(errno));
  }

  // Final output
  const double fps = static_cast<double>(bestConfig.interval.denominator) /
                     bestConfig.interval.numerator;
  log("Selected configuration:");
  log("\tFormat: %s", bestConfig.description.c_str());
  log("\tResolution: %dx%d", bestConfig.width, bestConfig.height);
  log("\tFrame rate: %.2f FPS", fps);

  close(fd);
  log("Configuration complete");
  return EXIT_SUCCESS;
}