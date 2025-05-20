#include "main.hh"

#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

// Global variables
int fd = -1;
int errCode = 0;

/**
 * @brief Wrapper for ioctl() which retries on interrupt
 * @param fd File descriptor
 * @param request Request code
 * @param arg Request argument
 * @return Result code (negative on error)
 */
int xioctl(int fd, int request, void* arg) {
  int ret;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret == -1 && errno == EINTR);
  return ret;
}

/**
 * @brief Opens a video device with specified flags
 * @param device Device path
 * @param flag Open flags
 * @return File descriptor or -1 on error
 */
int openDevice(const char* device, int flag) {
  int fd = open(device, flag);
  if (fd < 0) {
    log("Open failed: %s (%d)", strerror(errno), errno);
  }
  return fd;
}

/**
 * @brief Queries device capabilities
 * @return Capability structure or empty structure on error
 */
v4l2_capability queryDeviceCapa() {
  v4l2_capability capas = {0};
  if ((errCode = xioctl(fd, VIDIOC_QUERYCAP, &capas)) == -1) {
    log("Query capabilities failed: %s", strerror(errno));
    return {0};
  }
  return capas;
}

/**
 * @brief Gets supported formats and descriptions
 * @return Vector of supported format information
 */
std::vector<FormatInfo> getFormatsAndDesc() {
  // Reserve initial capacity to avoid reallocations
  std::vector<FormatInfo> formatInfos;
  formatInfos.reserve(8);  // Reasonable assumption for most cameras

  for (uint32_t idx = 0;; ++idx) {
    v4l2_fmtdesc fmtDesc = {};
    fmtDesc.index = idx;
    fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if ((errCode = xioctl(fd, VIDIOC_ENUM_FMT, &fmtDesc)) == -1) {
      if (errno == EINVAL) break;  // Normal enumeration end

      log("Format enumeration failed: %s", strerror(errno));
      formatInfos.clear();  // Release memory before return
      return formatInfos;
    }

    // Store pointer to description to avoid string copies
    formatInfos.push_back({fmtDesc.pixelformat,
                           reinterpret_cast<const char*>(fmtDesc.description)});
  }
  return formatInfos;
}

/**
 * @brief Finds the highest frame rate for given format and resolution
 * @param fmt Pixel format
 * @param width Frame width
 * @param height Frame height
 * @return Best frame interval found
 */
v4l2_fract findBestFrameRate(uint32_t fmt, uint32_t width, uint32_t height) {
  v4l2_fract bestInterval = {0, 0};

  for (uint32_t intvIdx = 0;; ++intvIdx) {
    v4l2_frmivalenum frmIntv = {};
    frmIntv.index = intvIdx;
    frmIntv.pixel_format = fmt;
    frmIntv.width = width;
    frmIntv.height = height;

    if ((errCode = xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmIntv)) == -1) {
      if (errno == EINVAL) break;
      log("Interval enumeration failed: %s", strerror(errno));
      return {0, 0};
    }

    // Process based on interval type
    if (frmIntv.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
      if (frmIntv.discrete.denominator == 0) continue;

      const float currentFps =
          static_cast<float>(frmIntv.discrete.denominator) /
          frmIntv.discrete.numerator;
      const float bestFps = (bestInterval.denominator == 0)
                                ? 0.0f
                                : static_cast<float>(bestInterval.denominator) /
                                      bestInterval.numerator;

      if (bestInterval.denominator == 0 || currentFps > bestFps) {
        bestInterval = frmIntv.discrete;
      }
    } else if (frmIntv.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
      if (frmIntv.stepwise.min.denominator == 0) continue;

      const float maxFps =
          static_cast<float>(frmIntv.stepwise.min.denominator) /
          frmIntv.stepwise.min.numerator;
      const float bestFps = (bestInterval.denominator == 0)
                                ? 0.0f
                                : static_cast<float>(bestInterval.denominator) /
                                      bestInterval.numerator;

      if (bestInterval.denominator == 0 || maxFps > bestFps) {
        bestInterval = frmIntv.stepwise.min;
      }
    }
  }

  return bestInterval;
}

/**
 * @brief Gets the best supported configuration based on priority
 * @param formatInfos Reference to format information vector
 * @return Best configuration found
 */
bestConfig getSupportedPriorityConfig(
    const std::vector<FormatInfo>& formatInfos) {
  bestConfig config = {};

  // Use find_if with lambda to check if priority format is supported
  for (auto priorityFmt : priorityFormats) {
    // Find matching format in supported formats
    auto it = std::find_if(formatInfos.begin(), formatInfos.end(),
                           [priorityFmt](const FormatInfo& info) {
                             return info.pixelFormat == priorityFmt;
                           });

    if (it == formatInfos.end()) continue;

    // Find maximum resolution for this format
    uint32_t maxWidth = 0, maxHeight = 0;
    for (uint32_t sizeIdx = 0;; ++sizeIdx) {
      v4l2_frmsizeenum frmSize = {};
      frmSize.index = sizeIdx;
      frmSize.pixel_format = priorityFmt;

      if ((errCode = xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmSize)) == -1) {
        if (errno == EINVAL) break;
        log("Frame size enumeration failed: %s", strerror(errno));
        return {};
      }

      if (frmSize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        const uint64_t res = static_cast<uint64_t>(frmSize.discrete.width) *
                             frmSize.discrete.height;
        if (res > static_cast<uint64_t>(maxWidth) * maxHeight) {
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
    v4l2_fract bestInterval =
        findBestFrameRate(priorityFmt, maxWidth, maxHeight);
    if (bestInterval.denominator == 0) continue;

    // Update best configuration
    config.format = priorityFmt;
    config.description = it->description;  // Store pointer to avoid copy
    config.width = maxWidth;
    config.height = maxHeight;
    config.interval = bestInterval;
    break;  // Found highest priority format
  }

  return config;
}

/**
 * @brief Applies frame rate configuration to device
 * @param bestPriorityConfig Reference to best configuration
 * @return Stream parameters or empty structure on error
 */
v4l2_streamparm applyFrameRateConfig(const bestConfig& bestPriorityConfig) {
  v4l2_streamparm streamParams = {0};
  streamParams.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  streamParams.parm.capture.timeperframe = bestPriorityConfig.interval;

  if ((errCode = xioctl(fd, VIDIOC_S_PARM, &streamParams)) == -1) {
    log("Failed to set frame rate: %s", strerror(errno));
    return {0};
  }

  return streamParams;
}

/**
 * @brief Safely closes file descriptor if open
 */
void cleanupResources() {
  if (fd >= 0) {
    close(fd);
    fd = -1;
  }
}

/**
 * @brief Main entry point
 * @param argc Argument count
 * @param argv Argument values
 * @return Exit code
 */
int main(int argc, char** argv) {
  // Check arguments
  if (argc < 2) {
    log("Usage: %s <video_device>", argv[0]);
    return EXIT_FAILURE;
  }

  const char* device = argv[1];
  registerSignals();

  // Initialize with error state
  int exitCode = EXIT_FAILURE;

  // Open device - defer cleanup to end
  if ((fd = openDevice(device, O_RDWR | O_NONBLOCK)) == -1) {
    cleanupResources();
    return EXIT_FAILURE;
  }

  // Query device capabilities
  v4l2_capability capas = queryDeviceCapa();
  if (capas.capabilities == 0) {
    cleanupResources();
    return EXIT_FAILURE;
  }

  log("Driver: %s", capas.driver);
  log("Card: %s", capas.card);

  // Validate required capabilities
  if (!(capas.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    log("Device lacks video capture support");
    cleanupResources();
    return EXIT_FAILURE;
  }

  if (!(capas.capabilities & V4L2_CAP_STREAMING)) {
    log("Device lacks streaming support");
    cleanupResources();
    return EXIT_FAILURE;
  }

  // Get and check format information
  std::vector<FormatInfo> formatInfos = getFormatsAndDesc();
  if (formatInfos.empty()) {
    log("No format available. Exiting...");
    cleanupResources();
    return EXIT_SUCCESS;  // This is considered a normal exit
  }

  // Get best supported priority configuration
  bestConfig bestPriorityConfig = getSupportedPriorityConfig(formatInfos);
  if (bestPriorityConfig.format == 0) {
    log("No supported configuration found");
    cleanupResources();
    return EXIT_FAILURE;
  }

  // Apply frame rate configuration
  v4l2_streamparm streamParams = applyFrameRateConfig(bestPriorityConfig);
  if (streamParams.type == 0) {
    log("Failed to set frame rate: %s", strerror(errno));
    // Continue anyway as we might still have useful information
  }

  // Final output
  const float fps =
      static_cast<float>(bestPriorityConfig.interval.denominator) /
      bestPriorityConfig.interval.numerator;
  log("Selected configuration:");
  log("\tFormat: %s", bestPriorityConfig.description);
  log("\tResolution: %dx%d", bestPriorityConfig.width,
      bestPriorityConfig.height);
  log("\tFrame rate: %.2f FPS", fps);

  // Success
  log("Configuration complete");
  cleanupResources();
  return EXIT_SUCCESS;
}