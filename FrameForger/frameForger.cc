#include "frameForger.hh"

#include <csignal> // For sig_atomic_t
#include <cstdint>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept> // For std::runtime_error
#include <sys/ioctl.h>
#include <sys/mman.h>

// Extern signal flag set by main's handler (async-signal-safe)
extern volatile sig_atomic_t g_sigint_received;

// **--- Constructor ---**

// All initialization is performed here. If any step fails, an exception is thrown,
// and the object is not constructed, preventing use of a partially-initialized object.
FrameForger::FrameForger(std::string_view devicePath,
        uint32_t                          w,
        uint32_t                          h,
        uint32_t                          format,
        uint32_t                          fps)
    : width(w)
    , height(h) {
  // The order of these calls is critical for correct V4L2 setup.
  openDevice(devicePath);
  initializeDevice(format, fps);
  initializeBuffers(); // Allocate and map buffers
}

// **--- Destructor ---**

// It's guaranteed to be called when the object goes out of scope,
// ensuring all acquired resources are released correctly.
FrameForger::~FrameForger() {
  // The order of release is the reverse of acquisition.
  if (fd != -1) {
    // Stop the stream first, if it's running.
    stopStream();

    // Unmap all buffers from this process's memory space.
    for (const auto &buffer : buffers) {
      if (munmap(buffer.start, buffer.length) == -1) {
        // Log error but continue cleanup. Can't throw in a destructor.
        ERROR_MSG("Failed to unmap buffer. Reason: %s", strerror(errno));
      }
    }
    HIGH_MSG("All buffers unmapped.");

    // Finally, close the device file descriptor.
    if (close(fd) == -1) {
      ERROR_MSG("Failed to close device descriptor. Reason: %s", strerror(errno));
    }
    HIGH_MSG("Device closed.");
  }
}

// **--- Public Methods ---**

void FrameForger::startStream() {
  // This function must be called before the streaming loop begins.
  // It queues the buffers and tells the driver to start capturing.
  queueAllBuffers();
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(fd, VIDIOC_STREAMON, &type);
  streamOn    = true; // Mark that STREAMON succeeded and needs STREAMOFF
  isStreaming = true; // Set the loop control flag
  INFO_MSG("Streaming started.");
}

void FrameForger::stop() {
  // This is the public method to signal the loop to stop.
  isStreaming = false;
}

void FrameForger::stopStream() {
  // This is the internal method that actually issues the ioctl call.
  // We should attempt to call STREAMOFF if we previously called STREAMON.
  if (fd == -1 || !streamOn.load()) {
    // Even if isStreaming is false we should still attempt to stop the stream
    // only when streamOn was set to true earlier.
    return;
  }
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  // We use a raw ioctl call here because we don't want to throw from the destructor.
  if (ioctl(fd, VIDIOC_STREAMOFF, &type) == -1 && errno != ENODEV) {
    ERROR_MSG("VIDIOC_STREAMOFF failed. Reason: %s", strerror(errno));
  }
  streamOn    = false; // Clear the STREAMON marker
  isStreaming = false; // Ensure flag is cleared
  INFO_MSG("Streaming stopped.");
}

// **--- HOT LOOP : Streaming Loop ---**
void FrameForger::streamingLoop(std::function<void(Frame &&)> frameHandler) {
  // Validate preconditions before starting the loop
  if (!isStreaming.load()) {
    WARN_MSG("Stream not started. Call startStream() before streamingLoop().");
    return;
  }
  if (fd == -1) {
    ERROR_MSG("Invalid file descriptor. Device not properly initialized.");
    return;
  }
  INFO_MSG("Entering streaming loop...");

  // Main streaming loop - continues until isStreaming is set to false
  while (isStreaming.load()) {
    // Check for signal-based stop request set by a handler
    if (g_sigint_received) {
      INFO_MSG("SIGINT received, stopping streaming loop.");
      stop();
      break;
    }

    // poll() is more efficient than select() for a single file descriptor.
    // It puts the process to sleep until the driver has data ready.
    // Setup poll structure to monitor the device file descriptor
    // We monitor for:
    // - POLLIN: Data available to read
    // - POLLERR: Error condition
    // - POLLHUP: Hang up (device disconnected)
    struct pollfd fds_poll = {.fd = fd, .events = POLLIN | POLLERR | POLLHUP, .revents = 0};

    // Wait for events with a 2-second timeout
    // This puts the thread to sleep when no data is available, saving CPU cycles
    int ret = poll(&fds_poll, 1, 2000);

    // Handle poll() return status
    if (ret == -1) {
      // Error cases
      if (errno == EINTR) {
        // Interrupted by signal - check if we should continue
        DEBUG_MSG("poll() interrupted by signal");
        // If a signal was received, let the top-of-loop check handle it
        continue;
      }
      // Critical poll error - log and exit
      ERROR_MSG("Critical poll() error: %s. Exiting loop.", strerror(errno));
      isStreaming = false;
      return;
    } else if (ret == 0) {
      // Timeout occurred - device might be slow or stuck
      WARN_MSG("poll() timeout - no frame received in 2 seconds");
      continue;
    }

    // Check for error conditions first
    if (fds_poll.revents & (POLLERR | POLLHUP)) {
      ERROR_MSG("Device error (POLLERR) or hangup (POLLHUP) detected");
      isStreaming = false;
      return;
    }

    // Check if data is actually available to be read.
    if (fds_poll.revents & POLLIN) {
      struct v4l2_buffer buff = {};
      buff.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buff.memory             = V4L2_MEMORY_MMAP;

      // Dequeue a buffer with the captured frame data.
      // This is a non-blocking call because poll() told us data is ready.
      if (ioctl(fd, VIDIOC_DQBUF, &buff) == -1) {
        ERROR_MSG("Failed to dequeue buffer. Reason: %s", strerror(errno));
        // ENODEV means device was disconnected
        if (errno == ENODEV) {
          ERROR_MSG("Device disconnected - stopping stream");
          isStreaming = false;
          return;
        }
        // For other errors, try again on next iteration
        continue;
      }

      // Validate buffer index to prevent array out-of-bounds
      if (buff.index >= buffers.size()) {
        ERROR_MSG("Invalid buffer index %u (max %zu)", buff.index, buffers.size());
        isStreaming = false;
        return;
      }

      // **--- ZERO-COPY FRAME HANDLING ---**
      // 1. Create a Frame object on the stack. No heap allocation.
      // 2. Pass it to the user's callback using std::move to transfer ownership
      //    without copying the data structure.
      try {
        // Log frame metadata before passing to handler
        // INFO_MSG("Frame Metadata:");
        // INFO_MSG("  - Buffer Index: %u", buff.index);
        // INFO_MSG("  - Size: %u bytes", buff.bytesused);
        // INFO_MSG("  - Timestamp: %ld.%06ld sec", buff.timestamp.tv_sec, buff.timestamp.tv_usec);
        // INFO_MSG("  - Sequence: %u", buff.sequence);
        // INFO_MSG("  - Flags: %s%s%s",
        //     (buff.flags & V4L2_BUF_FLAG_KEYFRAME) ? "KEYFRAME " : "",
        //     (buff.flags & V4L2_BUF_FLAG_PFRAME) ? "PFRAME " : "",
        //     (buff.flags & V4L2_BUF_FLAG_BFRAME) ? "BFRAME " : "");
        frameHandler(Frame{.start = buffers[buff.index].start,
                .length           = buff.bytesused,
                .timestamp        = buff.timestamp});
      } catch (const std::exception &e) {
        // Handle exceptions from frameHandler to prevent stream termination
        ERROR_MSG("Frame handler threw exception: %s", e.what());
      }

      // Immediately requeue the buffer so the driver can use it for the next frame.
      // Failing to do this will stall the stream once all buffers are held by the application.
      if (ioctl(fd, VIDIOC_QBUF, &buff) == -1) {
        ERROR_MSG("Failed to requeue buffer. Reason: %s", strerror(errno));
        // ENODEV means device was disconnected
        if (errno == ENODEV) {
          ERROR_MSG("Device disconnected - stopping stream");
        }
        isStreaming = false; // This is a critical error, stop the stream.
      }
    } else {
      // Unexpected poll event - log and continue
      DEBUG_MSG("Unexpected poll event: 0x%x", fds_poll.revents);
    }
  }
  INFO_MSG("Exited streaming loop.");
}

// **--- Private Helper Implementations ---**

void FrameForger::xioctl(int fd, int request, void *arg) {
  int ret;
  // Keep trying the ioctl call if it's interrupted by a signal (EINTR).
  // This makes system calls more robust.
  do {
    ret = ioctl(fd, request, arg);
  } while (ret == -1 && errno == EINTR);

  if (ret == -1) {
    // If ioctl fails for any other reason, it's a fatal error for setup.
    char error_buf[256];
    snprintf(error_buf,
            sizeof(error_buf),
            "ioctl request 0x%x (%s) failed. Reason: %s",
            (unsigned)request, // Cast to unsigned for comparison
            request == (int)VIDIOC_QUERYCAP            ? "VIDIOC_QUERYCAP"
                    : request == (int)VIDIOC_S_FMT     ? "VIDIOC_S_FMT"
                    : request == (int)VIDIOC_REQBUFS   ? "VIDIOC_REQBUFS"
                    : request == (int)VIDIOC_QUERYBUF  ? "VIDIOC_QUERYBUF"
                    : request == (int)VIDIOC_QBUF      ? "VIDIOC_QBUF"
                    : request == (int)VIDIOC_DQBUF     ? "VIDIOC_DQBUF"
                    : request == (int)VIDIOC_STREAMON  ? "VIDIOC_STREAMON"
                    : request == (int)VIDIOC_STREAMOFF ? "VIDIOC_STREAMOFF"
                                                       : "Unknown",
            strerror(errno));
    throw std::runtime_error(error_buf);
  }
}

void FrameForger::openDevice(std::string_view devicePath) {
  fd = open(devicePath.data(), O_RDWR | O_NONBLOCK, 0);
  if (fd == -1) {
    throw std::runtime_error("Cannot open device. Check path and permissions.");
  }
  HIGH_MSG("Device opened successfully: %s", devicePath.data());
}

void FrameForger::findLowestResolution(uint32_t format) {
  struct v4l2_frmsizeenum fsize;
  fsize.pixel_format = format;
  fsize.index        = 0;
  uint32_t min_area  = -1; // Max uint32_t

  INFO_MSG("Auto-detecting lowest resolution for format...");
  while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fsize) == 0) {
    if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
      uint32_t area = fsize.discrete.width * fsize.discrete.height;
      if (area < min_area) {
        min_area = area;
        width    = fsize.discrete.width;
        height   = fsize.discrete.height;
      }
    }
    fsize.index++;
  }

  if (min_area == (uint32_t)-1) {
    throw std::runtime_error("Could not enumerate frame sizes for the given format.");
  }
  HIGH_MSG("Auto-selected lowest resolution: %ux%u", width, height);
}

void FrameForger::initializeDevice(uint32_t format, uint32_t fps) {
  struct v4l2_capability capa;
  xioctl(fd, VIDIOC_QUERYCAP, &capa);

  if (!(capa.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    throw std::runtime_error("Device does not support video capture.");
  if (!(capa.capabilities & V4L2_CAP_STREAMING))
    throw std::runtime_error("Device does not support streaming I/O.");

  if (width == 0 || height == 0) {
    findLowestResolution(format);
  }

  // Initialize format structure
  struct v4l2_format fmt  = {};
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = width;
  fmt.fmt.pix.height      = height;
  fmt.fmt.pix.pixelformat = format;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;

  xioctl(fd, VIDIOC_S_FMT, &fmt);

  // The driver is allowed to adjust the format. We must check
  // what was actually set to avoid buffer overflows or misinterpreting data later.
  if (fmt.fmt.pix.pixelformat != format) {
    throw std::runtime_error("Driver did not accept the requested pixel format.");
  }
  // Update our internal width/height in case the driver adjusted them.
  width  = fmt.fmt.pix.width;
  height = fmt.fmt.pix.height;
  HIGH_MSG("Format set to %ux%u", width, height);

  struct v4l2_streamparm parm = {};
  parm.type                   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(fd, VIDIOC_G_PARM, &parm);
  if (parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    xioctl(fd, VIDIOC_S_PARM, &parm);
    HIGH_MSG("FPS set to: %u", parm.parm.capture.timeperframe.denominator);
  } else {
    WARN_MSG("Device does not support setting FPS.");
  }
}

void FrameForger::initializeBuffers(uint8_t reqBufferCount) {
  struct v4l2_requestbuffers buffReq = {};
  buffReq.count                      = reqBufferCount;
  buffReq.type                       = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffReq.memory                     = V4L2_MEMORY_MMAP;

  xioctl(fd, VIDIOC_REQBUFS, &buffReq);

  if (buffReq.count < 2) {
    throw std::runtime_error("Insufficient buffer memory on device.");
  }

  // Using a std::vector is safer and cleaner than manual calloc/free.
  buffers.resize(buffReq.count);

  for (uint8_t i = 0; i < buffers.size(); ++i) {
    struct v4l2_buffer buff = {};
    buff.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buff.memory             = V4L2_MEMORY_MMAP;
    buff.index              = i;

    xioctl(fd, VIDIOC_QUERYBUF, &buff);

    buffers[i].length = buff.length;
    buffers[i].start =
            mmap(nullptr, buff.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buff.m.offset);

    if (buffers[i].start == MAP_FAILED) {
      throw std::runtime_error("Failed to map buffer to memory.");
    }
  }
  HIGH_MSG("Successfully mapped %zu buffers.", buffers.size());
}

void FrameForger::queueAllBuffers() {
  for (size_t i = 0; i < buffers.size(); ++i) {
    struct v4l2_buffer buff = {};
    buff.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buff.memory             = V4L2_MEMORY_MMAP;
    buff.index              = i;
    xioctl(fd, VIDIOC_QBUF, &buff);
  }
  HIGH_MSG("Successfully queued %zu buffers.", buffers.size());
}