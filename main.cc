#include "FrameForger/frameForger.hh"
#include <csignal>
#include <cstdlib>
#include <linux/videodev2.h>
#include <memory>

// Async-signal-safe flag set by the signal handler. Checked in the streaming loop.
volatile sig_atomic_t g_sigint_received = 0;

// A global pointer to the FrameForger object.
// This is only used by main for lifecycle management; the signal handler must not call methods.
std::unique_ptr<FrameForger> frameForger_ptr;

extern "C" void sigHandler(int /*sigNum*/) {
  // Async-signal-safe: only set a sig_atomic_t flag.
  g_sigint_received = 1;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    FAIL_MSG("Usage: %s <device> [width height] [fps] [debug_level]", argv[0]);
    FAIL_MSG("Example: %s /dev/video0 640 480 30", argv[0]);
    FAIL_MSG("Debug levels: 0=NONE, 1=ERROR, 2=FAIL, 3=WARN, 4=INFO, 5=HIGH, 6=DEBUG");
    FAIL_MSG("Omitting width/height/fps will auto-detect the lowest resolution.");
    return EXIT_FAILURE;
  }

  const char *device = argv[1];
  // Default to 0, which signals the FrameForger to auto-detect the lowest resolution.
  uint32_t width  = 0;
  uint32_t height = 0;
  uint32_t fps    = 10; // Default to a low FPS to save CPU
  if (argc >= 4) {
    width  = static_cast<uint32_t>(std::atoi(argv[2]));
    height = static_cast<uint32_t>(std::atoi(argv[3]));
    if (width == 0 || height == 0) {
      FAIL_MSG("Invalid width/height provided");
      return EXIT_FAILURE;
    }
  }
  if (argc >= 5) {
    fps = static_cast<uint32_t>(std::atoi(argv[4]));
    if (fps == 0) {
      FAIL_MSG("Invalid FPS provided.");
      return EXIT_FAILURE;
    }
  }

  // Use MJPEG by default
  const uint32_t pixelFormat = V4L2_PIX_FMT_MJPEG;

  try {
    // Create the FrameForger object and store it in the unique_ptr.
    frameForger_ptr = std::make_unique<FrameForger>(device, width, height, pixelFormat, fps);

    // Install a simple SIGINT handler that sets an async-safe flag.
    struct sigaction sa = {};
    sa.sa_handler       = sigHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);

    // Start streaming and enter the capture loop. The loop will call the
    // provided lambda for every captured frame. The lambda only logs
    // readable metadata; it must not free the frame memory (owned by the driver).
    // Start streaming.
    frameForger_ptr->startStream();

    // Streaming loop will check g_sigint_received and stop gracefully when set.
    frameForger_ptr->streamingLoop([](Frame &&f) {
      long long timestamp_ms = (long long)f.timestamp.tv_sec * 1000 + f.timestamp.tv_usec / 1000;
      DEBUG_MSG("Frame received: size=%zu bytes, timestamp=%lld ms", f.length, timestamp_ms);
      // Application may process the buffer (zero-copy).
      // Do not munmap/free f.start here.
    });

  } catch (const std::exception &e) {
    ERROR_MSG("Fatal error: %s", e.what());
    return EXIT_FAILURE;
  }

  INFO_MSG("Exiting.");
  return EXIT_SUCCESS;
}
