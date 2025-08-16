#include "frameForger.hh"
#include <csignal>
#include <cstdlib>
#include <linux/videodev2.h>

// Signal flag set by handler (async-signal-safe)
volatile sig_atomic_t g_sigint_received = 0;

extern "C" void sigHandler(int /*sigNum*/) {
  g_sigint_received = 1;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    FAIL_MSG("Usage: %s <device> width height", argv[0]);
    return EXIT_FAILURE;
  }

  const char *device = argv[1];
  uint32_t    width  = 1280;
  uint32_t    height = 720;
  if (argc >= 4) {
    width  = static_cast<uint32_t>(std::atoi(argv[2]));
    height = static_cast<uint32_t>(std::atoi(argv[3]));
    if (width == 0 || height == 0) {
      FAIL_MSG("Invalid width/height provided");
      return EXIT_FAILURE;
    }
  }

  // Use MJPEG by default
  const uint32_t pixelFormat = V4L2_PIX_FMT_MJPEG;

  try {
    FrameForger ff(device, width, height, pixelFormat);

    // Simple SIGINT handler that sets an async-safe flag.
    struct sigaction sa = {};
    sa.sa_handler       = sigHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);

    // Start streaming and enter the capture loop. The loop will call the
    // provided lambda for every captured frame. The lambda only logs
    // readable metadata; it must not free the frame memory (owned by the driver).
    ff.startStream();

    // Passing the global flag into the lambda by reference through an extern
    // variable declared in frameForger.cc (the loop checks it and stops).
    ff.streamingLoop([](Frame &&f) {
      INFO_MSG("Frame received: size=%zu bytes, timestamp=%ld.%06ld, ptr=%p",
          f.length,
          (long)f.timestamp.tv_sec,
          (long)f.timestamp.tv_usec,
          f.start);
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
