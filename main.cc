#include "FrameForger/frameForger.hh"
#include <fcntl.h>
#include <cstdlib>

int main(int argc, char *argv[]) {
  if (argc < 2) {
    return EXIT_FAILURE;
  }

  FrameForger frameForger(argv[1], O_RDWR | O_NONBLOCK);

  frameForger.queryCapabilities();

  if (frameForger.isCaptureSupported) {
    INFO_MSG("Device %s supports CAPTURE.", frameForger.getDevice().data());
  } else {
    FAIL_MSG("Device %s doesn't supports CAPTURE.", frameForger.getDevice().data());
    return EXIT_FAILURE;
  }

  if (frameForger.isStreamSupported) {
    INFO_MSG("Device %s supports STREAMING.", frameForger.getDevice().data());
  } else {
    FAIL_MSG("Device %s doesn't supports STREAMING.", frameForger.getDevice().data());
    return EXIT_FAILURE;
  }

  INFO_MSG("Device %s opened successfully.", frameForger.getDevice().data());

  if (frameForger.setFormat("MJPEG", 1280, 720)) {
    INFO_MSG("Successfully configured device with configuration = %s (%dx%d)", frameForger.pixelFormatStr.data(),
              frameForger.width, frameForger.height);
  } else {
    exit(EXIT_FAILURE);
  }

  if (frameForger.allocateBuffers() == 0) {
    FAIL_MSG("Cannot allocate requested buffers. Exiting...");
    exit(EXIT_FAILURE);
  } else {
    HIGH_MSG("Successfully allocated %u buffers", frameForger.noOfBuffers);
  }

  if (!frameForger.mapBuffers()) {
    exit(EXIT_FAILURE);
  }

  frameForger.queueBuffers();

  return EXIT_SUCCESS;
}
