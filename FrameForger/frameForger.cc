#include "./frameForger.hh"

#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

void FrameForger::openDevice(int flags) {
  int ret;
  ret = open(device.data(), flags);
  if (EACCES == ret) {
    DEBUG_MSG("Cannot access device may be permission issues");
    return;
  } else if (EBUSY == ret) {
    DEBUG_MSG("Device already in use.");
    return;
  } else if (ret < 0) {
    DEBUG_MSG("Unknown error. Reason : %s", strerror(errno));
  }
  FrameForger::fd = ret;
}

FrameForger::FrameForger(char *devicePath, int flags) {
  device = devicePath;
  openDevice(flags);
}

FrameForger::~FrameForger() {
  free(buffPtrs);
  close(fd);
}

std::string FrameForger::getDevice() {
  return device;
}

int FrameForger::getFD() {
  return fd;
}

bool FrameForger::queryCaptureCapa() {
  xioctl(fd, VIDIOC_QUERYCAP, &capa);
  return capa.capabilities & V4L2_CAP_VIDEO_CAPTURE;
}

bool FrameForger::queryStreamCapa() {
  xioctl(fd, VIDIOC_QUERYCAP, &capa);
  return capa.capabilities & V4L2_CAP_STREAMING;
}

void FrameForger::queryCapabilities() {
  isCaptureSupported = queryCaptureCapa();
  isStreamSupported  = queryStreamCapa();
}

bool FrameForger::setFormat(std::string format, uint32_t wd, uint32_t ht) {
  ///\todo Before making VIDIOC_S_FMT request, check format using VIDIOC_G_FMT
  // Check the supported list
  if (format != "MJPEG") {
    DEBUG_MSG("Given format (%s) not supported.", format.data());
    return false;
  }

  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = wd;
  fmt.fmt.pix.height = ht;

  // If format is MJPEG
  if (format == "MJPEG") {
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    pixelFormat = V4L2_PIX_FMT_MJPEG;
    pixelField = V4L2_FIELD_ANY;
    pixelFormatStr = format;
  }

  xioctl(fd, VIDIOC_S_FMT, &fmt);

  ///\note The Driver may not have accepted the format exactly. Check what was actually set.
  if (fmt.fmt.pix.pixelformat != pixelFormat) {
    FAIL_MSG("Driver didn't accept %s (%dx%d) format. Is it supported?", format.data(), wd, ht);
    return false;
  }

  width = fmt.fmt.pix.width;
  height = fmt.fmt.pix.height;

  return true;
}

uint8_t FrameForger::allocateBuffers(int reqBufferCount) {
  if (reqBufferCount < 2) {
    WARN_MSG("Requested Buffer count is less than 2. Trying to allocate 4...");
    reqBufferCount = 4;
  }

  memset(&buffReq, 0, sizeof(buffReq));
  buffReq.count = reqBufferCount;
  buffReq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffReq.memory = V4L2_MEMORY_MMAP;

  xioctl(fd, VIDIOC_REQBUFS, &buffReq);

  ///\note The Driver might allocate fewer buffers than requested (may be due to memory constraints)
  if (buffReq.count < 2) {
    FAIL_MSG("Cannot allocate even 2 buffers. May be unsufficient memory.");
    return 0;
  }

  noOfBuffers = buffReq.count;

  return noOfBuffers;
}

bool FrameForger::mapBuffers() {
  // Calloc is used as zeros initialization of the memory is must
  buffPtrs = static_cast<void**>(calloc(noOfBuffers, sizeof(*buffPtrs)));

  for (uint8_t i = 0; i < noOfBuffers; ++i) {
    struct v4l2_buffer buff;
    memset(&buff, 0, sizeof(buff));

    buff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buff.memory = V4L2_MEMORY_MMAP;
    buff.index = i;

    xioctl(fd, VIDIOC_QUERYBUF, &buff);

    buffPtrs[i] = mmap(
      NULL,                   // Let the kernel chose the address
      buff.length,            // Length of buffer
      PROT_READ | PROT_WRITE, // Read & Write to be performed on memory
      MAP_SHARED,             // Shared mapping
      fd,                     // Device file descriptor
      buff.m.offset           // Offset to the buffer in device memory
    );

    if (MAP_FAILED == buffPtrs[i]) {
      FAIL_MSG("Cannot map buffer at index %d. Exiting...", i);
      return false;
    }
  }
  HIGH_MSG("Successfully mapped %d buffers", noOfBuffers);
  return true;
}

void FrameForger::queueBuffers() {
  for (uint8_t i = 0; i < noOfBuffers; ++i) {
    struct v4l2_buffer buff;
    memset(&buff, 0, sizeof(buff));

    buff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buff.memory = V4L2_MEMORY_MMAP;
    buff.index = i;

    xioctl(fd, VIDIOC_QBUF, &buff);
  }
  HIGH_MSG("Successfully queued %d buffers", noOfBuffers);
}
