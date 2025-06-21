#include "FrameForger_MJPEG.hh"

#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sys/mman.h>
#include <thread>

FrameForger_MJPEG::FrameForger_MJPEG(RingMaster<MjpegFrame> &mjpegBuffer,
    const CameraConfig                                      &config)
    : cameraManager_(nullptr)
    , bufferAllocator_(nullptr)
    , rgbBuffer_(mjpegBuffer)
    , config_(config)
    , initialized_(false) {
  // Use pre-configured camera
  camera_         = config.camera;
  cameraAcquired_ = true;
}

FrameForger_MJPEG::~FrameForger_MJPEG() {
  stopCapture();
  cleanup();
}

bool FrameForger_MJPEG::getNextFrame(MjpegFrame &frame) {
  while (capturing_.load()) {
    if (rgbBuffer_.pop(frame)) return true;
    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  return false;
}

void FrameForger_MJPEG::requestComplete(libcamera::Request *request) {
  if (request->status() == libcamera::Request::RequestCancelled) return;

  const libcamera::Request::BufferMap          &bufs = request->buffers();
  libcamera::Request::BufferMap::const_iterator it   = bufs.find(stream_);
  if (it == bufs.end()) return;
  libcamera::FrameBuffer *fb = it->second;

  MjpegFrame frame;
  frame.jpegData.resize(frameSize_);
  size_t copied = 0;
  for (const libcamera::FrameBuffer::Plane &plane : fb->planes()) {
    int  fd    = plane.fd.get();
    auto mapIt = fdMappings_.find(fd);
    if (mapIt == fdMappings_.end()) continue;
    uint8_t *src    = static_cast<uint8_t *>(mapIt->second.first) + plane.offset;
    size_t   toCopy = std::min<size_t>(plane.length, frameSize_ - copied);
    std::memcpy(frame.jpegData.data() + copied, src, toCopy);
    copied += toCopy;
  }

  auto ts = request->metadata().get(libcamera::controls::SensorTimestamp);
  if (ts) {
    frame.timestamp = std::chrono::microseconds(*ts);
  } else {
    frame.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch());
  }
  frame.width          = width_;
  frame.height         = height_;
  frame.sequenceNumber = seqCounter_++;

  while (!rgbBuffer_.push(std::move(frame))) {
    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }

  request->reuse();
  if (request->addBuffer(stream_, fb) < 0) return;
  camera_->queueRequest(request);
}

void FrameForger_MJPEG::requestCallback(libcamera::Request *request, void *user_data) {
  static_cast<FrameForger_MJPEG *>(user_data)->requestComplete(request);
}

int FrameForger_MJPEG::initialize() {
  if (initialized_) return SUCCESS;
  int ret = configureStream();
  if (ret != SUCCESS) return ret;
  ret = allocateBuffers();
  if (ret != SUCCESS) return ret;
  ret = createRequests();
  if (ret != SUCCESS) return ret;
  initialized_ = true;
  return SUCCESS;
}

int FrameForger_MJPEG::configureStream() {
  cameraConfig_ = camera_->generateConfiguration({libcamera::StreamRole::VideoRecording});
  if (!cameraConfig_ || cameraConfig_->empty()) {
    return ErrorCode::ERR_CONFIG_GENERATION;
  }

  libcamera::StreamConfiguration &streamConfig = cameraConfig_->at(0);
  streamConfig.pixelFormat                     = config_.format;
  streamConfig.size                            = libcamera::Size(config_.width, config_.height);
  width_                                       = config_.width;
  height_                                      = config_.height;

  if (cameraConfig_->validate() == libcamera::CameraConfiguration::Invalid) {
    return ErrorCode::ERR_CONFIG_VALIDATION;
  }

  if (camera_->configure(cameraConfig_.get())) {
    return ErrorCode::ERR_CONFIG_APPLICATION;
  }

  stream_    = streamConfig.stream();
  frameSize_ = streamConfig.frameSize;
  return ErrorCode::SUCCESS;
}

int FrameForger_MJPEG::allocateBuffers() {
  bufferAllocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
  if (bufferAllocator_->allocate(stream_) < 0) {
    return ErrorCode::ERR_BUFFER_ALLOCATION;
  }

  std::unordered_map<int, size_t> maxLens;
  for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : bufferAllocator_->buffers(stream_)) {
    for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
      int    fd     = plane.fd.get();
      size_t reqLen = plane.offset + plane.length;
      if (reqLen > maxLens[fd]) {
        maxLens[fd] = reqLen;
      }
    }
  }

  for (const auto &[fd, len] : maxLens) {
    void *addr = mmap(nullptr, len, PROT_READ, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
      for (const auto &[mapped_fd, mapping] : fdMappings_) {
        munmap(mapping.first, mapping.second);
      }
      fdMappings_.clear();
      return ErrorCode::ERR_BUFFER_ALLOCATION;
    }
    fdMappings_[fd] = {addr, len};
  }

  buffersAllocated_ = bufferAllocator_->buffers(stream_).size();
  return ErrorCode::SUCCESS;
}

int FrameForger_MJPEG::createRequests() {
  for (const auto &buffer : bufferAllocator_->buffers(stream_)) {
    RequestPtr request = camera_->createRequest();
    if (request->addBuffer(stream_, buffer.get()) < 0) {
      return ErrorCode::ERR_REQUEST_BUFFER;
    }
    requests_.push_back(std::move(request));
  }
  return ErrorCode::SUCCESS;
}

int FrameForger_MJPEG::startCapture() {
  if (capturing_.load()) return SUCCESS;
  camera_->requestCompleted.connect(this, &FrameForger_MJPEG::requestComplete);
  if (camera_->start()) return ErrorCode::ERR_CAMERA_START;
  for (RequestPtr &req : requests_) {
    if (camera_->queueRequest(req.get()) < 0) return ErrorCode::ERR_QUEUE_REQUEST;
  }
  capturing_.store(true);
  return SUCCESS;
}

void FrameForger_MJPEG::stopCapture() {
  if (!capturing_.exchange(false)) return;
  camera_->stop();
  requests_.clear();
}

void FrameForger_MJPEG::cleanup() {
  for (auto &[fd, mapping] : fdMappings_) {
    munmap(mapping.first, mapping.second);
  }
  fdMappings_.clear();
  releaseCamera();
}

void FrameForger_MJPEG::releaseCamera() {
  if (cameraAcquired_) {
    camera_->release();
    cameraAcquired_ = false;
  }
}