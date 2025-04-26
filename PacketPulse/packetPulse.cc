#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "packetPulse.hh"

template <typename FrameType>
PacketPulse<FrameType>::PacketPulse(RingMaster<FrameType>& frameBuffer,
                                    const std::string& destIp,
                                    uint16_t destPort, uint32_t width,
                                    uint32_t height, uint8_t payloadType,
                                    uint32_t ssrc, unsigned int clockRate,
                                    unsigned int framerate)
    : frameBuffer_(frameBuffer),
      width_(width),
      height_(height),
      payloadType_(payloadType),
      ssrc_(ssrc),
      clockRate_(clockRate),
      framerate_(framerate),
      timestampInc_(clockRate / framerate),
      isActive_(false),
      sequenceNum_(0),
      timestamp_(0),
      packetBuffer_(kMtu) {
  // Validate input parameters
  if (width == 0 || height == 0) {
    throw std::invalid_argument("Invalid frame dimensions");
  }
  if (framerate == 0 || framerate > 120) {
    throw std::invalid_argument("Invalid frame rate");
  }

  setupNetwork(destIp, destPort);
}

template <typename FrameType>
PacketPulse<FrameType>::~PacketPulse() {
  stop();
  if (udpSocket_ >= 0) {
    close(udpSocket_);
  }
}

template <typename FrameType>
void PacketPulse<FrameType>::start() {
  if (!isActive_.exchange(true)) {
    worker_ = std::thread(&PacketPulse::streamingLoop, this);
  }
}

template <typename FrameType>
void PacketPulse<FrameType>::stop() {
  if (isActive_.exchange(false)) {
    if (worker_.joinable()) {
      worker_.join();
    }
  }
}

template <typename FrameType>
void PacketPulse<FrameType>::streamingLoop() {
  using namespace std::chrono;
  const auto frameInterval = milliseconds(1000 / framerate_);

  while (isActive_.load(std::memory_order_relaxed)) {
    auto frameStart = steady_clock::now();

    FrameType frame;
    if (!frameBuffer_.pop(frame)) {
      std::this_thread::sleep_for(microseconds(100));
      continue;
    }

    const uint8_t* jpegData = frame.jpegData.data();
    const size_t totalSize = frame.jpegData.size();
    size_t offset = 0;

    while (offset < totalSize && isActive_) {
      const size_t remaining = totalSize - offset;
      const size_t payloadSize = std::min(remaining, kMaxPayloadSize);
      const bool markerBit = (offset + payloadSize) >= totalSize;

      // Construct headers directly in send buffer
      createRtpHeader(packetBuffer_.data(), markerBit, timestamp_.load(),
                      sequenceNum_.load());

      createJpegHeader(packetBuffer_.data() + kRtpHeaderSize, offset,
                       totalSize);

      // Copy JPEG fragment
      memcpy(packetBuffer_.data() + kRtpHeaderSize + kJpegHeaderSize,
             jpegData + offset, payloadSize);

      // Non-blocking send
      sendto(udpSocket_, packetBuffer_.data(),
             kRtpHeaderSize + kJpegHeaderSize + payloadSize, 0,
             reinterpret_cast<sockaddr*>(&destAddr_), sizeof(destAddr_));

      sequenceNum_++;
      offset += payloadSize;
    }

    timestamp_ += timestampInc_;

    // Precise frame rate control
    const auto elapsed = steady_clock::now() - frameStart;
    if (elapsed < frameInterval) {
      std::this_thread::sleep_for(frameInterval - elapsed);
    }
  }
}

template <typename FrameType>
void PacketPulse<FrameType>::setupNetwork(const std::string& destIp,
                                          uint16_t destPort) {
  // Create UDP socket
  udpSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (udpSocket_ < 0) {
    throw std::runtime_error("socket() failed: " +
                             std::string(strerror(errno)));
  }

  // Configure non-blocking I/O
  int flags = fcntl(udpSocket_, F_GETFL, 0);
  if (fcntl(udpSocket_, F_SETFL, flags | O_NONBLOCK) < 0) {
    close(udpSocket_);
    throw std::runtime_error("fcntl() failed: " + std::string(strerror(errno)));
  }

  // Set large send buffer
  int bufferSize = 1 * 1024 * 1024;  // 1MB
  if (setsockopt(udpSocket_, SOL_SOCKET, SO_SNDBUF, &bufferSize,
                 sizeof(bufferSize)) < 0) {
    close(udpSocket_);
    throw std::runtime_error("setsockopt() failed: " +
                             std::string(strerror(errno)));
  }

  // Configure destination address
  memset(&destAddr_, 0, sizeof(destAddr_));
  destAddr_.sin_family = AF_INET;
  destAddr_.sin_port = htons(destPort);
  if (inet_pton(AF_INET, destIp.c_str(), &destAddr_.sin_addr) <= 0) {
    close(udpSocket_);
    throw std::runtime_error("inet_pton() failed for address: " + destIp);
  }
}

template <typename FrameType>
void PacketPulse<FrameType>::createRtpHeader(uint8_t* buffer, bool markerBit,
                                             uint32_t timestamp,
                                             uint16_t sequenceNum) const {
  // RTP version 2, no padding/extension/CSRCs
  buffer[0] = 0x80;
  // Marker bit and payload type
  buffer[1] = (markerBit ? 0x80 : 0x00) | (payloadType_ & 0x7F);
  // Sequence number (network byte order)
  *reinterpret_cast<uint16_t*>(buffer + 2) = htons(sequenceNum);
  // Timestamp (network byte order)
  *reinterpret_cast<uint32_t*>(buffer + 4) = htonl(timestamp);
  // SSRC identifier (network byte order)
  *reinterpret_cast<uint32_t*>(buffer + 8) = htonl(ssrc_);
}

template <typename FrameType>
void PacketPulse<FrameType>::createJpegHeader(uint8_t* buffer,
                                              uint32_t fragmentOffset,
                                              uint32_t totalSize) const {
  // Type specific field (0 for baseline JPEG)
  buffer[0] = 0x00;
  // Fragment offset (3 bytes, big-endian)
  buffer[1] = (fragmentOffset >> 16) & 0xFF;
  buffer[2] = (fragmentOffset >> 8) & 0xFF;
  buffer[3] = fragmentOffset & 0xFF;
  // JPEG type (1 = 420)
  buffer[4] = 0x01;
  // Quality factor (255 = table specified)
  buffer[5] = 0xFF;
  // Width/height in 8-pixel blocks
  buffer[6] = (width_ + 7) / 8;
  buffer[7] = (height_ + 7) / 8;
}
