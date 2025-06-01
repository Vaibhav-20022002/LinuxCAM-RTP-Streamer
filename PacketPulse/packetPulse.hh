#pragma once

extern "C" {
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>
}

#include "frameTypes.hh"
#include "ringMaster.hh"
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

/**
 * @class PacketPulse
 * @brief High-Performance RTP/MJPEG streamer optimized for low-power consumption
 *
 * @details This class implements an efficient RTP/MJPEG streaming pipeline using :
 * - Scatter/Gather I/O to avoid data copying
 * - Cache-aligned data strucutures
 * - Frame Timestamp synchronization
 * - Non-blocking UDP with large send buffers
 * - Frame blocking management
 *
 * @tparam FrameType Frame Type (must be MjpegFrame)
 * @see frameTypes.hh file
 */
template<typename FrameType> class PacketPulse {
public:
  // **---- Network Constants ----**
  static constexpr size_t kMtu           = 1400; ///< Default Ethernet MTU to avoid IP Fragmentation
  static constexpr size_t kRtpHeaderSize = 12;   ///< Fixed RTP header size in bytes
  static constexpr size_t kJpegHeaderSize = 8;   ///< RFC 2435 JPEG payload header size
  static constexpr size_t KMaxPayloadSize =
      kMtu - kRtpHeaderSize - kJpegHeaderSize; ///< Max JPEG fragment size per packet

  // **---- Functions -----**
  /**
   * @brief Constructor
   *
   * @param framebuffer Frame source buffer
   * @param destIP Destination IP address
   * @param destPort Destination port
   * @param width Frame width
   * @param height Frame height
   * @param quaility JPEG quality
   * @param payloadType RTP payload type
   * @param ssrc Synchronization Source
   * @param clockRate Timestamp clock rate
   * @param framerate Target frameRate
   */
  PacketPulse(RingMaster<FrameType> &frameBuffer,
      const std::string             &destIP,
      uint16_t                       destPort,
      uint16_t                       width,
      uint16_t                       height,
      uint8_t                        quality     = 50,
      uint8_t                        payloadType = 96,
      uint32_t                       ssrc        = 0,
      uint32_t                       clockRate   = 90'000,
      uint8_t                        framerate   = 8);

  ~PacketPulse();

  void start();
  void stop();

  std::string generateSDP(const std::string &sessionName,
      const std::string                     &sourceIP,
      uint64_t                               sessionID = 0) const;

  bool writeSDP(const std::string &filePath,
      const std::string           &sessionName,
      const std::string           &sourceIP,
      uint64_t                     sessionID = 0) const;

private:
  void streamingLoop();
  void setupNetwork(const std::string &destIP, uint16_t destPort);
  inline void
  createRtpHeader(uint8_t *buffer, bool markerBit, uint32_t timestamp, uint16_t seqNum) const;
  inline void createJpegHeader(uint8_t *buffer, uint32_t fragmentOffset, uint32_t totalSize) const;

  // **---- Configuration ----**
  RingMaster<FrameType> &frameBuffer_;
  const uint16_t         width_;
  const uint16_t         height_;
  const uint8_t          quality_;
  const uint8_t          payloadType_;
  const uint32_t         ssrc_;
  const uint32_t         clockRate_;
  const uint8_t          framerate_;
  const uint32_t         timestampInc_;

  // **---- Networking ----**
  int         udpSocket_ = -1;
  sockaddr_in destAddr_{};
  uint16_t    destPort_;

  // **---- Thread Management ----**
  std::atomic<bool> isActive_{false};
  std::thread       worker_;

  // **---- RTP state ----**
  std::atomic<uint16_t> seqNum_{0};
  std::atomic<uint32_t> timestamp_{0};

  // **---- Header buffer (cache aligned) ----**
  alignas(kCacheAlign) std::array<uint8_t, kRtpHeaderSize + kJpegHeaderSize> headerBuffer_;
};

//  *-------------------------------------------------* //
//  *   Implementation below (template definitions)   * //
//  *-------------------------------------------------* //

template<typename FrameType>
inline PacketPulse<FrameType>::PacketPulse(RingMaster<FrameType> &frameBuffer,
    const std::string                                            &destIP,
    uint16_t                                                      destPort,
    uint16_t                                                      width,
    uint16_t                                                      height,
    uint8_t                                                       quality,
    uint8_t                                                       payloadType,
    uint32_t                                                      ssrc,
    uint32_t                                                      clockRate,
    uint8_t                                                       framerate)
    : frameBuffer_(frameBuffer)
    , width_(width)
    , height_(height)
    , quality_(quality)
    , payloadType_(payloadType)
    , ssrc_(ssrc)
    , clockRate_(clockRate)
    , framerate_(framerate)
    , timestampInc_(clockRate / framerate)
    , destPort_(destPort) {
  if (width == 0 || height == 0) {
    throw std::invalid_argument("Invalid frame dimensions");
  }

  if (framerate == 0 && framerate > 60) {
    throw std::invalid_argument("Invalid frame rate");
  }

  setupNetwork(destIP, destPort);
}

template<typename FrameType> PacketPulse<FrameType>::~PacketPulse() {
  stop();
  if (udpSocket_ >= 0) {
    close(udpSocket_);
  }
}

template<typename FrameType> void PacketPulse<FrameType>::start() {
  if (!isActive_.exchange(true)) {
    worker_ = std::thread(&PacketPulse::streamingLoop, this);

// Set real-time priority if available
#ifdef __linux__
    sched_param sch{};
    sch.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_setschedparam(worker_.native_handle(), SCHED_FIFO, &sch)) {
      std::cerr << "Warning: Failed to set thread priority\n";
    }
#endif
  }
}

template<typename FrameType> void PacketPulse<FrameType>::stop() {
  if (isActive_.exchange(false) && worker_.joinable()) {
    worker_.join();
  }
}

template<typename FrameType> void PacketPulse<FrameType>::streamingLoop() {
  using namespace std::chrono;
  const auto   frameInterval = microseconds(1000000 / framerate_);
  uint32_t     backlogCount  = 0;
  const size_t maxBacklog    = framerate_ * 2; // 2 seconds of frames

  // Local state to reduce atomic access
  uint16_t localSeqNum    = sequenceNum_.load(std::memory_order_relaxed);
  uint32_t localTimestamp = timestamp_.load(std::memory_order_relaxed);

  while (isActive_.load(std::memory_order_relaxed)) {
    auto frameStart = steady_clock::now();

    // Handle frame backlog
    size_t bufferSize = frameBuffer_.size();
    if (bufferSize > maxBacklog) {
      backlogCount = frameBuffer_.remove(bufferSize - maxBacklog);
      if (backlogCount > 0) {
        std::cerr << "Dropped " << backlogCount << " frames (backlog)\n";
      }
    }

    FrameType frame;
    if (!frameBuffer_.pop(frame)) {
      std::this_thread::sleep_for(100us);
      continue;
    }

    const uint8_t *jpegData  = frame.jpegData.data();
    const size_t   totalSize = frame.jpegData.size();
    size_t         offset    = 0;

    while (offset < totalSize && isActive_.load(std::memory_order_relaxed)) {
      const size_t remaining   = totalSize - offset;
      const size_t payloadSize = std::min(remaining, kMaxPayloadSize);
      const bool   markerBit   = (offset + payloadSize) >= totalSize;

      // Prepare headers
      createRtpHeader(headerBuffer_.data(), markerBit, localTimestamp, localSeqNum);
      createJpegHeader(headerBuffer_.data() + kRtpHeaderSize, offset, totalSize);

      // Scatter/gather I/O
      iovec iov[2] = {{headerBuffer_.data(), kRtpHeaderSize + kJpegHeaderSize},
          {const_cast<uint8_t *>(jpegData + offset), payloadSize}};

      msghdr msg      = {};
      msg.msg_name    = &destAddr_;
      msg.msg_namelen = sizeof(destAddr_);
      msg.msg_iov     = iov;
      msg.msg_iovlen  = 2;

      ssize_t sent = sendmsg(udpSocket_, &msg, 0);
      if (sent < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
        std::cerr << "Send error: " << strerror(errno) << "\n";
      }

      localSeqNum++;
      offset += payloadSize;
    }

    // Use frame's timestamp if available
    if (frame.timestamp.count() > 0) {
      localTimestamp = static_cast<uint32_t>(
          duration_cast<microseconds>(frame.timestamp).count() * clockRate_ / 1000000);
    } else {
      localTimestamp += timestampInc_;
    }

    // Periodic state sync
    if (localSeqNum % 32 == 0) {
      sequenceNum_.store(localSeqNum, std::memory_order_relaxed);
      timestamp_.store(localTimestamp, std::memory_order_relaxed);
    }

    // Frame rate control
    auto elapsed = duration_cast<microseconds>(steady_clock::now() - frameStart);
    if (elapsed < frameInterval) {
      std::this_thread::sleep_for(frameInterval - elapsed);
    } else if (elapsed > frameInterval * 1.5) {
      std::cerr << "Frame processing slow: " << elapsed.count() << "μs > " << frameInterval.count()
                << "μs\n";
    }
  }

  // Final state update
  sequenceNum_.store(localSeqNum, std::memory_order_relaxed);
  timestamp_.store(localTimestamp, std::memory_order_relaxed);
}

template<typename FrameType>
void PacketPulse<FrameType>::setupNetwork(const std::string &destIp, uint16_t destPort) {
  udpSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (udpSocket_ < 0) {
    throw std::runtime_error("socket() failed: " + std::string(strerror(errno)));
  }

  // Non-blocking mode
  int flags = fcntl(udpSocket_, F_GETFL, 0);
  if (fcntl(udpSocket_, F_SETFL, flags | O_NONBLOCK) < 0) {
    close(udpSocket_);
    throw std::runtime_error("fcntl() failed: " + std::string(strerror(errno)));
  }

  // Large send buffer
  int bufSize = 2 * 1024 * 1024; // 2MB
  if (setsockopt(udpSocket_, SOL_SOCKET, SO_SNDBUF, &bufSize, sizeof(bufSize)) < 0) {
    std::cerr << "Warning: Failed to set socket buffer size\n";
  }

  // Reuse address
  int reuse = 1;
  if (setsockopt(udpSocket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    std::cerr << "Warning: Failed to set SO_REUSEADDR\n";
  }

  // Configure destination
  memset(&destAddr_, 0, sizeof(destAddr_));
  destAddr_.sin_family = AF_INET;
  destAddr_.sin_port   = htons(destPort);
  if (inet_pton(AF_INET, destIp.c_str(), &destAddr_.sin_addr) <= 0) {
    close(udpSocket_);
    throw std::runtime_error("inet_pton() failed for: " + destIp);
  }
}

template<typename FrameType>
inline void PacketPulse<FrameType>::createRtpHeader(uint8_t *buffer,
    bool                                                     markerBit,
    uint32_t                                                 timestamp,
    uint16_t                                                 sequenceNum) const {
  buffer[0]  = 0x80; // Version 2, no extensions
  buffer[1]  = (markerBit ? 0x80 : 0x00) | (payloadType_ & 0x7F);
  buffer[2]  = (sequenceNum >> 8) & 0xFF;
  buffer[3]  = sequenceNum & 0xFF;
  buffer[4]  = (timestamp >> 24) & 0xFF;
  buffer[5]  = (timestamp >> 16) & 0xFF;
  buffer[6]  = (timestamp >> 8) & 0xFF;
  buffer[7]  = timestamp & 0xFF;
  buffer[8]  = (ssrc_ >> 24) & 0xFF;
  buffer[9]  = (ssrc_ >> 16) & 0xFF;
  buffer[10] = (ssrc_ >> 8) & 0xFF;
  buffer[11] = ssrc_ & 0xFF;
}

template<typename FrameType>
inline void PacketPulse<FrameType>::createJpegHeader(uint8_t *buffer,
    uint32_t                                                  fragmentOffset,
    uint32_t                                                  totalSize) const {
  buffer[0] = 0x00; // Type specific
  buffer[1] = (fragmentOffset >> 16) & 0xFF;
  buffer[2] = (fragmentOffset >> 8) & 0xFF;
  buffer[3] = fragmentOffset & 0xFF;
  buffer[4] = 0x01; // JPEG type (4:2:0)
  buffer[5] = quality_;
  buffer[6] = (width_ + 7) / 8;  // Width in macroblocks
  buffer[7] = (height_ + 7) / 8; // Height in macroblocks
}

template<typename FrameType>
std::string PacketPulse<FrameType>::generateSDP(const std::string &sessionName,
    const std::string                                             &sourceIp,
    uint64_t                                                       sessionId) const {
  // Use current time as session ID if not provided
  if (sessionId == 0) {
    sessionId = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch())
            .count());
  }

  std::stringstream sdp;

  // SDP version (v=)
  sdp << "v=0\r\n";

  // Session origin (o=)
  // <username> <session-id> <version> <network-type> <address-type> <address>
  sdp << "o=- " << sessionId << " " << sessionId << " IN IP4 " << sourceIp << "\r\n";

  // Session name (s=)
  sdp << "s=" << sessionName << "\r\n";

  // Timing (t=)
  sdp << "t=0 0\r\n";

  // Media description (m=)
  // <media> <port> <proto> <fmt>
  sdp << "m=video " << destPort_ << " RTP/AVP " << static_cast<int>(payloadType_) << "\r\n";

  // Connection data (c=)
  sdp << "c=IN IP4 " << sourceIp << "\r\n";

  // Attributes (a=)
  sdp << "a=rtpmap:" << static_cast<int>(payloadType_) << " JPEG/" << clockRate_ << "\r\n";

  // Media attributes
  sdp << "a=framerate:" << framerate_ << "\r\n";
  sdp << "a=tool:PacketPulse RTP Streamer\r\n";
  sdp << "a=recvonly\r\n";

  // JPEG-specific attributes
  sdp << "a=fmtp:" << static_cast<int>(payloadType_) << " width=" << width_ << ";height=" << height_
      << ";quality=" << quality_ << "\r\n";

  return sdp.str();
}

template<typename FrameType>
bool PacketPulse<FrameType>::writeSDP(const std::string &filePath,
    const std::string                                   &sessionName,
    const std::string                                   &sourceIp,
    uint64_t                                             sessionId) const {
  try {
    std::ofstream sdpFile(filePath, std::ios::out | std::ios::trunc);
    if (!sdpFile) {
      throw std::runtime_error("Failed to open SDP file for writing: " + filePath);
    }

    std::string sdpContent = generateSDP(sessionName, sourceIp, sessionId);
    sdpFile << sdpContent;
    sdpFile.close();

    return true;
  } catch (const std::exception &e) {
    std::cerr << "SDP file creation error: " << e.what() << std::endl;
    return false;
  }
}
