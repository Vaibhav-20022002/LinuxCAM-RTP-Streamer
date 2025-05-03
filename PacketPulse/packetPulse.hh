#pragma once

extern "C" {
#include <arpa/inet.h>   // For inet_pton()
#include <fcntl.h>       // For file control options
#include <netinet/in.h>  // For sockaddr_in
#include <sys/socket.h>  // For socket(), sendto()
#include <unistd.h>      // For close()
}

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ringMaster.hh"
#include "turboSnap.hh"

/**
 * @class PacketPulse
 * @brief RTP/MJPEG streaming engine optimized for low-power hardware
 *
 * PacketPulse implements a compliant RTP payload format for MJPEG (RFC 2435),
 * tailored for resource-constrained devices (e.g., Raspberry Pi Zero 2W). It
 * integrates a lock-free SPSC frame buffer, zero-copy packet construction,
 * precise timing control, and SDP generation for client handshake.
 *
 * @section CoreFeatures
 * - Single-producer/single-consumer (SPSC) frame buffer via RingMaster.
 * - Pre-allocated packet buffer to avoid dynamic allocations on the hot path.
 * - Inline RTP and JPEG header assembly with correct network byte order.
 * - Non-blocking UDP socket with configurable MTU-based fragmentation.
 * - Frame rate enforcement and backlog management (frame drop warnings).
 * - SDP content generation and file output for client configuration.
 *
 * @section Assumptions
 * - FrameType must provide JPEG frame payload via .jpegData
 * (std::vector<uint8_t>) and .frameNumber for sequencing metadata.
 * - Only one thread calls start()/stop(), and one consumer thread runs
 * streamingLoop().
 * - Destination IP is IPv4; destPort is a valid UDP port.
 *
 * @tparam FrameType Type representing a JPEG frame, typically `jpegFrame_`.
 */
template <typename FrameType>
class PacketPulse {
 public:
  /**
   * @name NetworkParameters
   * Constants defining packet sizes and header lengths.
   */
  ///@{
  static constexpr size_t kMtu =
      1400; /**< Default Ethernet MTU to avoid IP fragmentation  */
  static constexpr size_t kRtpHeaderSize =
      12; /**< Fixed RTP header size in bytes                      */
  static constexpr size_t kJpegHeaderSize =
      8; /**< RFC 2435 JPEG payload header size                  */
  static constexpr size_t kMaxPayloadSize =
      kMtu - kRtpHeaderSize -
      kJpegHeaderSize; /**< Max JPEG fragment size per packet */
  ///@}

  /**
   * @brief Construct PacketPulse streamer
   *
   * Configures the RTP/MJPEG streamer with specified network and media
   * parameters, pre-allocates buffers, and initializes network resources.
   *
   * @param frameBuffer  Reference to SPSC ring buffer containing JPEG frames
   * @param destIp       IPv4 destination address (dotted decimal)
   * @param destPort     UDP port for RTP packet transmission
   * @param width        Frame width in pixels (must match FrameType content)
   * @param height       Frame height in pixels (must match FrameType content)
   * @param payloadType  RTP payload type code (dynamic range 96–127)
   * @param ssrc         RTP synchronization source identifier
   * @param clockRate    RTP timestamp clock rate (Hz), e.g., 90000 for video
   * @param framerate    Desired frame rate (frames per second)
   *
   * @throws std::invalid_argument if width, height, or framerate are out of
   * range
   * @throws std::runtime_error if network socket creation or configuration
   * fails
   */
  PacketPulse(RingMaster<FrameType>& frameBuffer, std::string_view destIp,
              uint16_t destPort, uint16_t width, uint16_t height,
              uint8_t quality, uint_fast8_t payloadType = 96,
              uint_fast16_t ssrc = 0, uint_fast16_t clockRate = 90000,
              uint8_t framerate = 10);

  /**
   * @brief Destructor: stops streamer and frees network resources
   *
   * Invokes stop(), waits for thread join, then closes the UDP socket if open.
   */
  ~PacketPulse();

  /**
   * @brief Start RTP streaming thread
   *
   * Launches the internal worker thread executing streamingLoop(). Safe to call
   * multiple times; only the first invocation spawns the thread.
   */
  void start();

  /**
   * @brief Stop RTP streaming thread
   *
   * Signals the worker thread to exit at the next frame boundary and joins it.
   * Can be invoked from any thread context.
   */
  void stop();

  /**
   * @brief Generate SDP description string
   *
   * Builds an SDP (RFC 4566) session description for clients to connect.
   * Includes media and codec attributes for MJPEG over RTP.
   *
   * @param sessionName Name of the streaming session (s=)
   * @param sourceIp    IP address of the streaming source (c=)
   * @param sessionId   Unique session identifier; defaults to current timestamp
   * @return std::string Full SDP content with CRLF line endings
   */
  std::string generateSDP(const std::string& sessionName,
                          const std::string& sourceIp,
                          uint64_t sessionId = 0) const;

  /**
   * @brief Write SDP to file system
   *
   * Creates or overwrites a file at `filePath` containing the SDP generated
   * by generateSDP(). Useful for distribution to playback clients.
   *
   * @param filePath    Destination file path
   * @param sessionName Name of the streaming session
   * @param sourceIp    IP address of the streaming source
   * @param sessionId   Unique session identifier; defaults to timestamp
   * @return true if file was written successfully; false otherwise (and logs
   * error)
   * @throws std::runtime_error if file cannot be opened for writing
   */
  bool writeSDP(std::string_view filePath, std::string_view sessionName,
                std::string_view sourceIp, uint64_t sessionId = 0) const;

 private:
  /**
   * @brief Main loop performing packetization and transmission
   *
   * Executes until stop() is called. Steps per frame:
   * 1. Dequeue next JPEG frame from frameBuffer_
   * 2. Fragment into kMaxPayloadSize slices
   * 3. Build RTP (createRtpHeader) and JPEG (createJpegHeader) headers
   * 4. Send via non-blocking UDP socket
   * 5. Maintain frame timing and drop warnings
   */
  void streamingLoop();

  /**
   * @brief Set up UDP socket and destination addressing
   *
   * Creates a non-blocking UDP socket, adjusts send buffer size, sets
   * SO_REUSEADDR, and initializes destAddr_.
   *
   * @param destIp    IPv4 address string
   * @param destPort  UDP port number
   * @throws std::runtime_error if any network operation fails
   */
  void setupNetwork(std::string_view destIp, uint16_t destPort);

  /**
   * @brief Assemble RTP header in-place
   *
   * Populates the first kRtpHeaderSize bytes of `buffer` with:
   * - RTP version, padding, extension, CC fields
   * - Marker bit and payloadType_
   * - sequenceNum (network order)
   * - timestamp (network order)
   * - ssrc_ (network order)
   *
   * @param[out] buffer     Pointer to packet buffer (>= kRtpHeaderSize)
   * @param markerBit       True on last fragment of a frame
   * @param timestamp       RTP timestamp value
   * @param sequenceNum     Packet sequence number
   */
  inline void createRtpHeader(uint8_t* buffer, bool markerBit,
                              uint32_t timestamp, uint16_t sequenceNum) const;

  /**
   * @brief Assemble JPEG payload header in-place
   *
   * Writes the 8-byte JPEG header after the RTP header, encoding:
   * - type-specific field (JPEG baseline = 0)
   * - fragment offset (3-byte big-endian)
   * - JPEG type and quality fields
   * - width/height in macroblocks (8-pixel units)
   *
   * @param[out] buffer        Pointer to buffer (>= kJpegHeaderSize)
   * @param fragmentOffset     Byte offset into full JPEG frame
   * @param totalSize          Full JPEG payload size in bytes
   */
  inline void createJpegHeader(uint8_t* buffer, uint32_t fragmentOffset,
                               uint32_t totalSize) const;

  // Configuration (immutable after construction)
  RingMaster<FrameType>& frameBuffer_; /**< Frame queue source */
  const uint16_t width_;  /**< Frame width (px)                            */
  const uint16_t height_; /**< Frame height (px)                           */
  const uint8_t payloadType_; /**< RTP dynamic payload type */
  const uint8_t quality_;     /**< JPEG quality in range [0, 100] */
  const uint32_t ssrc_; /**< Synchronization source identifier           */
  const unsigned int clockRate_; /**< RTP clock rate (Hz) */
  const unsigned int framerate_; /**< Target frames per second */
  const uint32_t timestampInc_;  /**< Timestamp increment per frame =
                                    clockRate_/framerate_ */

  // Networking
  int udpSocket_;        /**< UDP socket descriptor                       */
  sockaddr_in destAddr_; /**< Destination address struct                  */
  uint16_t destPort_;    /**< Stored port for SDP output                  */

  // Thread and state
  std::atomic<bool> isActive_; /**< True when streamingLoop_ should run */
  std::thread worker_; /**< Worker thread handle                        */

  // RTP sequencing
  std::atomic<uint16_t> sequenceNum_; /**< Packet sequence counter */
  std::atomic<uint32_t> timestamp_;   /**< RTP timestamp accumulator   */

  // Packet buffer
  alignas(
      64) std::vector<uint8_t> packetBuffer_; /**< Pre-allocated send buffer */
};

template <typename FrameType>
PacketPulse<FrameType>::PacketPulse(RingMaster<FrameType>& frameBuffer,
                                    std::string_view destIp, uint16_t destPort,
                                    uint16_t width, uint16_t height,
                                    uint8_t quality, uint_fast8_t payloadType,
                                    uint_fast16_t ssrc, uint_fast16_t clockRate,
                                    uint8_t framerate)
    : frameBuffer_(frameBuffer),
      width_(width),
      height_(height),
      quality_(quality),
      payloadType_(payloadType),
      ssrc_(ssrc),
      clockRate_(clockRate),
      framerate_(framerate),
      timestampInc_(clockRate / framerate),
      destPort_(destPort),
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
    udpSocket_ = -1;
  }
}

template <typename FrameType>
void PacketPulse<FrameType>::start() {
  if (!isActive_.exchange(true)) {
    worker_ = std::thread(&PacketPulse::streamingLoop, this);

// Set thread priority (if available)
#ifdef __linux__
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(worker_.native_handle(), SCHED_FIFO, &param);
    pthread_attr_destroy(&attr);
#endif
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
std::string PacketPulse<FrameType>::generateSDP(const std::string& sessionName,
                                                const std::string& sourceIp,
                                                uint64_t sessionId) const {
  // Use current time as session ID if not provided
  if (sessionId == 0) {
    sessionId = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
  }

  std::stringstream sdp;

  // SDP version (v=)
  sdp << "v=0\r\n";

  // Session origin (o=)
  // <username> <session-id> <version> <network-type> <address-type> <address>
  sdp << "o=- " << sessionId << " " << sessionId << " IN IP4 " << sourceIp
      << "\r\n";

  // Session name (s=)
  sdp << "s=" << sessionName << "\r\n";

  // Timing (t=)
  sdp << "t=0 0\r\n";

  // Media description (m=)
  // <media> <port> <proto> <fmt>
  sdp << "m=video " << destPort_ << " RTP/AVP "
      << static_cast<int>(payloadType_) << "\r\n";

  // Connection data (c=)
  sdp << "c=IN IP4 " << sourceIp << "\r\n";

  // Attributes (a=)
  sdp << "a=rtpmap:" << static_cast<int>(payloadType_) << " JPEG/" << clockRate_
      << "\r\n";

  // Media attributes
  sdp << "a=framerate:" << framerate_ << "\r\n";
  sdp << "a=tool:PacketPulse RTP Streamer\r\n";
  sdp << "a=recvonly\r\n";

  // JPEG-specific attributes
  sdp << "a=fmtp:" << static_cast<int>(payloadType_) << " width=" << width_
      << ";height=" << height_ << ";quality=" << quality_ << "\r\n";

  return sdp.str();
}

template <typename FrameType>
bool PacketPulse<FrameType>::writeSDP(std::string_view filePath,
                                      std::string_view sessionName,
                                      std::string_view sourceIp,
                                      uint64_t sessionId) const {
  try {
    std::ofstream sdpFile(filePath, std::ios::out | std::ios::trunc);
    if (!sdpFile) {
      throw std::runtime_error("Failed to open SDP file for writing: " +
                               filePath);
    }

    std::string sdpContent = generateSDP(sessionName, sourceIp, sessionId);
    sdpFile << sdpContent;
    sdpFile.close();

    return true;
  } catch (const std::exception& e) {
    std::cerr << "SDP file creation error: " << e.what() << std::endl;
    return false;
  }
}

template <typename FrameType>
void PacketPulse<FrameType>::streamingLoop() {
  using namespace std::chrono;
  const auto frameInterval = microseconds(1000000 / framerate_);
  bool frameDropped = false;

  // Cache loop variables to reduce memory access
  const uint8_t payloadType = payloadType_;
  const uint32_t timestampInc = timestampInc_;
  uint16_t localSeqNum = sequenceNum_.load(std::memory_order_relaxed);
  uint32_t localTimestamp = timestamp_.load(std::memory_order_relaxed);

  while (isActive_.load(std::memory_order_relaxed)) {
    auto frameStart = steady_clock::now();

    FrameType frame;
    if (!frameBuffer_.pop(frame)) {
      // Short sleep to avoid busy-wait on empty buffer
      std::this_thread::sleep_for(microseconds(100));
      continue;
    }

    const uint8_t* jpegData = frame.jpegData.data();
    const size_t totalSize = frame.jpegData.size();
    size_t offset = 0;

    while (offset < totalSize && isActive_.load(std::memory_order_relaxed)) {
      const size_t remaining = totalSize - offset;
      const size_t payloadSize = std::min(remaining, kMaxPayloadSize);
      const bool markerBit = (offset + payloadSize) >= totalSize;

      // Construct headers directly in send buffer
      createRtpHeader(packetBuffer_.data(), markerBit, localTimestamp,
                      localSeqNum);
      createJpegHeader(packetBuffer_.data() + kRtpHeaderSize, offset,
                       totalSize);

      // Copy JPEG fragment
      memcpy(packetBuffer_.data() + kRtpHeaderSize + kJpegHeaderSize,
             jpegData + offset, payloadSize);

      // Non-blocking send
      ssize_t sent = sendto(udpSocket_, packetBuffer_.data(),
                            kRtpHeaderSize + kJpegHeaderSize + payloadSize, 0,
                            reinterpret_cast<const sockaddr*>(&destAddr_),
                            sizeof(destAddr_));

      if (sent < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
        std::cerr << "Send error: " << strerror(errno) << "\n";
      }

      localSeqNum++;
      offset += payloadSize;
    }

    localTimestamp += timestampInc;

    // Update atomic values occasionally to reduce contention
    static int updateCounter = 0;
    if (++updateCounter >= 30) {  // Update every ~1 second at 30fps
      sequenceNum_.store(localSeqNum, std::memory_order_relaxed);
      timestamp_.store(localTimestamp, std::memory_order_relaxed);
      updateCounter = 0;
    }

    // Precise frame rate control
    const auto processingTime = steady_clock::now() - frameStart;

    if (processingTime > frameInterval) {
      // If we're behind schedule, consider dropping frames
      if (!frameDropped) {
        std::cerr << "Warning: Frame processing time ("
                  << duration_cast<microseconds>(processingTime).count()
                  << "µs) exceeds interval (" << frameInterval.count()
                  << "µs)\n";
        frameDropped = true;
      }
    } else {
      frameDropped = false;
      std::this_thread::sleep_for(frameInterval - processingTime);
    }
  }

  // Final update of shared values before exit
  sequenceNum_.store(localSeqNum, std::memory_order_relaxed);
  timestamp_.store(localTimestamp, std::memory_order_relaxed);
}

template <typename FrameType>
void PacketPulse<FrameType>::setupNetwork(std::string_view destIp,
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
  int bufferSize = 2 * 1024 * 1024;  // 2MB for better buffering
  if (setsockopt(udpSocket_, SOL_SOCKET, SO_SNDBUF, &bufferSize,
                 sizeof(bufferSize)) < 0) {
    std::cerr
        << "Warning: Failed to set socket buffer size, using system default\n";
  }

  // Allow socket address reuse
  int reuse = 1;
  if (setsockopt(udpSocket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) <
      0) {
    std::cerr << "Warning: Failed to set SO_REUSEADDR\n";
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
inline void PacketPulse<FrameType>::createRtpHeader(
    uint8_t* buffer, bool markerBit, uint32_t timestamp,
    uint16_t sequenceNum) const {
  // RTP version 2, no padding/extension/CSRCs
  buffer[0] = 0x80;
  // Marker bit and payload type
  buffer[1] = (markerBit ? 0x80 : 0x00) | (payloadType_ & 0x7F);
  // Sequence number (network byte order)
  buffer[2] = (sequenceNum >> 8) & 0xFF;
  buffer[3] = sequenceNum & 0xFF;
  // Timestamp (network byte order)
  buffer[4] = (timestamp >> 24) & 0xFF;
  buffer[5] = (timestamp >> 16) & 0xFF;
  buffer[6] = (timestamp >> 8) & 0xFF;
  buffer[7] = timestamp & 0xFF;
  // SSRC identifier (network byte order)
  buffer[8] = (ssrc_ >> 24) & 0xFF;
  buffer[9] = (ssrc_ >> 16) & 0xFF;
  buffer[10] = (ssrc_ >> 8) & 0xFF;
  buffer[11] = ssrc_ & 0xFF;
}

template <typename FrameType>
inline void PacketPulse<FrameType>::createJpegHeader(uint8_t* buffer,
                                                     uint32_t fragmentOffset,
                                                     uint32_t totalSize) const {
  // Type specific field (0 for baseline JPEG)
  buffer[0] = 0x00;
  // Fragment offset (3 bytes, big-endian)
  buffer[1] = (fragmentOffset >> 16) & 0xFF;
  buffer[2] = (fragmentOffset >> 8) & 0xFF;
  buffer[3] = fragmentOffset & 0xFF;
  // JPEG type (1 = YUV 4:2:0)
  buffer[4] = 0x01;
  // Quality factor (255 = table specified)
  buffer[5] = 0xFF;
  // Width/height in 8-pixel blocks (macroblock units)
  buffer[6] = (width_ + 7) / 8;
  buffer[7] = (height_ + 7) / 8;
}
