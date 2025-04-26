#pragma once

#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "ringMaster.hh"
#include "turboSnap.hh"

/**
 * @class PacketPulse
 * @brief High-efficiency RTP/MJPEG streamer optimized for low-power devices
 *
 * This class implements a compliant RTP/MJPEG streaming solution designed
 * specifically for resource-constrained environments like the Raspberry Pi Zero
 * 2W. It features:
 * - Lock-free SPSC buffer integration
 * - Zero-copy packet construction
 * - RFC 2435 (RTP Payload Format for JPEG) compliance
 * - Precise frame rate control
 * - Automatic network buffer management
 *
 * The class uses pre-allocated buffers and atomic operations to minimize
 * resource contention and ensure smooth streaming performance even under heavy
 * load.
 */
template <typename FrameType>
class PacketPulse {
 public:
  /// Network configuration constants
  static constexpr size_t kMtu = 1200;  ///< Safe MTU for Ethernet networks
  static constexpr size_t kRtpHeaderSize = 12;  ///< Fixed RTP header size
  static constexpr size_t kJpegHeaderSize = 8;  ///< RFC 2435 JPEG header size
  static constexpr size_t kMaxPayloadSize =
      kMtu - kRtpHeaderSize - kJpegHeaderSize;  ///< Maximum JPEG fragment size

  /**
   * @brief Construct a new PacketPulse streamer instance
   *
   * @param frameBuffer Reference to the SPSC ring buffer containing JPEG frames
   * @param destIp Destination IP address for RTP stream (IPv4 dotted notation)
   * @param destPort Destination UDP port for RTP stream
   * @param width Frame width in pixels (must match input frames)
   * @param height Frame height in pixels (must match input frames)
   * @param payloadType RTP payload type (96-127 for dynamic types)
   * @param ssrc Synchronization Source identifier (unique stream ID)
   * @param clockRate RTP clock frequency in Hz (90000 for video)
   * @param framerate Target output frame rate in frames/second
   *
   * @throws std::runtime_error If network initialization fails
   */
  PacketPulse(RingMaster<FrameType>& frameBuffer, const std::string& destIp,
              uint16_t destPort, uint32_t width, uint32_t height,
              uint8_t payloadType = 96, uint32_t ssrc = 0,
              unsigned int clockRate = 90000, unsigned int framerate = 30);

  /**
   * @brief Destroy the PacketPulse instance
   *
   * Ensures clean shutdown by:
   * 1. Stopping the streaming thread
   * 2. Closing network sockets
   * 3. Releasing all allocated resources
   */
  ~PacketPulse();

  /**
   * @brief Start the RTP streaming thread
   *
   * Initializes the worker thread that handles:
   * - Frame dequeuing from buffer
   * - Packet fragmentation
   * - Network transmission
   * - Frame rate timing
   *
   * Thread-safe: Can be called multiple times but only starts once
   */
  void start();

  /**
   * @brief Stop the streaming thread
   *
   * Gracefully stops the worker thread within one frame interval.
   * Ensures final packet transmission completes before shutdown.
   *
   * Thread-safe: Can be called from any context
   */
  void stop();

 private:
  /**
   * @brief Main streaming loop executed by the worker thread
   *
   * Performs the following operations in sequence:
   * 1. Dequeues JPEG frames from the ring buffer
   * 2. Fragments frames into MTU-sized packets
   * 3. Constructs RTP headers with proper sequencing
   * 4. Transmits packets via non-blocking UDP
   * 5. Maintains precise frame timing using std::chrono
   */
  void streamingLoop();

  /**
   * @brief Initialize network resources
   *
   * @param destIp Destination IP address string
   * @param destPort Destination UDP port
   *
   * Creates and configures:
   * - Non-blocking UDP socket
   * - Socket buffer sizes
   * - Destination address structure
   *
   * @throws std::runtime_error If any network operation fails
   */
  void setupNetwork(const std::string& destIp, uint16_t destPort);

  /**
   * @brief Construct RTP header in buffer
   *
   * @param[out] buffer Destination buffer (must be ≥kRtpHeaderSize)
   * @param markerBit Set for last packet in frame
   * @param timestamp RTP presentation timestamp
   * @param sequenceNum Packet sequence number
   */
  void createRtpHeader(uint8_t* buffer, bool markerBit, uint32_t timestamp,
                       uint16_t sequenceNum) const;

  /**
   * @brief Construct JPEG-specific RTP payload header
   *
   * @param[out] buffer Destination buffer (must be ≥kJpegHeaderSize)
   * @param fragmentOffset Byte offset in JPEG frame
   * @param totalSize Total JPEG frame size in bytes
   */
  void createJpegHeader(uint8_t* buffer, uint32_t fragmentOffset,
                        uint32_t totalSize) const;

  // Configuration parameters (immutable after construction)
  RingMaster<FrameType>& frameBuffer_;  ///< Thread-safe frame buffer
  const uint32_t width_;                ///< Frame width in pixels
  const uint32_t height_;               ///< Frame height in pixels
  const uint8_t payloadType_;           ///< RTP payload type
  const uint32_t ssrc_;                 ///< Stream synchronization source
  const unsigned int clockRate_;        ///< RTP clock frequency (Hz)
  const unsigned int framerate_;        ///< Target frame rate (fps)
  const uint32_t timestampInc_;         ///< Timestamp increment per frame

  // Network resources
  int udpSocket_;         ///< UDP socket descriptor
  sockaddr_in destAddr_;  ///< Destination address

  // Thread control
  std::atomic<bool> isActive_;  ///< Streamer state flag
  std::thread worker_;          ///< Streaming thread handle

  // RTP state
  std::atomic<uint16_t> sequenceNum_;  ///< Packet sequence number
  std::atomic<uint32_t> timestamp_;    ///< Presentation timestamp

  // Pre-allocated buffers
  std::vector<uint8_t> packetBuffer_;  ///< Reusable packet buffer
};
