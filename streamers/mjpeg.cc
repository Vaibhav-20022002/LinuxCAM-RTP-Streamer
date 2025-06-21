#include "../main.hh"
#include "frameForger_MJPEG.hh"
#include "frameTypes.hh"
#include "packetPulse.hh"
#include "ringMaster.hh"
#include "turboSnap.hh"
#include <atomic>
#include <csignal>
#include <iostream>
#include <string_view>
#include <time.h>

// Default constants
constexpr uint16_t      WIDTH        = 640;
constexpr uint16_t      HEIGHT       = 480;
constexpr uint8_t       FRAMERATE    = 10;
constexpr uint8_t       JPEG_QUALITY = 50;
constexpr uint16_t      DEST_PORT    = 5004;
constexpr std::string   DEST_IP      = "0.0.0.0";
constexpr uint_fast8_t  PAYLOAD_TYPE = 96;
constexpr uint_fast16_t SSRC         = 12345;
constexpr uint_fast16_t CLOCK_RATE   = 90'000;

// Registering multiple signals
inline void signalHandlers() {
  std::signal(SIGINT, handleSignal);  // Ctrl+C
  std::signal(SIGTERM, handleSignal); // Termination request
  std::signal(SIGHUP, handleSignal);  // Hangup
}

void runMjpegStreamer(BestConfig &config, const std::string &device) {
  // Register signal handlers
  struct sigaction sa;
  sa.sa_handler = [](int) { g_running = 0; };
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  // Set variables using bestConfig
  const uint16_t width     = config.width;
  const uint16_t height    = config.height;
  const uint8_t  framerate = static_cast<uint8_t>(config.getFPS());
  const uint8_t  quality   = 50; // For RTP header (doesn't affect capture)

  // Network configuration
  const std::string   destIp      = "0.0.0.0"; // Replace with actual destination
  const uint16_t      destPort    = 5004;
  const uint_fast8_t  payloadType = 96;
  const uint_fast16_t ssrc        = 12345;
  const uint_fast16_t clockRate   = 90000;

  log("Starting MJPEG streamer: %ux%u @ %ufps", width, height, framerate);

  // Initialize ring buffer
  RingMaster<MjpegFrame> jpegBuffer;
  log("Ring buffer initialized");

  // Initialize frame capturer with config
  FrameForger_MJPEG forger(jpegBuffer, config);
  log("Initializing camera...");
  if (forger.initialize() != FrameForger_MJPEG::SUCCESS) {
    log("ERROR: Failed to initialize camera");
    return;
  }

  // Start frame capture
  log("Starting frame capture...");
  if (forger.startCapture() != FrameForger_MJPEG::SUCCESS) {
    log("ERROR: Failed to start capture");
    return;
  }

  // Initialize RTP streamer
  log("Initializing network streamer...");
  PacketPulse<MjpegFrame> streamer(
      jpegBuffer, destIp, destPort, width, height, quality, payloadType, ssrc, clockRate, framerate);
  streamer.start();
  streamer.writeSDP("stream.sdp", "LiveStream", destIp);
  log("Network streamer started");

  log("SDP file generated: stream.sdp");
  log("Streaming started - Press Ctrl+C to stop");

  // Frame counter and status tracking
  uint64_t frameCount     = 0;
  auto     lastStatusTime = std::chrono::steady_clock::now();

  // Main loop with proper shutdown handling
  while (g_running) {
    auto now = std::chrono::steady_clock::now();

    if (std::chrono::duration_cast<std::chrono::seconds>(now - lastStatusTime).count() >= 5) {
      log("Streaming - Frames: %lu, Buffer: %zu/%d", frameCount, jpegBuffer.size(), CAPACITY);
      lastStatusTime = now;
    }

    // Check if we need to shutdown
    if (!g_running) break;

    frameCount++;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Clean shutdown sequence
  log("Stopping network streamer...");
  streamer.stop();

  log("Stopping frame capture...");
  forger.stopCapture();

  // Wait for streams to flush
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  log("Shutdown complete");
}

// int main(int argc, char *argv[]) {
//   // Register signal handlers
//   signalHandlers();

//   // log("Configuration: %ux%u @ %ufps, JPEG quality: %u", width, height, framerate, quality);
//   // log("Destination: %s:%u", destIp.data(), destPort);

//   // // Intializing Ring Buffers for RGB & JPEG frame
//   // RingMaster<MjpegFrame> jpegBuffer;
//   // log("Ring buffers initialized");

//   // // Initializing JPEG frame capture
//   // FrameForger_MJPEG forger(jpegBuffer);
//   // log("Initializing camera...");
//   // if (forger.initialize()) {
//   //   log("ERROR: Failed to initialize camera");
//   //   return EXIT_FAILURE;
//   // }

//   // // Starting JPEG frame capture
//   // log("Starting frame capture...");
//   // if (forger.startCapture()) {
//   //   log("ERROR: Failed to start capture");
//   //   return EXIT_FAILURE;
//   // }
//   // log("Frame capture started successfully");

//   // // // Initializing JPEG encoder
//   // // log("Initializing JPEG encoder...");
//   // // TurboSnap<RingMaster<Frame>, RingMaster<jpegFrame_>> snap(
//   // //     rgbBuffer, jpegBuffer, width, height, quality);
//   // // log("JPEG encoder initialized");

//   // // Initializing RTP streamer
//   // log("Initializing network streamer...");
//   // PacketPulse<jpegFrame_> streamer(
//   //     jpegBuffer, destIp, destPort, width, height, quality, payloadType, ssrc, clockRate,
//   framerate);
//   // streamer.start();
//   // streamer.writeSDP("stream.sdp", "LiveStream", destIp);
//   // log("Network streamer started");

//   // log("Streaming started - Press Ctrl+C to stop");

//   // // Frame counter for periodic status updates
//   // uint_fast64_t frameCount     = 0;
//   // time_t        lastStatusTime = time(nullptr);

//   // // Calculate optimal sleep time based on framerate
//   // // For 10fps, we want about 100ms per frame
//   // const long      sleepTimeNs = 1000000000L / framerate;
//   // struct timespec ts          = {0, sleepTimeNs};

//   // // Main loop
//   // while (g_running.load(std::memory_order_relaxed)) {
//   //   // Sleep for short time to avoid high CPU spikes
//   //   nanosleep(&ts, nullptr);

//   //   // Print periodic status (every 5 seconds)
//   //   time_t now = time(nullptr);
//   //   if (now - lastStatusTime >= 5) {
//   //     log("Still streaming... (frames processed: %lu)", frameCount);
//   //     lastStatusTime = now;
//   //   }

//   //   ++frameCount;
//   // }

//   // // Clean shutdown
//   // log("Stopping network streamer...");
//   // streamer.stop();

//   // log("Stopping frame capture...");
//   // forger.stopCapture();

//   // log("Shutdown complete");

//   return EXIT_SUCCESS;
// }
