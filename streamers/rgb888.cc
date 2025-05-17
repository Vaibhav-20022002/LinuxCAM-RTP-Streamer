#include <time.h>

#include <atomic>
#include <csignal>
#include <iostream>
#include <string_view>

#include "FraframeTypes.hh"
#include "frameForger.hh"
#include "packetPulse.hh"
#include "ringMaster.hh"
#include "turboSnap.hh"

// Default constants
constexpr uint16_t WIDTH = 640;
constexpr uint16_t HEIGHT = 480;
constexpr uint8_t FRAMERATE = 10;
constexpr uint8_t JPEG_QUALITY = 50;
constexpr uint16_t DEST_PORT = 5004;
constexpr std::string DEST_IP = "0.0.0.0";
constexpr uint_fast8_t PAYLOAD_TYPE = 96;
constexpr uint_fast16_t SSRC = 12345;
constexpr uint_fast16_t CLOCK_RATE = 90'000;

// Use atomic for thread safety
std::atomic<bool> g_running{true};

// Signal handler function
inline void handleSignal(int signal) {
  printf("\n[INFO] Signal %d received, shutting down...\n", signal);
  g_running.store(false, std::memory_order_relaxed);
}

// Registering multiple signals
inline void signalHandlers() {
  std::signal(SIGINT, handleSignal);   // Ctrl+C
  std::signal(SIGTERM, handleSignal);  // Termination request
  std::signal(SIGHUP, handleSignal);   // Hangup
}

// Logging with Timestamps
char* getCurrentTimestamp() {
  static char buffer[24];  // static to avoid repeated allocation
  time_t now = time(nullptr);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&now));
  return buffer;
}

// Variadic template with minimal overhead
template <typename... Args>
inline void log(const char* format, Args... args) {
  printf("[%s] ", getCurrentTimestamp());
  printf(format, args...);
  printf("\n");
}

// Overload for string literals without format
inline void log(const char* message) {
  printf("[%s] %s\n", getCurrentTimestamp(), message);
}

int main(int argc, char* argv[]) {
  // Register signal handlers
  signalHandlers();

  // Setting up variables via env
  const uint16_t width = WIDTH, height = HEIGHT;
  const uint8_t quality = JPEG_QUALITY;
  const uint8_t framerate = FRAMERATE;
  const std::string& destIp = DEST_IP;
  const uint16_t destPort = DEST_PORT;
  const uint_fast8_t payloadType = PAYLOAD_TYPE;
  const uint_fast16_t ssrc = SSRC;
  const uint_fast16_t clockRate = CLOCK_RATE;

  log("Configuration: %ux%u @ %ufps, JPEG quality: %u", width, height,
      framerate, quality);
  log("Destination: %s:%u", destIp.data(), destPort);

  // Intializing Ring Buffers for RGB & JPEG frames
  RingMaster<Frame> rgbBuffer;
  RingMaster<jpegFrame_> jpegBuffer;
  log("Ring buffers initialized");

  // Initializing RGB frame capture
  FrameForger forger(rgbBuffer);
  log("Initializing camera...");
  if (forger.initialize()) {
    log("ERROR: Failed to initialize camera");
    return EXIT_FAILURE;
  }

  // Starting RGB frame capture
  log("Starting frame capture...");
  if (forger.startCapture()) {
    log("ERROR: Failed to start capture");
    return EXIT_FAILURE;
  }
  log("Frame capture started successfully");

  // Initializing JPEG encoder
  log("Initializing JPEG encoder...");
  TurboSnap<RingMaster<Frame>, RingMaster<jpegFrame_>> snap(
      rgbBuffer, jpegBuffer, width, height, quality);
  log("JPEG encoder initialized");

  // Initializing RTP streamer
  log("Initializing network streamer...");
  PacketPulse<jpegFrame_> streamer(jpegBuffer, destIp, destPort, width, height,
                                   quality, payloadType, ssrc, clockRate,
                                   framerate);
  streamer.start();
  streamer.writeSDP("stream.sdp", "LiveStream", destIp);
  log("Network streamer started");

  log("Streaming started - Press Ctrl+C to stop");

  // Frame counter for periodic status updates
  uint_fast64_t frameCount = 0;
  time_t lastStatusTime = time(nullptr);

  // Calculate optimal sleep time based on framerate
  // For 10fps, we want about 100ms per frame
  const long sleepTimeNs = 1000000000L / framerate;
  struct timespec ts = {0, sleepTimeNs};

  // Main loop
  while (g_running.load(std::memory_order_relaxed)) {
    // Sleep for short time to avoid high CPU spikes
    nanosleep(&ts, nullptr);

    // Print periodic status (every 5 seconds)
    time_t now = time(nullptr);
    if (now - lastStatusTime >= 5) {
      log("Still streaming... (frames processed: %lu)", frameCount);
      lastStatusTime = now;
    }

    ++frameCount;
  }

  // Clean shutdown
  log("Stopping network streamer...");
  streamer.stop();

  log("Stopping frame capture...");
  forger.stopCapture();

  log("Shutdown complete");

  return EXIT_SUCCESS;
}
