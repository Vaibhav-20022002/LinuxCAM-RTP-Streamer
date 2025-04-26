/**
 * @file jpeg2RTPServer.cpp
 * @author Vaibhav
 * @brief An extremely verbose implementation for streaming JPEG files over RTP.
 * @details This program implements a Real-time Transport Protocol (RTP)
 * streamer specifically designed for JPEG video data, adhering to the
 * specifications outlined in RFC 2435 ("RTP Payload Format for JPEG-compressed
 * Video").
 *
 * Key functionalities include:
 * - **JPEG File Loading:** Reads a sequence of JPEG files (e.g., "jpeg_0.jpg",
 * "jpeg_1.jpg", ...) from the local disk.
 * - **JPEG Parsing:** Performs basic parsing of each JPEG file to:
 * - Validate the JPEG structure (finding SOI and EOI markers).
 * - Extract essential metadata like image width and height (from the SOF0
 * marker).
 * - Determine the JPEG type (specifically, the chroma subsampling format like
 * YUV 4:2:0) required for the RTP JPEG header.
 * - Estimate the JPEG quality factor (from quantization tables, DQT marker) for
 * the RTP header.
 * - **Frame Storage:** Stores the loaded JPEG data along with extracted
 * metadata in memory using a `std::deque` for efficient access during
 * streaming.
 * - **SDP File Generation:** Creates a Session Description Protocol (SDP) file
 * ("stream.sdp") as defined by RFC 4566. This file contains parameters
 * describing the stream (IP address, port, payload type, resolution, frame
 * rate), allowing standard media players like VLC to easily connect and play
 * the stream.
 * - **UDP Socket Setup:** Initializes a UDP socket for network communication,
 * setting appropriate buffer sizes for potentially high data rates.
 * - **RTP Packet Construction:** Dynamically builds RTP packets for each JPEG
 * frame:
 * - Constructs the standard 12-byte RTP header (Version, Payload Type, Sequence
 * Number, Timestamp, SSRC).
 * - Constructs the 8-byte RTP JPEG header extension specific to RFC 2435
 * (Fragment Offset, Type, Quality, Width/Height in blocks).
 * - **JPEG Fragmentation:** Implements logic to fragment large JPEG frames into
 * multiple RTP packets if they exceed the maximum payload size (calculated to
 * avoid IP layer fragmentation). Crucially, this fragmentation respects JPEG
 * marker boundaries, especially restart markers (RSTm), as recommended by RFC
 * 2435, to ensure decodability at the receiver. The 'Marker' bit in the RTP
 * header is set for the last packet of a frame.
 * - **Network Transmission:** Sends the constructed RTP packets over UDP to the
 * specified destination IP address and port.
 * - **Frame Rate Control:** Uses `std::chrono` and
 * `std::this_thread::sleep_for` to maintain a consistent streaming frame rate,
 * pacing the transmission of frames.
 * - **Logging:** Provides simple timestamped logging for informational messages
 * and error reporting.
 *
 * The implementation prioritizes clarity and adherence to the relevant RFCs
 * while demonstrating core concepts of network streaming, packetization, and
 * basic media data handling.
 */

// ========================================================================== //
// ==                            INCLUDE DIRECTIVES                        == //
// ========================================================================== //

#include <arpa/inet.h>  // Provides functions for network address manipulation, specifically 'inet_pton' (string to network address) and 'htonl'/'htons' (host to network byte order conversions). Needed for setting up the destination address and RTP headers.
#include <netinet/in.h>  // Defines internet address family structures like 'sockaddr_in', which holds IPv4 address and port information. Essential for socket programming.
#include <sys/socket.h>  // Contains core socket API function declarations ('socket', 'sendto', 'setsockopt') and related constants (AF_INET, SOCK_DGRAM). Fundamental for network communication.
#include <unistd.h>  // Provides POSIX operating system API functions, notably 'close()' used here to close the network socket file descriptor.

#include <algorithm>  // Provides standard algorithms like 'std::min' (used for payload size calculation) and 'std::copy_n' (used for copying JPEG data into packets).
#include <chrono>  // Provides time-related utilities ('std::chrono::system_clock', 'std::chrono::steady_clock', 'std::chrono::milliseconds'). Used for logging timestamps, calculating frame delays, and controlling the streaming rate.
#include <cstring>  // Provides C-style string manipulation functions, specifically 'strerror()' used here to get human-readable error messages corresponding to 'errno' values.
#include <deque>  // Provides the 'std::deque' (double-ended queue) container. Used here to store the loaded 'FrameData' objects, offering efficient insertion at the back and iteration.
#include <fstream>  // Provides classes for file stream operations ('std::ifstream' for reading JPEGs, 'std::ofstream' for writing the SDP file).
#include <iomanip>  // Provides stream manipulators like 'std::put_time'. Used here to format timestamps for logging.
#include <iostream>  // Provides standard input/output stream objects ('std::cout' for logging, 'std::cerr' for errors).
#include <thread>  // Provides threading capabilities, specifically 'std::this_thread::sleep_for()', used to pause execution and maintain the desired frame rate.
#include <vector>  // Provides the 'std::vector' dynamic array container. Used extensively for storing raw byte data (JPEG file content, RTP packets).

// ========================================================================== //
// ==                             GLOBAL CONSTANTS                         == //
// ========================================================================== //

/**
 * @brief Constant string defining the destination IP address for the RTP
 * stream.
 * @details "127.0.0.1" is the standard loopback address, meaning the stream
 * will be sent to the same machine that is running this program. Change this to
 * a specific network IP address to stream to another device on the same
 * network.
 */
const std::string DEST_IP = "127.0.0.1";

/**
 * @brief Constant integer defining the destination UDP port number for the RTP
 * stream.
 * @details Port 5004 is a common choice for the first RTP stream in a session
 * (often paired with port 5005 for RTCP, although RTCP is not implemented
 * here). Ensure this port is not blocked by firewalls. UDP requires a port
 * number for demultiplexing packets at the receiver.
 */
const int DEST_PORT = 5004;

/**
 * @brief Constant integer defining the target frame rate of the output video
 * stream.
 * @details This value (in frames per second) determines the pacing of the
 * streaming loop. The program will attempt to send one frame every (1000 /
 * FRAME_RATE) milliseconds. It also influences the RTP timestamp increment
 * between frames.
 */
const int FRAME_RATE = 15;

/**
 * @brief Constant string defining the prefix part of the input JPEG filenames.
 * @details The program expects input files named sequentially using this prefix
 * followed by a number and the suffix (e.g., "jpeg_0.jpg", "jpeg_1.jpg").
 */
const std::string INPUT_FILE_PREFIX = "jpeg_";

/**
 * @brief Constant string defining the suffix (file extension) of the input JPEG
 * filenames.
 * @details Used in conjunction with the prefix and a counter to construct the
 * full filenames of the JPEG images to be loaded and streamed.
 */
const std::string INPUT_FILE_SUFFIX = ".jpg";

/**
 * @brief Constant string defining the filename for the generated Session
 * Description Protocol (SDP) file.
 * @details This file will contain metadata about the RTP stream, allowing media
 * players like VLC to easily open and play the stream by simply opening this
 * file (`vlc stream.sdp`).
 */
const std::string SDP_FILENAME = "stream.sdp";

// ========================================================================== //
// ==                              DATA STRUCTURES                         == //
// ========================================================================== //

/**
 * @brief Structure designed to hold the data and essential metadata for a
 * single video frame.
 * @details This aggregates the raw compressed JPEG data along with parameters
 * needed specifically for constructing the RTP JPEG header (RFC 2435). Storing
 * these together simplifies the streaming loop.
 */
struct FrameData {
  /**
   * @brief A vector containing the raw bytes of the compressed JPEG frame data.
   * @details This typically includes the data from the Start Of Image (SOI)
   * marker (0xFFD8) up to and including the End Of Image (EOI) marker (0xFFD9).
   */
  std::vector<uint8_t> jpeg;

  /**
   * @brief The width of the decoded frame in pixels.
   * @details Extracted from the JPEG header (SOF0 marker). Used for SDP file
   * generation and potentially for the RTP JPEG header (though the header uses
   * width in 8-pixel blocks).
   */
  uint16_t width;

  /**
   * @brief The height of the decoded frame in pixels.
   * @details Extracted from the JPEG header (SOF0 marker). Used for SDP file
   * generation and potentially for the RTP JPEG header (though the header uses
   * height in 8-pixel blocks).
   */
  uint16_t height;

  /**
   * @brief The estimated JPEG quality factor (typically 1-100, or 255 if
   * unknown).
   * @details This value is required for the 'Q' field in the RTP JPEG header
   * (RFC 2435). It's estimated by examining the quantization tables (DQT
   * marker) within the JPEG data. Lower quantization values imply higher
   * quality, so the estimation maps this inversely.
   */
  uint8_t quality;

  /**
   * @brief The JPEG type code indicating the color subsampling format.
   * @details This corresponds to the 'Type' field in the RTP JPEG header (RFC
   * 2435, Section 3.1.3). Common values: 0 for grayscale, 1 for YCbCr 4:2:0, 2
   * for YCbCr 4:2:2. It's determined by examining the component sampling
   * factors in the SOF0 marker.
   */
  uint8_t type;
};

// ========================================================================== //
// ==                       UTILITY & HELPER FUNCTIONS                     == //
// ========================================================================== //

/**
 * @brief Logs an informational message to the standard output stream, prefixed
 * with a timestamp.
 * @details Provides a simple mechanism for tracing the program's execution flow
 * and status. Uses `std::chrono` to get the current time and `std::put_time` to
 * format it as HH:MM:SS.
 * @param message A constant reference to the `std::string` containing the
 * message to be logged.
 */
void log(const std::string& message) {
  /**
   * @brief Get the current point in time from the system clock.
   * @details `std::chrono::system_clock` provides the wall-clock time.
   */
  auto now = std::chrono::system_clock::now();

  /**
   * @brief Convert the high-resolution time point to a standard `time_t`
   * format.
   * @details `time_t` is a legacy C time representation, often suitable for
   * calendar time functions.
   */
  auto t = std::chrono::system_clock::to_time_t(now);

  /**
   * @brief Output the formatted time, the "[INFO]" prefix, and the message to
   * `std::cout`.
   * @details `std::localtime(&t)` converts `time_t` to a broken-down local time
   * structure. `std::put_time` formats this structure according to the format
   * string "%T" (equivalent to %H:%M:%S). `std::endl` ensures the message is
   * flushed and a newline is added.
   */
  std::cout << std::put_time(std::localtime(&t), "%T")  // Format as HH:MM:SS
            << " [INFO] " << message
            << std::endl;  // Output with INFO prefix and newline
}

/**
 * @brief Logs an error message to the standard error stream, appending system
 * error details.
 * @details Used for reporting errors encountered during program execution. It
 * includes the specific error message provided by the caller and also appends
 * the system-level error description obtained via `strerror(errno)`, providing
 * more context for system call failures.
 * @param msg A constant reference to the `std::string` containing the primary
 * error message.
 */
void error(const std::string& msg) {
  /**
   * @brief Output the "[ERROR]" prefix, the user message, and the system error
   * message to `std::cerr`.
   * @details `errno` is a global variable set by system calls upon failure to
   * indicate the error type. `strerror(errno)` converts the integer `errno`
   * value into a human-readable string description (e.g., "Permission denied",
   * "No such file or directory"). Outputting to `std::cerr` is standard
   * practice for error messages.
   */
  std::cerr << "[ERROR] " << msg << " (" << strerror(errno) << ")" << std::endl;
}

/**
 * @brief Searches for a specific two-byte JPEG marker within a vector of bytes.
 * @details JPEG files use markers (starting with 0xFF followed by a marker code
 * byte, excluding 0x00 and 0xFF) to delimit different segments of data
 * (headers, scan data, etc.). This function finds the first occurrence of a
 * specified marker (`0xFF` followed by the `marker` byte) within the `data`
 * vector, starting from the `start` index.
 * @param data A constant reference to the `std::vector<uint8_t>` containing the
 * byte data (e.g., JPEG file content) to search within.
 * @param marker The second byte (`uint8_t`) of the marker sequence to find
 * (e.g., 0xD8 for SOI, 0xC0 for SOF0, 0xDB for DQT, 0xD9 for EOI).
 * @param start The starting index (`size_t`) within the `data` vector from
 * where the search should begin. Defaults to 0 (start of the vector).
 * @return The index (`int`) of the first byte (the 0xFF) of the found marker
 * sequence. Returns -1 if the marker is not found within the specified range.
 */
int find_marker(const std::vector<uint8_t>& data, uint8_t marker,
                size_t start = 0) {
  /**
   * @brief Iterate through the data vector starting from the `start` index.
   * @details The loop continues up to `data.size() - 1` because we need to look
   * at pairs of bytes (`data[i]` and `data[i+1]`).
   */
  for (size_t i = start; i < data.size() - 1; ++i) {
    /**
     * @brief Check if the current byte is 0xFF and the next byte matches the
     * target `marker`.
     */
    if (data[i] == 0xFF && data[i + 1] == marker) {
      return i;  // Found the marker, return the index of the 0xFF byte.
    }
  }
  /**
   * @brief If the loop completes without finding the marker, return -1.
   */
  return -1;  // Marker not found.
}

/**
 * @brief Attempts to locate JPEG quantization tables (DQT marker) and estimate
 * the quality factor.
 * @details This function searches for the DQT (Define Quantization Table)
 * marker (0xFFDB) in the JPEG data. If found, it calculates a rough estimate of
 * the JPEG quality setting (1-100) based on the average magnitude of the
 * initial quantization values in the first table encountered. Lower
 * quantization values generally correspond to higher quality. This estimated
 * quality factor is needed for the RTP JPEG header. Note: This implementation
 * *estimates* quality and does *not* fully parse or extract the complete tables
 * into `quant_tables`.
 * @param data A constant reference to the `std::vector<uint8_t>` containing the
 * JPEG frame data.
 * @param quant_tables A reference to a `std::vector<uint8_t>` intended to hold
 * the extracted quantization table data. **Note:** This implementation
 * currently does not populate this vector.
 * @param q_factor A reference to a `uint8_t` where the estimated quality factor
 * (1-100, or 255 if tables are not found) will be stored.
 * @return `size_t` The size of the quantization table data found. **Note:**
 * This implementation currently always returns 0, as it only estimates quality,
 * not extracts tables.
 */
size_t extract_quantization_tables(const std::vector<uint8_t>& data,
                                   std::vector<uint8_t>& quant_tables,
                                   uint8_t& q_factor) {
  /**
   * @brief Search for the DQT (Define Quantization Table) marker (0xFF, 0xDB).
   */
  int dqt_pos = find_marker(data, 0xDB);  // DQT marker code is 0xDB

  /**
   * @brief Handle the case where no DQT marker is found.
   */
  if (dqt_pos == -1) {
    // No quantization tables found in the JPEG data. This might occur in some
    // JPEGs.
    /**
     * @brief Set quality factor to 255 to indicate unknown/missing tables, as
     * per RFC 2435 recommendation (though 0 is also allowed).
     */
    q_factor = 255;  // Use 255 to signify 'tables not included' or unknown.
    /**
     * @brief Return 0 as no table data was processed or extracted.
     */
    return 0;
  }

  /**
   * @brief Estimate the quality factor based on the values in the first found
   * quantization table.
   * @details Check if there are enough bytes following the DQT marker to read
   * some table data. The DQT segment structure is typically: FF DB (Marker),
   * Length (2 bytes), Table Info (1 byte), Table Data (Nx 64 bytes). We look at
   * bytes starting from `dqt_pos + 5` (skipping marker, length, and info byte).
   */
  if (dqt_pos + 6 <
      data.size()) {  // Need at least marker(2)+length(2)+info(1)+one_value(1)
                      // = 6 bytes
    /**
     * @brief Initialize sum and count for averaging quantization values.
     */
    uint32_t sum = 0;
    int count = 0;
    /**
     * @brief Sum the first few quantization values (e.g., first 10) from the
     * table.
     * @details This provides a sample representative of the table's magnitude.
     * Loop safely checks boundaries.
     */
    for (size_t i = dqt_pos + 5; i < dqt_pos + 15 && i < data.size(); i++) {
      sum += data[i];  // Add quantization value to sum.
      count++;         // Increment count of values added.
    }

    /**
     * @brief Calculate the average quantization value if any values were read.
     */
    if (count > 0) {
      /**
       * @brief Compute the average value.
       */
      uint8_t avg = sum / count;
      /**
       * @brief Estimate the quality factor (scale 1-100) based on the average.
       * @details This is a heuristic: lower average quantization values imply
       * higher quality. The formula `100 - (avg / 2.55)` attempts to map the
       * average (potentially 0-255) inversely to a 1-100 quality scale. Clamp
       * the result between 1 and 100.
       */
      q_factor = std::max(1, std::min(100, (int)(100 - (avg / 2.55))));
    } else {
      /**
       * @brief If no values could be read (unlikely here, but safe check),
       * assign a default medium quality.
       */
      q_factor = 50;  // Default to medium quality if calculation fails.
    }
  } else {
    /**
     * @brief If the DQT segment is too short to read values, assign a default
     * medium quality.
     */
    q_factor = 50;  // Default to medium quality if segment seems truncated.
  }

  /**
   * @brief Return 0, indicating that although quality was estimated, the full
   * tables weren't extracted/returned.
   * @details A more complete implementation would parse the DQT segment length
   * and copy the table data into the `quant_tables` vector, returning the
   * actual size.
   */
  return 0;  // Placeholder: We don't actually extract the tables into the
             // output vector in this simplified version.
}

/**
 * @brief Determines the JPEG type (color subsampling format) based on the SOF0
 * marker data.
 * @details This function searches for the Start Of Frame 0 (SOF0 - Baseline
 * DCT) marker (0xFFC0). If found, it examines the component information within
 * the SOF0 segment to determine the chroma subsampling format (e.g., Grayscale,
 * 4:2:0, 4:2:2, 4:4:4). This 'type' value is required for the RTP JPEG header
 * (RFC 2435, Section 3.1.3).
 * @param data A constant reference to the `std::vector<uint8_t>` containing the
 * JPEG frame data.
 * @return `uint8_t` The JPEG type code according to RFC 2435:
 * - 0: Grayscale (typically 1 component)
 * - 1: YCbCr 4:2:0 (typically 3 components, Y horizontal sampling = 2)
 * - 2: YCbCr 4:2:2 or 4:4:4 (typically 3 components, Y horizontal sampling = 1
 * or others) Defaults to 1 (4:2:0) if the format cannot be reliably determined.
 */
uint8_t determine_jpeg_type(const std::vector<uint8_t>& data) {
  /**
   * @brief Search for the SOF0 (Start Of Frame - Baseline DCT) marker (0xFF,
   * 0xC0).
   */
  int sof_pos = find_marker(data, 0xC0);  // SOF0 marker code is 0xC0

  /**
   * @brief Check if the SOF0 marker was found and if there's enough data
   * following it to read component info.
   * @details The SOF0 segment structure: FF C0 (Marker), Length (2 bytes),
   * Precision (1 byte), Height (2 bytes), Width (2 bytes), Num Components (1
   * byte), [Component Info (3 bytes each)]... We need at least up to the Num
   * Components byte (`sof_pos + 9`). For 3 components, we need up to the
   * sampling factors of the first component (`sof_pos + 11`).
   */
  if (sof_pos != -1 &&
      sof_pos + 10 <= data.size()) {  // Check up to Num Components byte
    /**
     * @brief Get the number of image components from the SOF0 segment (at
     * offset 9 from the marker start).
     * @details Typically 1 for grayscale, 3 for YCbCr color.
     */
    uint8_t components = data[sof_pos + 9];

    /**
     * @brief Handle Grayscale case.
     */
    if (components == 1) {
      return 0;  // Type 0 for Grayscale.
    }
    /**
     * @brief Handle Color case (typically 3 components).
     */
    else if (components == 3) {
      /**
       * @brief Check if there's enough data to read the sampling factors for
       * the first component (Y).
       * @details Each component has 3 bytes of info: ID, Sampling Factors
       * (Horiz/Vert), Quant Table ID. The Y component is usually first.
       * Sampling factors are at offset 11. We need up to byte 11 (index sof_pos
       * + 11). Accessing byte at sof_pos+11 requires size >= sof_pos+12. Let's
       * check up to 17 for safety as used later, though 12 is minimum for Y
       * sampling.
       */
      if (sof_pos + 12 <= data.size()) {  // Need byte at sof_pos+11
        /**
         * @brief Extract the Horizontal Sampling factor for the first component
         * (usually Y).
         * @details This is stored in the high nibble (4 bits) of the byte at
         * offset 11.
         * `>> 4` shifts the high nibble into the lower 4 bits.
         */
        uint8_t y_sampling_h =
            data[sof_pos + 11] >> 4;  // Horizontal sampling factor for Y

        // Vertical sampling is in the lower nibble: data[sof_pos + 11] & 0x0F;

        /**
         * @brief Determine type based on Y's horizontal sampling factor.
         * @details - If Y H-sampling is 2 (meaning Cb, Cr are likely 1, i.e.,
         * half horizontal resolution), it's typically 4:2:0 or 4:2:2. RFC 2435
         * Type 1 covers 4:2:0.
         * - If Y H-sampling is 1 (meaning Cb, Cr are likely 1, i.e., full
         * horizontal resolution), it's typically 4:4:4. RFC 2435 Type 2 covers
         * 4:2:2 and implies it might cover 4:4:4 too, or that Type 1/2
         * distinction might be based on other factors in the RFC context.
         * Common practice: H=2 maps to 4:2:0. H=1 maps to 4:2:2 or 4:4:4.
         */
        if (y_sampling_h == 2) {
          return 1;  // Type 1 usually corresponds to 4:2:0 (most common for
                     // video).
        } else if (y_sampling_h == 1) {
          // Could be 4:4:4 or potentially 4:2:2 depending on vertical sampling.
          // RFC 2435 Type 2 is defined as 4:2:2. Let's map H=1 to Type 2.
          return 2;  // Type 2 corresponds to 4:2:2.
        }
        // Other sampling factors (e.g., 4) are less common and not explicitly
        // mapped in basic RFC types. Fall through to default.
      }
      // Fall through to default if component info is too short.
    }
    // Fall through to default if component count is not 1 or 3.
  }

  /**
   * @brief Default case if marker not found or parsing fails.
   * @details Assume Type 1 (YCbCr 4:2:0) as it's a very common format for
   * compressed video.
   */
  return 1;  // Default to Type 1 (YUV 4:2:0) if specific type cannot be
             // determined.
}

/**
 * @brief Loads a sequence of JPEG files from disk, parses metadata, and stores
 * them in memory.
 * @details This function iteratively attempts to load files named according to
 * the global `INPUT_FILE_PREFIX`, a sequential integer counter (starting from
 * 0), and `INPUT_FILE_SUFFIX` (e.g., "jpeg_0.jpg", "jpeg_1.jpg", ...). For each
 * file found, it reads the binary data, validates it as a basic JPEG (checks
 * for SOI and EOI markers), extracts the width, height, estimated quality, and
 * type using helper functions, and stores this information along with the JPEG
 * data itself in a `FrameData` structure within the provided `frames` deque.
 * The process stops when a file in the sequence is not found.
 * @param frames A reference to a `std::deque<FrameData>` object. Loaded frames
 * will be appended (`push_back`) to this deque. The deque should be empty or
 * cleared before calling if only the current sequence is desired.
 * @return `true` if at least one frame was successfully loaded. `false` if the
 * very first file ("jpeg_0.jpg") couldn't be found or if any critical error
 * occurred during loading or parsing of any file.
 */
bool load_frames(std::deque<FrameData>& frames) {
  /**
   * @brief Initialize a counter used for generating sequential filenames.
   * Starts at 0.
   */
  int count = 0;

  /**
   * @brief Log the start of the frame loading process.
   */
  log("Starting frame loading...");

  /**
   * @brief Loop indefinitely, attempting to load the next file in the sequence.
   * The loop breaks internally when a file is not found.
   */
  while (true) {
    /**
     * @brief Construct the filename for the current frame number in the
     * sequence.
     * @details Uses the prefix, the current count converted to a string, and
     * the suffix.
     */
    std::string filename =
        INPUT_FILE_PREFIX + std::to_string(count) + INPUT_FILE_SUFFIX;

    /**
     * @brief Attempt to open the constructed filename for reading in binary
     * mode.
     * @details `std::ios::binary` prevents text-mode translations.
     * `std::ios::ate` positions the file pointer at the end of the file
     * initially. This is a common trick to easily get the file size using
     * `tellg()` right after opening.
     */
    std::ifstream file(filename, std::ios::binary | std::ios::ate);

    /**
     * @brief Check if the file failed to open.
     * @details `!file` evaluates to true if the file stream is in an error
     * state (e.g., file not found, permissions error).
     */
    if (!file) {
      /**
       * @brief Check if this is the very first file (count is 0).
       * @details If the first file ("jpeg_0.jpg") cannot be opened, it's
       * considered a fatal error for this program.
       */
      if (count == 0) {
        error("No input files found (expected '" + INPUT_FILE_PREFIX + "0" +
              INPUT_FILE_SUFFIX + "')");  // Report specific error.
        return false;  // Abort loading operation, return failure.
      }
      /**
       * @brief If it's not the first file, assume we've reached the end of the
       * sequence.
       */
      break;  // Exit the while loop normally, indicating end of file sequence.
    }

    /**
     * @brief Get the size of the file in bytes.
     * @details Since the file was opened with `std::ios::ate`, `file.tellg()`
     * returns the position of the end-of-file marker, which equals the file
     * size.
     */
    size_t size = file.tellg();

    /**
     * @brief Reposition the file pointer back to the beginning of the file for
     * reading.
     */
    file.seekg(0);

    /**
     * @brief Create a vector of bytes (`uint8_t`) sized to hold the entire file
     * content.
     */
    std::vector<uint8_t> buffer(size);

    /**
     * @brief Read the entire file content into the `buffer` vector.
     * @details `file.read()` reads raw bytes.
     * `reinterpret_cast<char*>(buffer.data())` provides a `char*` pointer
     * required by `read`, pointing to the vector's underlying data. Check the
     * stream state (`!file.read(...)`) after the read attempt to ensure it was
     * successful.
     */
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
      error("Failed to read file content: " + filename);  // Report read error.
      // File object will close automatically when it goes out of scope here.
      return false;  // Abort loading operation, return failure.
    }
    // File is implicitly closed here as 'file' goes out of scope.

    /**
     * @brief Find the Start Of Image (SOI) marker (0xFF, 0xD8) in the buffer.
     */
    int soi = find_marker(buffer, 0xD8);
    /**
     * @brief Find the End Of Image (EOI) marker (0xFF, 0xD9) in the buffer.
     */
    int eoi = find_marker(buffer, 0xD9);

    /**
     * @brief Perform a basic validation check for JPEG integrity.
     * @details Ensure both SOI and EOI markers were found. This doesn't
     * guarantee a perfectly valid JPEG but catches obviously truncated or
     * non-JPEG files.
     */
    if (soi == -1 || eoi == -1) {
      error("Invalid or incomplete JPEG file (missing SOI/EOI): " +
            filename);  // Report format error.
      return false;     // Abort loading operation, return failure.
    }

    /**
     * @brief Create a `FrameData` structure to hold the data and metadata for
     * this frame.
     */
    FrameData frame;

    /**
     * @brief Copy the relevant JPEG data (from SOI marker to EOI marker,
     * inclusive) into the `frame.jpeg` vector.
     * @details `buffer.begin() + soi` points to the start of the SOI marker.
     * `buffer.begin() + eoi + 2` points just after the EOI marker (since EOI is
     * 2 bytes). `frame.jpeg.assign()` efficiently copies this range into the
     * frame's vector.
     */
    frame.jpeg.assign(buffer.begin() + soi, buffer.begin() + eoi + 2);

    /**
     * @brief Find the Start Of Frame 0 (SOF0 - Baseline DCT) marker (0xFF,
     * 0xC0) within the extracted frame data.
     * @details The SOF0 marker contains the image dimensions (width and
     * height).
     */
    int sof0 = find_marker(frame.jpeg, 0xC0);

    /**
     * @brief Check if SOF0 was found and if there's enough data following it to
     * read dimensions.
     * @details Width and height are 2 bytes each, located at offsets +5
     * (Height) and +7 (Width) relative to the start of the SOF0 marker. We need
     * at least 9 bytes after the marker (index `sof0 + 8`). Check `sof0 + 9 >=
     * size` for safety.
     */
    if (sof0 == -1 || static_cast<size_t>(sof0 + 9) > frame.jpeg.size()) {
      error("Missing or truncated SOF0 marker (cannot find dimensions) in: " +
            filename);  // Report missing dimension data.
      return false;     // Abort loading operation, return failure.
    }

    /**
     * @brief Extract the frame height from the SOF0 segment.
     * @details Height is a 16-bit unsigned integer stored in big-endian format
     * at offsets +5 and +6 relative to the start of the SOF0 marker.
     * `frame.jpeg[sof0 + 5]` is the high byte, `frame.jpeg[sof0 + 6]` is the
     * low byte. Shift the high byte left by 8 bits (`<< 8`) and OR (`|`) it
     * with the low byte. Cast high byte to uint16_t before shifting to avoid
     * potential issues.
     */
    frame.height = (static_cast<uint16_t>(frame.jpeg[sof0 + 5]) << 8) |
                   frame.jpeg[sof0 + 6];

    /**
     * @brief Extract the frame width from the SOF0 segment.
     * @details Width is a 16-bit unsigned integer stored in big-endian format
     * at offsets +7 and +8 relative to the start of the SOF0 marker. Logic is
     * the same as for height, using bytes at `sof0 + 7` (high) and `sof0 + 8`
     * (low).
     */
    frame.width = (static_cast<uint16_t>(frame.jpeg[sof0 + 7]) << 8) |
                  frame.jpeg[sof0 + 8];

    /**
     * @brief Determine the JPEG type (color subsampling format) using the
     * helper function.
     * @details This analyzes the component information within the SOF0 segment.
     */
    frame.type = determine_jpeg_type(frame.jpeg);

    /**
     * @brief Declare a temporary vector for quantization tables (though not
     * populated in this version).
     */
    std::vector<uint8_t>
        quant_tables;  // Not actually filled by extract_quantization_tables in
                       // this version.

    /**
     * @brief Call the helper function to estimate the JPEG quality factor.
     * @details This analyzes the DQT marker(s) if present. The result is stored
     * in `frame.quality`.
     */
    extract_quantization_tables(frame.jpeg, quant_tables, frame.quality);

    /**
     * @brief Add the fully populated `FrameData` structure to the back of the
     * `frames` deque.
     */
    frames.push_back(std::move(frame));  // Use std::move for potentially better
                                         // performance with vector assignment.

    /**
     * @brief Increment the file counter for the next iteration. Post-increment
     * used here.
     */
    count++;

    /**
     * @brief Log progress periodically (e.g., every 50 frames loaded).
     * @details Uses the modulo operator (`%`) to check if the count is a
     * multiple of 50.
     */
    if (count % 50 == 0) {
      log("Loaded " + std::to_string(count) + " frames...");
    }

  }  // End of while(true) loop

  /**
   * @brief Log the total number of frames successfully loaded after the loop
   * finishes.
   * @details Uses `frames.size()` to get the final count in the deque.
   */
  log("Total loaded: " + std::to_string(frames.size()) + " frames");

  /**
   * @brief Return true, indicating the loading process completed (even if 0
   * frames were loaded after the first check, as long as no errors occurred).
   * It might be better to check `!frames.empty()` here if loading zero frames
   * should be considered failure. The original logic returns true if the loop
   * finishes.
   */
  return true;  // Loading sequence finished (or no files found after the first
                // one).
}

/**
 * @brief Generates and writes a Session Description Protocol (SDP) file.
 * @details This function creates a text file formatted according to SDP
 * specifications (RFC 4566). The SDP file describes the RTP stream, including
 * connection details (IP address, port), media format (JPEG payload type 26),
 * clock rate (90000 Hz for video), frame rate, and image dimensions. Media
 * players like VLC can use this file to connect to and play the stream.
 * @param width The width (`uint16_t`) of the video frames in pixels. Used in
 * the `fmtp` attribute line.
 * @param height The height (`uint16_t`) of the video frames in pixels. Used in
 * the `fmtp` attribute line.
 * @return `true` if the SDP file was successfully created and written. `false`
 * if the file could not be opened for writing.
 */
bool create_sdp(uint16_t width, uint16_t height) {
  /**
   * @brief Attempt to open the SDP file (filename defined by `SDP_FILENAME`)
   * for writing.
   * @details `std::ofstream` creates an output file stream. If the file exists,
   * it will be overwritten.
   */
  std::ofstream file(SDP_FILENAME);

  /**
   * @brief Check if the file stream was opened successfully.
   * @details `!file` is true if opening failed (e.g., permissions issue).
   */
  if (!file) {
    error("Failed to create/open SDP file: " +
          SDP_FILENAME);  // Report file creation error.
    return false;         // Abort SDP creation, return failure.
  }

  /**
   * @brief Write the SDP content line by line to the file stream using the `<<`
   * operator.
   * @details Each line follows the SDP standard format (`<type>=<value>`).
   * - v=0: SDP version 0.
   * - o=- 0 0 IN IP4 <DEST_IP>: Originator field (user '-', session ID 0,
   * version 0, network type IN, address type IP4, address).
   * - s=JPEG Stream: Session name.
   * - c=IN IP4 <DEST_IP>: Connection data (network type IN, address type IP4,
   * connection address). Specifies the destination address for the stream
   * *media*, although RTP usually uses the 'm=' line address. Using DEST_IP
   * here for consistency.
   * - t=0 0: Timing field (start time 0, stop time 0, indicating a permanent
   * session).
   * - m=video <DEST_PORT> RTP/AVP 26: Media description. Media type 'video',
   * destination port `DEST_PORT`, protocol 'RTP/AVP' (RTP over UDP using the
   * Audio/Video Profile), payload type '26'. 26 is the standard static payload
   * type assigned to JPEG in RFC 3551.
   * - a=rtpmap:26 JPEG/90000: Attribute line mapping payload type 26 to the
   * encoding name "JPEG" and a clock rate of 90000 Hz. 90kHz is the standard
   * RTP clock rate for video, allowing fine-grained timestamping.
   * - a=framerate:<FRAME_RATE>: Attribute line specifying the video frame rate.
   * - a=fmtp:26 width=<width>; height=<height>;: Format-specific parameters for
   * payload type 26. Specifies the image dimensions. RFC 2435 doesn't
   * explicitly define these fmtp parameters, but they are commonly
   * used/understood by receivers like VLC.
   */
  file << "v=0\n"  // SDP Version
       << "o=- 0 0 IN IP4 " << DEST_IP
       << "\n"  // Origin field (user, sess id, sess ver, net type, addr type,
                // addr)
       << "s=JPEG Stream\n"  // Session Name
       << "c=IN IP4 " << DEST_IP
       << "\n"       // Connection Data (applies to all media unless overridden)
       << "t=0 0\n"  // Timing (permanent session)
       << "m=video " << DEST_PORT
       << " RTP/AVP 26\n"  // Media description (video, port, proto, payload
                           // type 26=JPEG)
       << "a=rtpmap:26 JPEG/90000\n"  // Attribute: RTP map (PT 26 -> JPEG
                                      // encoding, 90kHz clock)
       << "a=framerate:" << FRAME_RATE << "\n"  // Attribute: Frame rate
       << "a=fmtp:26 width=" << width << "; height=" << height
       << ";\n";  // Attribute: Format specific parameters (width, height)

  /**
   * @brief Check the file stream state after writing. Although less critical
   * for ofstream unless disk full, it's good practice. `file.good()` could be
   * used.
   */
  // Optional: Check file.good() here.

  /**
   * @brief File stream automatically closes when 'file' goes out of scope.
   * Return success.
   */
  return true;  // SDP file written successfully.
}

/**
 * @brief Sends a block of data as a single UDP packet to the specified
 * destination.
 * @details This is a simple wrapper around the standard `sendto` system call,
 * providing a slightly cleaner interface for sending raw byte data over UDP.
 * Error handling for `sendto` (e.g., checking return value) is omitted here for
 * brevity but would be crucial in production code.
 * @param sockfd The integer file descriptor representing the active UDP socket
 * to send from.
 * @param addr A constant reference to the `sockaddr_in` structure containing
 * the destination IP address and port number.
 * @param data A pointer (`const uint8_t*`) to the beginning of the byte buffer
 * containing the data to be sent (e.g., the constructed RTP packet).
 * @param len The number of bytes (`size_t`) from the `data` buffer to send.
 */
void send_packet(int sockfd, const sockaddr_in& addr, const uint8_t* data,
                 size_t len) {
  /**
   * @brief Call the `sendto` system function to transmit the UDP datagram.
   * @param sockfd The socket file descriptor.
   * @param data Pointer to the data buffer.
   * @param len Length of the data to send.
   * @param 0 Flags (e.g., MSG_DONTWAIT), 0 for standard blocking behavior.
   * @param reinterpret_cast<const sockaddr*>(&addr) Casts the `sockaddr_in`
   * structure pointer to the generic `sockaddr` pointer required by the API.
   * @param sizeof(addr) The size of the destination address structure.
   * @return On success, `sendto` returns the number of bytes sent. On error, -1
   * is returned, and `errno` is set. (Return value is ignored here).
   */
  ssize_t bytes_sent =
      sendto(sockfd, data, len, 0, reinterpret_cast<const sockaddr*>(&addr),
             sizeof(addr));

  /**
   * @brief Optional: Add error checking for sendto failure.
   */
  if (bytes_sent < 0) {
    // error("sendto failed"); // Add proper error logging if needed
  } else if (static_cast<size_t>(bytes_sent) != len) {
    // error("sendto sent partial packet"); // Add proper error logging if
    // needed
  }
}

/**
 * @brief Determines a safe length for a JPEG fragment to avoid splitting
 * markers, especially restart markers.
 * @details When fragmenting JPEG data for RTP (RFC 2435), it's important not to
 * split certain structures across packet boundaries. This function calculates a
 * suitable length for the next fragment, starting at `start` within the `data`
 * vector, ensuring it does not exceed `max_len`. It specifically tries to
 * adhere to RFC 2435 Section 3.1.7 regarding restart markers (0xD0-0xD7): if a
 * fragment contains a restart marker, it should ideally extend to include all
 * data up to the *next* restart marker, or end immediately after the current
 * one if the next one is too far. It also avoids ending a fragment with a
 * solitary 0xFF byte, which might be misinterpreted as the start of a marker by
 * the receiver if the next packet is lost.
 * @param data A constant reference to the `std::vector<uint8_t>` containing the
 * full JPEG frame data.
 * @param start The starting index (`size_t`) within `data` for the current
 * fragment.
 * @param max_len The maximum permissible length (`size_t`) for this fragment
 * (e.g., `MAX_PAYLOAD`).
 * @return The calculated safe length (`size_t`) for the fragment starting at
 * `start`. This length might be less than `max_len`. Returns 0 if `start` is
 * already at or beyond the end of `data`.
 */
size_t find_safe_fragment_length(const std::vector<uint8_t>& data, size_t start,
                                 size_t max_len) {
  /**
   * @brief Handle edge case where the starting position is already at or past
   * the end of the data.
   */
  if (start >= data.size()) {
    return 0;  // Nothing left to fragment.
  }

  /**
   * @brief Determine the maximum possible length for this fragment.
   * @details It's the minimum of the requested `max_len` and the remaining data
   * size (`data.size() - start`).
   */
  size_t current_max_len = std::min(max_len, data.size() - start);

  /**
   * @brief Initialize the safe length to the maximum possible length calculated
   * above. We will potentially reduce this length based on marker rules.
   */
  size_t safe_len = current_max_len;

  /**
   * @brief Check for restart markers (0xFF 0xD0 to 0xFF 0xD7) within the
   * potential fragment range.
   * @details Iterate from the start of the potential fragment up to one byte
   * before its end, checking pairs of bytes.
   */
  // This restart marker logic seems complex and might not be strictly necessary
  // for all decoders, but attempts to follow RFC guidance. Simpler logic might
  // just avoid ending on 0xFF. Let's keep the original logic for analysis.
  // Iterate up to safe_len - 1 to check pairs data[i], data[i+1]
  for (size_t i = start; i < start + safe_len - 1; ++i) {
    // Ensure index i+1 is valid before accessing data[i+1]
    if (i + 1 >= data.size()) break;

    /**
     * @brief Check if the current pair of bytes forms a restart marker (RSTm).
     */
    if (data[i] == 0xFF && data[i + 1] >= 0xD0 && data[i + 1] <= 0xD7) {
      // Found a restart marker within the potential fragment.
      // RFC suggests including data up to the *next* restart marker.
      size_t next_rst_pos =
          std::string::npos;  // Use npos to indicate not found initially

      /**
       * @brief Search for the *next* restart marker starting after the current
       * one.
       */
      for (size_t j = i + 2; j < data.size() - 1; ++j) {
        if (data[j] == 0xFF && data[j + 1] >= 0xD0 && data[j + 1] <= 0xD7) {
          next_rst_pos = j;  // Found the next restart marker at index j.
          break;             // Stop searching for the next marker.
        }
      }

      /**
       * @brief Decide where to end the fragment based on the next restart
       * marker's position.
       */
      if (next_rst_pos != std::string::npos) {
        // Next restart marker found.
        /**
         * @brief Calculate the length required to include data up to the start
         * of the next marker.
         */
        size_t len_to_next = next_rst_pos - start;
        /**
         * @brief If including up to the next marker fits within the original
         * `max_len`, adjust `safe_len`.
         */
        if (len_to_next <= max_len) {
          safe_len = len_to_next;
        } else {
          /**
           * @brief If the next marker is too far, end the fragment right after
           * the *current* marker.
           */
          safe_len =
              (i + 2) - start;  // Length includes the current marker (2 bytes).
        }
      } else {
        // No more restart markers found after the current one.
        // End the fragment right after the current marker. (The original loop
        // structure implies this if next_rst_pos remains npos, but let's be
        // explicit). safe_len = (i + 2) - start; However, if this is the last
        // marker, the fragment should probably continue to the end of the data
        // or max_len. The original logic seems to break the outer loop,
        // effectively keeping the initial safe_len unless adjusted. Let's
        // refine. If no *next* marker is found, let the fragment extend as far
        // as possible (up to current_max_len), including this last found
        // marker. No change needed to safe_len here if we let the loop finish.
      }

      /**
       * @brief Break the outer loop once the first restart marker within the
       * range is handled. The logic focuses on the first RST encountered.
       */
      break;  // Processed the first restart marker found in the initial range.
    }
  }  // End of restart marker check loop

  /**
   * @brief Final safety check: Avoid ending the fragment with a single 0xFF
   * byte.
   * @details If the last byte of the calculated fragment is 0xFF, it might be
   * the start of a two-byte marker. If the next packet is lost, the receiver
   * might misinterpret this 0xFF. It's safer to shorten the fragment by one
   * byte. Check bounds carefully: `start + safe_len - 1` is the index of the
   * last byte. Need `start + safe_len` to be <= `data.size()`.
   */
  if (safe_len > 0 && (start + safe_len <= data.size()) &&
      data[start + safe_len - 1] == 0xFF) {
    // Ensure we don't shrink to zero length if safe_len was initially 1.
    if (safe_len > 0) {
      safe_len--;
    }
  }

  // The original code had a second check for escaped 0xFF (0xFF 0x00), but
  // ending on 0xFF 0x00 should be safe. The primary concern is ending on an
  // isolated 0xFF that *could* start a marker if the next byte (in the next
  // packet) completes it. The check above handles the problematic case of
  // ending on 0xFF.

  /**
   * @brief Return the calculated safe fragment length.
   */
  return safe_len;
}

/**
 * @brief The core function that continuously streams the loaded frames over
 * RTP.
 * @details This function enters an infinite loop, iterating through the
 * provided `frames` deque. For each frame, it fragments the JPEG data into one
 * or more RTP packets using `find_safe_fragment_length`. It constructs the RTP
 * header and the RFC 2435 JPEG header for each packet, copies the JPEG fragment
 * data, and sends the packet using `send_packet`. It manages the RTP sequence
 * number and timestamp, incrementing them appropriately. Crucially, it uses
 * `std::chrono` and `std::this_thread::sleep_for` to pause between sending
 * complete frames, thereby controlling the output frame rate to match the
 * configured `FRAME_RATE`. The RTP Marker bit is set on the last packet of each
 * frame.
 * @param sockfd The integer file descriptor for the UDP socket used for sending
 * packets.
 * @param addr A constant reference to the `sockaddr_in` structure holding the
 * destination IP and port.
 * @param frames A constant reference to the `std::deque<FrameData>` containing
 * the pre-loaded video frames to be streamed.
 */
void stream(int sockfd, const sockaddr_in& addr,
            const std::deque<FrameData>& frames) {
  /**
   * @brief Define the size in bytes of the standard RTP header.
   */
  const int RTP_HEADER_SIZE = 12;

  /**
   * @brief Define the size in bytes of the RTP JPEG-specific header extension
   * (RFC 2435).
   */
  const int JPEG_HEADER_SIZE = 8;

  /**
   * @brief Calculate the maximum size of the JPEG data payload per RTP packet.
   * @details This aims to keep the total UDP packet size below common network
   * Path MTU (Maximum Transmission Unit) values (often around 1500 bytes) to
   * minimize the risk of IP-level fragmentation, which can be unreliable. We
   * subtract the sizes of the IP header (~20 bytes), UDP header (8 bytes), RTP
   * header (12 bytes), and RTP JPEG header (8 bytes) from a conservative MTU
   * estimate (e.g., 1448 to be safe, original used 1400 calculation base). 1500
   * (MTU) - 20 (IP) - 8 (UDP) = 1472 (Max UDP Payload). 1472 - 12 (RTP) - 8
   * (JPEG) = 1452. Let's use 1400 as a safer general value used in the original
   * code.
   */
  const size_t MAX_PAYLOAD =
      1400;  // Conservative max JPEG data bytes per packet.

  /**
   * @brief Initialize the RTP sequence number. A 16-bit unsigned integer that
   * increments for each packet sent. Starts randomly or at 0.
   */
  uint16_t seq =
      0;  // Start sequence number at 0 (can be randomized for robustness).

  /**
   * @brief Initialize the RTP timestamp. A 32-bit unsigned integer representing
   * the sampling instant of the frame.
   * @details For video, this typically uses a 90 kHz clock (as specified in the
   * SDP). It should increase monotonically. The initial value can be random or
   * zero.
   */
  uint32_t timestamp = 0;  // Start timestamp at 0 (can be randomized).

  /**
   * @brief Calculate the required delay between sending frames to achieve the
   * target `FRAME_RATE`.
   * @details `std::chrono::milliseconds(1000 / FRAME_RATE)` calculates the
   * duration of one frame interval in milliseconds.
   */
  const auto frame_delay = std::chrono::milliseconds(1000 / FRAME_RATE);

  /**
   * @brief Log the start of the streaming process, including the target frame
   * rate.
   */
  log("Streaming started (" + std::to_string(FRAME_RATE) + " fps) to " +
      DEST_IP + ":" + std::to_string(DEST_PORT));

  /**
   * @brief Create a reusable buffer to hold the complete RTP packet (headers +
   * payload) before sending.
   * @details Sized to accommodate the RTP header, JPEG header, and the maximum
   * possible JPEG payload fragment. Using a single reusable buffer can be
   * slightly more efficient than creating a new one for each packet.
   */
  std::vector<uint8_t> packet_buffer(RTP_HEADER_SIZE + JPEG_HEADER_SIZE +
                                     MAX_PAYLOAD);

  /**
   * @brief Generate a Synchronization Source (SSRC) identifier for this RTP
   * stream.
   * @details Defined in RFC 3550, the SSRC is a 32-bit random identifier that
   * uniquely identifies this specific stream source within an RTP session.
   * Receivers use it to group related packets. Using the current time provides
   * a reasonably unique value.
   */
  uint32_t ssrc = static_cast<uint32_t>(
      std::chrono::system_clock::now().time_since_epoch().count());
  log("Using SSRC: " + std::to_string(ssrc));  // Log the SSRC

  /**
   * @brief The main streaming loop. Continuously cycles through the loaded
   * frames.
   * @details This `while(true)` creates an infinite loop. The program needs to
   * be manually stopped (e.g., Ctrl+C).
   */
  while (true) {
    /**
     * @brief Iterate through each `FrameData` object in the `frames` deque.
     * @details This inner loop processes all loaded frames once per cycle of
     * the outer `while(true)` loop.
     */
    for (const auto& frame : frames) {
      /**
       * @brief Record the start time of processing for this frame. Used for
       * frame rate control.
       * @details `std::chrono::steady_clock` is used as it's guaranteed to be
       * monotonic and suitable for measuring intervals.
       */
      auto start_time_frame = std::chrono::steady_clock::now();

      /**
       * @brief Initialize the offset within the current frame's JPEG data.
       * Starts at 0.
       */
      size_t offset = 0;  // Byte offset into frame.jpeg data

      /**
       * @brief Loop to fragment the current frame's JPEG data into RTP packets.
       * @details Continues as long as the `offset` is less than the total size
       * of the JPEG data for this frame.
       */
      while (offset < frame.jpeg.size()) {
        /**
         * @brief Determine the size of the payload (JPEG data fragment) for the
         * current packet.
         * @details Calls `find_safe_fragment_length` to get a size that
         * respects marker boundaries and doesn't exceed `MAX_PAYLOAD` or the
         * remaining data size.
         */
        size_t payload_size = find_safe_fragment_length(
            frame.jpeg,  // Full JPEG data
            offset,      // Current starting offset
            std::min(
                (size_t)MAX_PAYLOAD,
                frame.jpeg.size() - offset)  // Max allowed for this fragment
        );

        /**
         * @brief Safety check: If `find_safe_fragment_length` returns 0, it
         * means no more data can be fragmented (shouldn't happen if offset <
         * frame.jpeg.size(), but defensive check).
         */
        if (payload_size == 0) {
          // This might indicate an issue with find_safe_fragment_length or very
          // small MAX_PAYLOAD
          // log("Warning: payload_size is 0, breaking fragment loop for
          // frame.");
          break;  // Exit the fragmentation loop for this frame.
        }

        /**
         * @brief Optional: Clear the reusable packet buffer before filling it.
         * @details While not strictly necessary if all bytes are overwritten,
         * explicitly filling with 0 can help debug packet contents. Might have
         * a minor performance cost.
         */
        // std::fill(packet_buffer.begin(), packet_buffer.begin() +
        // RTP_HEADER_SIZE + JPEG_HEADER_SIZE + payload_size, 0);

        // --- Construct RTP Header (12 bytes) ---
        // Reference: RFC 3550 Section 5.1

        /**
         * @brief Byte 0: Version (V=2), Padding (P=0), Extension (X=0), CSRC
         * count (CC=0)
         * @details V=2 corresponds to binary 10. P=0, X=0, CC=0000. Result:
         * 10000000 = 0x80.
         */
        packet_buffer[0] = 0x80;

        /**
         * @brief Byte 1: Marker (M), Payload Type (PT)
         * @details M=1 if this is the last packet of the frame, 0 otherwise.
         * Check if `offset + payload_size` reaches the end of the frame data.
         * PT=26 for JPEG video (static payload type from RFC 3551).
         * Marker bit is the most significant bit (0x80). OR it with PT=26
         * (0x1A).
         */
        packet_buffer[1] =
            ((offset + payload_size >= frame.jpeg.size()) ? 0x80 : 0x00) |
            26;  // Set marker bit (0x80) on last packet | PT=26

        /**
         * @brief Bytes 2-3: Sequence Number (16 bits, network byte order)
         * @details Store the current sequence number `seq` in big-endian
         * format.
         */
        packet_buffer[2] = (seq >> 8) & 0xFF;  // High byte of sequence number
        packet_buffer[3] = seq & 0xFF;         // Low byte of sequence number

        /**
         * @brief Bytes 4-7: Timestamp (32 bits, network byte order)
         * @details Store the current frame timestamp `timestamp` in big-endian
         * format using `htonl` (Host TO Network Long). `reinterpret_cast` is
         * used to treat the buffer location as a pointer to uint32_t for direct
         * assignment.
         */
        *reinterpret_cast<uint32_t*>(&packet_buffer[4]) = htonl(timestamp);

        /**
         * @brief Bytes 8-11: Synchronization Source (SSRC) identifier (32 bits,
         * network byte order)
         * @details Store the unique `ssrc` identifier for this stream in
         * big-endian format using `htonl`.
         */
        *reinterpret_cast<uint32_t*>(&packet_buffer[8]) = htonl(ssrc);

        // --- Construct RTP JPEG Header Extension (8 bytes) ---
        // Reference: RFC 2435 Section 3.1

        /**
         * @brief Byte 12: Type-Specific (8 bits) - Used by RFC 2435, but set to
         * 0 here. Relevant for restart markers with DRI header, not used in
         * this basic impl.
         */
        packet_buffer[RTP_HEADER_SIZE + 0] = 0x00;  // Type-specific: Set to 0

        /**
         * @brief Bytes 13-15: Fragment Offset (24 bits, network byte order)
         * @details The byte offset of the start of this fragment within the
         * original JPEG frame data. Stored big-endian.
         */
        packet_buffer[RTP_HEADER_SIZE + 1] =
            (offset >> 16) & 0xFF;  // Fragment Offset High byte
        packet_buffer[RTP_HEADER_SIZE + 2] =
            (offset >> 8) & 0xFF;  // Fragment Offset Middle byte
        packet_buffer[RTP_HEADER_SIZE + 3] =
            offset & 0xFF;  // Fragment Offset Low byte

        /**
         * @brief Byte 16: Type (8 bits) - JPEG color subsampling type.
         * @details Use the `frame.type` value determined during loading (e.g.,
         * 1 for 4:2:0).
         */
        packet_buffer[RTP_HEADER_SIZE + 4] = frame.type;

        /**
         * @brief Byte 17: Quality (Q) (8 bits) - JPEG quality factor.
         * @details Use the `frame.quality` value estimated during loading. RFC
         * 2435 allows values 0-255. Values 1-100 are common estimates. 255 can
         * mean tables not present/unknown.
         */
        packet_buffer[RTP_HEADER_SIZE + 5] = frame.quality;

        /**
         * @brief Byte 18: Width (8 bits) - Frame width divided by 8 pixels.
         * @details RFC 2435 requires width and height in units of 8 pixels. Use
         * ceiling division `(width + 7) / 8`.
         */
        packet_buffer[RTP_HEADER_SIZE + 6] = (frame.width + 7) / 8;

        /**
         * @brief Byte 19: Height (8 bits) - Frame height divided by 8 pixels.
         * @details Similar calculation for height: `(height + 7) / 8`.
         */
        packet_buffer[RTP_HEADER_SIZE + 7] = (frame.height + 7) / 8;

        // --- Copy JPEG Payload ---

        /**
         * @brief Copy the JPEG data fragment (`payload_size` bytes starting
         * from `frame.jpeg.begin() + offset`) into the `packet_buffer`
         * immediately following the RTP and JPEG headers.
         * @details `std::copy_n` copies exactly `payload_size` elements.
         * The destination starts at `packet_buffer.begin() + RTP_HEADER_SIZE +
         * JPEG_HEADER_SIZE`.
         */
        std::copy_n(frame.jpeg.begin() + offset,  // Source iterator
                    payload_size,                 // Number of bytes to copy
                    packet_buffer.begin() + RTP_HEADER_SIZE +
                        JPEG_HEADER_SIZE);  // Destination iterator

        /**
         * @brief Calculate the total size of the packet to be sent (headers +
         * payload).
         */
        size_t total_packet_size =
            RTP_HEADER_SIZE + JPEG_HEADER_SIZE + payload_size;

        /**
         * @brief Send the fully constructed RTP packet over the UDP socket.
         * @details Calls the `send_packet` helper function.
         */
        send_packet(
            sockfd,                // Socket descriptor
            addr,                  // Destination address structure
            packet_buffer.data(),  // Pointer to the packet data in the buffer
            total_packet_size);    // Total size of the packet to send

        /**
         * @brief Increment the RTP sequence number for the next packet. Wraps
         * around automatically from 65535 to 0.
         */
        seq++;

        /**
         * @brief Advance the offset within the current JPEG frame's data to the
         * start of the next fragment.
         */
        offset += payload_size;

        /**
         * @brief Optional: Introduce a very small delay between sending
         * fragments of the *same* frame.
         * @details This can sometimes help prevent overwhelming a slow receiver
         * or network buffers when a single large frame is split into many
         * packets sent in rapid succession. Not strictly required by RTP but
         * can be a practical measure.
         */
        if (offset < frame.jpeg.size()) {  // Only sleep if more fragments
                                           // remain for this frame
          std::this_thread::sleep_for(std::chrono::microseconds(
              100));  // Small delay (e.g., 100 microseconds)
        }

      }  // End of fragmentation loop (while offset < frame.jpeg.size())

      /**
       * @brief Calculate the time elapsed since the start of processing this
       * frame.
       */
      auto elapsed_time = std::chrono::steady_clock::now() - start_time_frame;

      /**
       * @brief Calculate the remaining time needed to wait to maintain the
       * target frame rate.
       * @details Subtract the `elapsed_time` from the desired `frame_delay`.
       */
      auto sleep_time = frame_delay - elapsed_time;

      /**
       * @brief If `sleep_time` is positive, pause execution for that duration.
       * @details This ensures that frames are sent out at intervals matching
       * the `FRAME_RATE`. If processing and sending took longer than
       * `frame_delay`, `sleep_time` will be negative or zero, and no sleep
       * occurs (the stream might lag slightly).
       */
      if (sleep_time > std::chrono::milliseconds(0)) {
        std::this_thread::sleep_for(sleep_time);
      }

      /**
       * @brief Increment the RTP timestamp for the *next* frame.
       * @details The timestamp should represent the sampling instant. For
       * constant frame rate video, increment by the fixed amount corresponding
       * to the 90 kHz clock. Timestamp increment = Clock Rate / Frame Rate =
       * 90000 / FRAME_RATE.
       */
      timestamp += (90000 / FRAME_RATE);

    }  // End of loop iterating through frames deque
  }  // End of infinite streaming loop (while true)
}

// ========================================================================== //
// ==                          MAIN APPLICATION ENTRY POINT                == //
// ========================================================================== //

/**
 * @brief The main entry point of the JPEG RTP streaming application.
 * @details This function orchestrates the entire process:
 * 1. Creates a UDP socket for network communication.
 * 2. Sets socket options (like buffer size).
 * 3. Initializes the destination network address structure.
 * 4. Calls `load_frames` to load JPEG images from disk into memory.
 * 5. Calls `create_sdp` to generate the SDP file based on loaded frame
 * dimensions.
 * 6. Prints instructions for playback (e.g., using VLC).
 * 7. Calls the `stream` function, which enters an infinite loop to send the
 * frames over RTP.
 * 8. Includes basic error handling for critical setup steps (socket, frame
 * loading, SDP).
 * @param argc The number of command-line arguments (unused).
 * @param argv An array of command-line argument strings (unused).
 * @return Returns 0 (`EXIT_SUCCESS`) if setup completes successfully before
 * entering the infinite stream loop. Returns 1 (`EXIT_FAILURE`) if a critical
 * error occurs during setup (socket creation, frame loading failure, SDP
 * creation failure). Note that successful execution implies entering an
 * infinite loop, so 0 is never normally returned from the end of `main`.
 */
int main() {
  /**
   * @brief Create a UDP socket using the `socket` system call.
   * @param AF_INET Specifies the address family (IPv4).
   * @param SOCK_DGRAM Specifies the socket type (UDP - datagram-based,
   * connectionless).
   * @param 0 Specifies the protocol (0 typically defaults to the correct
   * protocol for the type, which is UDP here).
   * @return Returns a non-negative integer file descriptor representing the
   * socket on success, or -1 on failure.
   */
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  /**
   * @brief Check if socket creation failed.
   */
  if (sockfd < 0) {
    error("Socket creation failed");  // Report error using the helper function.
    return 1;                         // Exit program with failure code.
  }
  log("UDP socket created (fd=" + std::to_string(sockfd) + ").");

  /**
   * @brief Set the socket's send buffer size using `setsockopt`.
   * @details Increasing the send buffer size can potentially improve
   * performance for high-bandwidth streaming by reducing the likelihood of
   * `sendto` blocking or dropping packets due to a full kernel buffer.
   * @param sockfd The socket file descriptor.
   * @param SOL_SOCKET Specifies that the option is at the general socket level.
   * @param SO_SNDBUF Specifies the option to set the send buffer size.
   * @param &buffer_size Pointer to the integer variable holding the desired
   * buffer size.
   * @param sizeof(buffer_size) The size of the buffer size variable.
   * @return Returns 0 on success, -1 on failure.
   */
  int buffer_size = 1 * 1024 * 1024;  // Request a 1 MB send buffer.
  if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &buffer_size,
                 sizeof(buffer_size)) < 0) {
    // Report error but don't exit, as it's not always critical. The system
    // default will be used.
    error("Failed to set socket send buffer size (continuing with default)");
  } else {
    // Optional: Get the actual buffer size set, as the OS might adjust it.
    int actual_size = 0;
    socklen_t optlen = sizeof(actual_size);
    getsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &actual_size, &optlen);
    log("Socket send buffer size set (requested=" +
        std::to_string(buffer_size) +
        ", actual=" + std::to_string(actual_size) + ").");
  }

  /**
   * @brief Initialize the destination address structure (`sockaddr_in`).
   * @details This structure holds the target IP address and port for sending
   * UDP packets.
   */
  sockaddr_in dest_addr{};  // Use brace initialization to zero-fill the
                            // structure initially.

  /**
   * @brief Set the address family to IPv4.
   */
  dest_addr.sin_family = AF_INET;

  /**
   * @brief Set the destination port number.
   * @details `htons` (Host TO Network Short) converts the integer port number
   * (`DEST_PORT`) from the host's byte order to network byte order
   * (big-endian), which is required by network protocols.
   */
  dest_addr.sin_port = htons(DEST_PORT);

  /**
   * @brief Set the destination IP address.
   * @details `inet_pton` (Presentation TO Network) converts the human-readable
   * IP address string (`DEST_IP.c_str()`) into its binary network format and
   * stores it in `dest_addr.sin_addr`.
   * @param AF_INET The address family (IPv4).
   * @param DEST_IP.c_str() The source IP address string.
   * @param &dest_addr.sin_addr Pointer to the destination address field within
   * the structure.
   * @return Returns 1 on success, 0 if the string is invalid, -1 on error.
   * (Error check omitted for brevity).
   */
  inet_pton(AF_INET, DEST_IP.c_str(), &dest_addr.sin_addr);

  /**
   * @brief Declare a deque to hold the loaded frame data.
   */
  std::deque<FrameData> frames;

  /**
   * @brief Call the `load_frames` function to read JPEGs from disk.
   * @details Check the return value. If loading fails or results in an empty
   * deque (no frames loaded), clean up the socket and exit.
   */
  if (!load_frames(frames) || frames.empty()) {
    log("Frame loading failed or resulted in zero frames.");
    close(sockfd);  // Close the socket before exiting.
    return 1;       // Exit program with failure code.
  }

  /**
   * @brief Call the `create_sdp` function to generate the SDP file.
   * @details Uses the dimensions from the *first* loaded frame (`frames[0]`)
   * assuming all frames have the same dimensions. Check the return value. If
   * SDP creation fails, clean up the socket and exit.
   */
  if (!create_sdp(frames[0].width, frames[0].height)) {
    log("SDP file creation failed.");
    close(sockfd);  // Close the socket before exiting.
    return 1;       // Exit program with failure code.
  }
  log("SDP file created: " + SDP_FILENAME);

  /**
   * @brief Log instructions for the user on how to play the stream using VLC.
   * @details Suggests opening the generated SDP file with VLC. The
   * `--network-caching` option can help smooth playback over potentially
   * jittery networks.
   */
  log("Stream ready. Play with a media player like VLC:");
  log("\tvlc " + SDP_FILENAME +
      " --network-caching=150");  // Suggest slightly more caching

  /**
   * @brief Call the `stream` function to begin the main RTP streaming loop.
   * @details This function will run indefinitely, sending the loaded frames.
   * Control will not return from this call under normal circumstances.
   */
  stream(sockfd, dest_addr, frames);

  /**
   * @brief This part of the code is typically unreachable because `stream()`
   * contains an infinite loop.
   * @details If the `stream` function were modified to eventually return, this
   * cleanup code would execute.
   */
  log("Streaming finished (unexpected exit from stream function).");
  close(sockfd);  // Close the socket resource.
  return 0;       // Indicate successful execution (if stream loop could exit
                  // cleanly).
}

