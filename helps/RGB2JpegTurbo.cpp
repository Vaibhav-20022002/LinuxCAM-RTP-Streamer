/**
 * @file RGB2JpegTurbo.cpp
 * @author Vaibhav
 * @brief Resource-efficient, batch-processing RGB to JPEG converter, optimized
 * with considerations for platforms like Raspberry Pi Zero 2W.
 *
 * @details
 * This program converts a sequence of raw RGB24 image files (expected format:
 * "frame_NUMBER.rgb") found in an input directory into JPEG files in an output
 * directory. It is designed with resource efficiency in mind, particularly for
 * embedded systems or low-power devices, employing several optimization
 * strategies:
 *
 * - **Batch Processing:** Instead of reading one RGB file, converting it, and
 * writing one JPEG file immediately, this implementation first scans the input
 * directory to identify all valid RGB frames. It then loads the data for *all*
 * valid frames into memory. Subsequently, it converts frames and buffers the
 * resulting JPEG data in memory before writing them to disk in larger batches
 * (`write_jpeg_batch`). This reduces the number of disk write operations and
 * potentially improves throughput by leveraging filesystem caching and reducing
 * I/O overhead.
 * - **libjpeg-turbo Utilization:** Leverages the high-performance libjpeg-turbo
 * library for the core JPEG compression (`tjCompress2`). Libjpeg-turbo is known
 * for its speed and includes SIMD optimizations (like ARM NEON) internally,
 * which are automatically used if available on the target platform,
 * accelerating the computationally intensive compression process without
 * explicit NEON code here.
 * - **Manual Memory Management:** Uses C-style memory allocation (`malloc`,
 * `free`, `strdup`) for frame data and paths. This provides fine-grained
 * control but requires careful management to prevent memory leaks. RGB data is
 * freed immediately after compression to minimize peak memory usage during the
 * conversion phase. JPEG buffers allocated by `tjCompress2` are freed using
 * `tjFree` after being written to disk.
 * - **Pre-computation/Loading:** The `load_rgb_frames` function performs a
 * two-pass process: first identifying and validating all frame metadata, then
 * loading the actual pixel data. This separates I/O from processing and ensures
 * frames are processed sequentially (after sorting).
 * - **Simplified Error Handling:** Primarily uses return codes and prints error
 * messages to `std::cerr` instead of relying on C++ exceptions, which can
 * introduce overhead.
 * - **Targeted Data Structures:** Uses simple `struct`s (`FrameInfo`,
 * `JpegOutput`) with raw pointers for data storage, minimizing object overhead.
 *
 * **Workflow:**
 * 1. Parse command-line arguments (input/output directories, width, height,
 * quality).
 * 2. Validate input parameters.
 * 3. Create the output directory if it doesn't exist.
 * 4. Call `load_rgb_frames`:
 * a. Scan the input directory.
 * b. Validate filenames ("frame_NUMBER.rgb") and file types.
 * c. Allocate memory for metadata and RGB data buffers for all valid frames.
 * d. Sort the frame list by frame number.
 * e. Read the actual RGB pixel data into the allocated buffers.
 * 5. Call `process_frames`:
 * a. Iterate through the loaded `FrameInfo` structures.
 * b. For each valid frame, call `rgb_to_yuv420_jpeg` (which uses `tjCompress2`)
 * to convert RGB data to a compressed JPEG buffer (using YUV 4:2:0
 * subsampling). c. Free the source RGB data buffer immediately after
 * compression. d. Store the resulting JPEG buffer pointer, size, and frame
 * number in a `JpegOutput` structure. e. Add the `JpegOutput` to a temporary
 * batch (`jpeg_outputs` vector). f. If the batch buffer reaches
 * `MAX_JPEG_BUFFER` size, call `write_jpeg_batch`. g. Free the frame's
 * `file_path` memory.
 * 6. After processing all frames, call `write_jpeg_batch` one last time to
 * write any remaining JPEGs in the buffer.
 * 7. `write_jpeg_batch`:
 * a. Iterates through the `JpegOutput` structures in the batch.
 * b. Constructs the output JPEG filename ("jpeg_NUMBER.jpg").
 * c. Opens the output file using standard C file I/O (`fopen`, `fwrite`,
 * `fclose`). d. Writes the compressed JPEG data to the file. e. Frees the JPEG
 * data buffer using `tjFree`. f. Clears the batch vector.
 *
 * @param input_dir Path to the directory containing input RGB24 frame files
 * (e.g., frame_0.rgb, frame_1.rgb...).
 * @param output_dir Path to the directory where output JPEG files will be saved
 * (e.g., jpeg_0.jpg, jpeg_1.jpg...). Will be created if it doesn't exist.
 * @param width The width of the input RGB frames in pixels. Must be consistent
 * for all frames.
 * @param height The height of the input RGB frames in pixels. Must be
 * consistent for all frames.
 * @param quality The desired JPEG quality factor (integer 1-100). Higher values
 * mean better quality and larger file sizes. Defaults to 75 if not provided.
 *
 * @note This code assumes input files are raw RGB24 pixel data (3 bytes per
 * pixel, R-G-B order) with no header, matching the specified width and height
 * exactly.
 */

// ========================================================================== //
// ==                            INCLUDE DIRECTIVES                        == //
// ========================================================================== //

#include <dirent.h>  // Provides definitions for directory stream operations ('opendir', 'readdir', 'closedir') and the 'dirent' structure. Needed for scanning the input directory.
#include <fcntl.h>  // Provides file control options (often used with 'open', though not directly used here as standard I/O is preferred). Included perhaps for historical reasons or POSIX compatibility.
#include <sys/stat.h>  // Provides functions for retrieving file status information ('stat') and related structures/macros ('struct stat', 'S_ISREG', 'mkdir'). Needed for validating files and creating the output directory.
#include <turbojpeg.h>  // Main header file for the libjpeg-turbo library API. Provides functions for JPEG compression ('tjInitCompress', 'tjCompress2', 'tjDestroy') and memory management ('tjFree').
#include <unistd.h>  // Provides access to the POSIX operating system API, including various constants and types. Implicitly used by other headers, potentially for filesystem interactions.

#include <algorithm>  // Provides standard algorithms like 'std::sort'. Used here to sort frames by number.
#include <cstring>  // Provides C-style string manipulation functions ('strcmp', 'strncmp', 'strrchr', 'strdup', 'strlen' implicitly via others). Used for filename parsing and path manipulation.
#include <iostream>  // Provides standard input/output stream objects ('std::cout', 'std::cerr'). Used for logging progress and error messages.
#include <memory>  // Provides smart pointer classes (like 'std::unique_ptr'). Although not directly used for core data buffers here (manual management is used), it's good practice to include if needed elsewhere.
#include <string>  // Provides the 'std::string' class. While C-style strings (`char*`) are used primarily for performance/control reasons here, `std::string` might be used internally or for temporary conversions.
#include <vector>  // Provides the 'std::vector' dynamic array container. Used to store the list of `FrameInfo` structures and the batch of `JpegOutput` structures.

// Conditional inclusion for ARM NEON intrinsics header.
// Note: While included, the explicit NEON conversion logic is removed in this
// version, as libjpeg-turbo handles SIMD optimizations internally. This block
// remains potentially for reference or future re-introduction of custom NEON
// code if needed.
#ifdef __ARM_NEON
#include <arm_neon.h>  // Provides ARM NEON SIMD intrinsics if compiling on an ARM platform with NEON support enabled.
#endif

// ========================================================================== //
// ==                                MACROS                                == //
// ========================================================================== //

/**
 * @brief Defines the maximum number of compressed JPEG outputs to buffer in
 * memory before triggering a batch write to disk.
 * @details A larger buffer reduces the frequency of disk write operations but
 * increases peak memory usage (holding multiple compressed JPEGs). This value
 * balances I/O efficiency and memory consumption.
 */
#define MAX_JPEG_BUFFER 32

// ========================================================================== //
// ==                              DATA STRUCTURES                         == //
// ========================================================================== //

/**
 * @brief Structure to hold metadata and data pointer for a single input RGB
 * frame.
 * @details Designed for efficiency, using raw pointers for data that needs
 * explicit memory management (allocation and deallocation). This avoids
 * overhead associated with standard containers like std::vector for the core
 * pixel data, potentially beneficial on resource-constrained systems.
 */
struct FrameInfo {
  /**
   * @brief Pointer to a C-style string holding the full path to the original
   * RGB frame file.
   * @details Memory for this path string is allocated using `strdup` and **must
   * be freed** by the owner of the `FrameInfo` structure (typically after the
   * frame is processed) using `free()`.
   */
  char* file_path;

  /**
   * @brief The width of the frame in pixels.
   * @details Obtained from command-line arguments, assumed constant for all
   * frames.
   */
  int width;

  /**
   * @brief The height of the frame in pixels.
   * @details Obtained from command-line arguments, assumed constant for all
   * frames.
   */
  int height;

  /**
   * @brief The sequential frame number extracted from the filename (e.g., 123
   * from "frame_123.rgb").
   * @details Used for sorting frames and generating output filenames.
   */
  int frame_number;

  /**
   * @brief Pointer to the buffer containing the raw RGB24 pixel data for this
   * frame.
   * @details Memory for this buffer is allocated using `malloc` and **must be
   * freed** by the owner of the `FrameInfo` structure using `free()`. It's
   * typically freed immediately after the RGB data has been compressed into
   * JPEG format to save memory. Can be NULL if the frame failed to load.
   */
  unsigned char* rgb_data;

  /**
   * @brief The expected size of the `rgb_data` buffer in bytes (width * height
   * * 3).
   * @details Stored for convenience and potential validation.
   */
  size_t rgb_size;
};

/**
 * @brief Structure to hold the compressed JPEG data and related information for
 * batch writing.
 * @details Aggregates the output of the JPEG compression process before it's
 * written to disk. Using a structure helps manage the JPEG buffer pointer and
 * its size together.
 */
struct JpegOutput {
  /**
   * @brief Pointer to the buffer containing the compressed JPEG data.
   * @details This buffer is allocated *internally* by libjpeg-turbo
   * (`tjCompress2`) and
   * **must be freed** using the specific libjpeg-turbo function `tjFree()`
   * after the data has been written to disk (or is no longer needed). Can be
   * NULL if compression failed.
   */
  unsigned char* jpeg_data;

  /**
   * @brief The size of the compressed JPEG data in the `jpeg_data` buffer, in
   * bytes.
   * @details This value is output by `tjCompress2`.
   */
  unsigned long jpeg_size;

  /**
   * @brief The original frame number corresponding to this JPEG data.
   * @details Used to construct the output JPEG filename (e.g.,
   * "jpeg_NUMBER.jpg").
   */
  int frame_number;
};

// ========================================================================== //
// ==                       HELPER FUNCTION DEFINITIONS                    == //
// ========================================================================== //

/**
 * @brief Validates if a given path points to a potential RGB frame file based
 * on filename pattern.
 * @details Performs lightweight checks suitable for embedded environments:
 * 1. Uses `stat` to verify the path exists and refers to a regular file (not a
 * directory, link, etc.).
 * 2. Extracts the filename part from the full path.
 * 3. Checks if the filename starts with the expected prefix "frame_".
 * 4. Attempts to parse an integer (frame number) following the prefix.
 * 5. Checks if the remaining part of the filename is exactly ".rgb".
 * It avoids C++ exceptions and returns simple integer codes for different
 * failure reasons.
 * @param path A C-style string (`const char*`) representing the full path to
 * the file to validate.
 * @param frame_num A pointer to an integer (`int*`) where the extracted frame
 * number will be stored if validation succeeds. The value is undefined on
 * failure.
 * @return `int` Return code:
 * - 0: Success. Path is a regular file matching the pattern "frame_NUMBER.rgb",
 * and `frame_num` is populated.
 * - 1: Stat failed or path is not a regular file.
 * - 2: Filename does not start with "frame_".
 * - 3: Failed to parse a valid frame number after "frame_", or the extension is
 * not ".rgb".
 */
int validate_rgb_filename(const char* path, int* frame_num) {
  /**
   * @brief Declare a 'stat' structure to hold file status information.
   */
  struct stat path_stat;

  /**
   * @brief Call 'stat' system call to get file information. Check for errors
   * and if it's a regular file.
   * @details `stat(path, &path_stat)` fills the `path_stat` structure. It
   * returns 0 on success, -1 on error. `S_ISREG(path_stat.st_mode)` checks the
   * file mode bits to see if it's a regular file.
   */
  if (stat(path, &path_stat) != 0 || !S_ISREG(path_stat.st_mode)) {
    return 1;  // Error code 1: Not a regular file or stat failed.
  }

  /**
   * @brief Extract the filename portion from the full path.
   * @details `strrchr(path, '/')` finds the last occurrence of '/' in the path.
   * If '/' is found, the filename starts one character after it (`filename +
   * 1`). If '/' is not found (path is just a filename), `strrchr` returns NULL,
   * so we use the original `path`.
   */
  const char* filename = strrchr(path, '/');
  filename =
      filename ? filename + 1
               : path;  // Point `filename` to the actual start of the filename.

  /**
   * @brief Check if the filename starts with the required prefix "frame_".
   * @details `strncmp` compares the first 6 characters of `filename` with
   * "frame_". It returns 0 if they match.
   */
  if (strncmp(filename, "frame_", 6) != 0) {
    return 2;  // Error code 2: Incorrect filename prefix.
  }

  /**
   * @brief Attempt to parse the frame number from the string following
   * "frame_".
   * @details `strtol` (String TO Long) converts a string to a long integer.
   * `filename + 6` points to the character immediately after "frame_".
   * `&endptr` is a pointer that `strtol` will set to point to the first
   * character *after* the parsed number. `10` specifies base 10 for the
   * conversion.
   */
  char* endptr;  // Will point to character after the number.
  *frame_num =
      strtol(filename + 6, &endptr, 10);  // Store result in *frame_num.

  /**
   * @brief Validate the parsed number and the remaining part of the filename.
   * @details Check if:
   * - The parsed frame number is within a reasonable range (e.g., 0 to 9999).
   * `strtol` might return 0 on failure, so check range.
   * - The character pointed to by `endptr` (the first non-digit character after
   * the number) marks the beginning of the expected ".rgb" extension.
   * `strcmp(endptr, ".rgb")` compares the rest of the string with ".rgb". It
   * returns 0 if they match.
   */
  if (*frame_num < 0 || *frame_num > 9999 || strcmp(endptr, ".rgb") != 0) {
    // Handle potential strtol errors where no digits are found (returns 0,
    // endptr==filename+6) - the strcmp will likely catch this.
    return 3;  // Error code 3: Invalid number format/range or incorrect file
               // extension.
  }

  /**
   * @brief If all checks pass, return 0 for success.
   */
  return 0;  // Success.
}

/**
 * @brief Reads raw RGB pixel data from a file into a pre-allocated buffer.
 * @details This function reads the entire content of the specified RGB file. It
 * validates that the file size exactly matches the expected size calculated
 * from the provided width and height (width * height * 3 bytes for RGB24). It
 * uses standard C file I/O (`fopen`, `fseek`, `ftell`, `fread`, `fclose`) for
 * portability and simplicity, rather than potentially less portable or more
 * complex direct I/O methods.
 * @param file_path C-style string (`const char*`) path to the RGB frame file to
 * read.
 * @param width The expected width (`int`) of the frame in pixels.
 * @param height The expected height (`int`) of the frame in pixels.
 * @param buffer A pointer (`unsigned char*`) to a pre-allocated memory buffer
 * large enough to hold the entire RGB frame data (`width * height * 3` bytes).
 * The caller is responsible for allocating and freeing this buffer.
 * @return `int` Return code:
 * - 0: Success. The buffer is filled with the RGB data.
 * - 1: File could not be opened for reading.
 * - 2: File size does not match the expected size (width * height * 3).
 * - 3: Error occurred while reading data from the file (e.g., read fewer bytes
 * than expected).
 */
int read_rgb_frame(const char* file_path, int width, int height,
                   unsigned char* buffer) {
  /**
   * @brief Calculate the expected size of the RGB data in bytes (Width * Height
   * * 3 bytes/pixel).
   */
  const size_t expected_size =
      static_cast<size_t>(width) * height * 3;  // Use size_t for calculation

  /**
   * @brief Open the file in binary read mode ("rb").
   * @details Returns a FILE pointer on success, NULL on failure.
   */
  FILE* fp = fopen(file_path, "rb");
  if (!fp) {
    // Use std::cerr for error reporting.
    std::cerr << "ERROR: Cannot open RGB file for reading: " << file_path
              << " (" << strerror(errno) << ")" << std::endl;
    return 1;  // Error code 1: File open error.
  }

  /**
   * @brief Seek to the end of the file to determine its size.
   * @details `fseek(fp, 0, SEEK_END)` moves the file position indicator to the
   * end.
   */
  if (fseek(fp, 0, SEEK_END) != 0) {
    std::cerr << "ERROR: Failed to seek to end of file: " << file_path << " ("
              << strerror(errno) << ")" << std::endl;
    fclose(fp);
    return 2;  // Treat seek error as a size check failure? Or new code? Let's
               // use 2.
  }

  /**
   * @brief Get the current file position, which is the file size since we are
   * at the end.
   * @details `ftell` returns the position as a `long`. Check for errors (-1L).
   */
  long file_size = ftell(fp);
  if (file_size < 0) {
    std::cerr << "ERROR: Failed to get file size (ftell): " << file_path << " ("
              << strerror(errno) << ")" << std::endl;
    fclose(fp);
    return 2;  // Size check failure.
  }

  /**
   * @brief Compare the actual file size with the expected size.
   */
  if (static_cast<size_t>(file_size) != expected_size) {
    std::cerr << "ERROR: File size mismatch for " << file_path << ": expected "
              << expected_size << " bytes, but got " << file_size << " bytes."
              << std::endl;
    fclose(fp);  // Close the file before returning.
    return 2;    // Error code 2: Size mismatch.
  }

  /**
   * @brief Reset the file position indicator back to the beginning of the file.
   * @details `fseek(fp, 0, SEEK_SET)` moves to the start (offset 0 from the
   * beginning). Check for errors.
   */
  if (fseek(fp, 0, SEEK_SET) != 0) {
    std::cerr << "ERROR: Failed to seek to start of file: " << file_path << " ("
              << strerror(errno) << ")" << std::endl;
    fclose(fp);
    return 3;  // Treat as read error? Or new code? Let's use 3.
  }

  /**
   * @brief Read the entire file content into the provided buffer.
   * @details `fread(buffer, 1, expected_size, fp)` attempts to read
   * `expected_size` items of size 1 byte from `fp` into `buffer`. It returns
   * the number of items successfully read.
   */
  size_t bytes_read = fread(buffer, 1, expected_size, fp);

  /**
   * @brief Close the file stream. Always important to do this, even if reading
   * failed.
   */
  fclose(fp);

  /**
   * @brief Check if the number of bytes read matches the expected size.
   */
  if (bytes_read != expected_size) {
    std::cerr << "ERROR: Failed to read expected " << expected_size
              << " bytes from " << file_path << ". Actually read " << bytes_read
              << " bytes." << std::endl;
    // Check ferror(fp) or feof(fp) before fclose could give more info, but
    // generally bytes_read mismatch is sufficient.
    return 3;  // Error code 3: Read error.
  }

  /**
   * @brief If all steps succeeded, return 0.
   */
  return 0;  // Success.
}

/**
 * @brief Compresses raw RGB pixel data into a JPEG image using libjpeg-turbo.
 * @details This function initializes a libjpeg-turbo compression instance,
 * performs the compression from RGB format to JPEG format with YUV 4:2:0 chroma
 * subsampling, and returns a buffer containing the compressed JPEG data. It
 * relies on libjpeg-turbo's internal optimizations (including potential NEON
 * usage on ARM platforms) via the `tjCompress2` function.
 * @param rgb_data A pointer (`const unsigned char*`) to the buffer containing
 * the input raw RGB24 pixel data.
 * @param width The width (`int`) of the input image in pixels.
 * @param height The height (`int`) of the input image in pixels.
 * @param quality The desired JPEG quality factor (`int`, 1-100).
 * @param jpeg_size A pointer (`unsigned long*`) where the size (in bytes) of
 * the resulting compressed JPEG data buffer will be stored.
 * @return `unsigned char*` A pointer to a memory buffer containing the
 * compressed JPEG data. This buffer is allocated by libjpeg-turbo and **must be
 * freed by the caller using `tjFree()`**. Returns `NULL` if initialization or
 * compression fails. `*jpeg_size` will be 0 in case of failure.
 */
unsigned char* rgb_to_yuv420_jpeg(const unsigned char* rgb_data, int width,
                                  int height, int quality,
                                  unsigned long* jpeg_size) {
  /**
   * @brief Initialize a TurboJPEG compressor instance.
   * @details `tjInitCompress()` allocates and returns a handle to a compressor
   * object. Returns NULL on failure.
   */
  tjhandle handle = tjInitCompress();
  if (!handle) {
    // Get descriptive error message from libjpeg-turbo.
    std::cerr << "ERROR: TurboJPEG compressor initialization failed: "
              << tjGetErrorStr() << std::endl;
    *jpeg_size = 0;  // Ensure size is 0 on failure.
    return NULL;     // Return NULL to indicate failure.
  }

  /**
   * @brief Declare a pointer for the output JPEG buffer, initialized to NULL.
   * @details libjpeg-turbo will allocate this buffer via `tjCompress2`.
   */
  unsigned char* jpeg_buf = NULL;

  /**
   * @brief Initialize the output JPEG size to 0.
   */
  *jpeg_size = 0;

  /**
   * @brief Set compression flags.
   * @details `TJFLAG_FASTDCT` enables a faster, less accurate DCT/IDCT
   * algorithm, often suitable for real-time encoding or resource-constrained
   * devices. The original TJFLAG_NOREALLOC is removed as it requires
   * pre-calculating the exact output buffer size, which is complex and
   * error-prone. Letting TurboJPEG allocate the buffer is generally safer and
   * simpler.
   */
  int flags = TJFLAG_FASTDCT;

  /**
   * @brief Perform the JPEG compression using `tjCompress2`.
   * @details This is the core compression function.
   * - `handle`: The compressor instance handle.
   * - `rgb_data`: Pointer to the input RGB pixel buffer.
   * - `width`: Width of the input image.
   * - `width * 3`: Pitch (stride) of the input image in bytes (bytes per line).
   * For tightly packed RGB24, it's width * 3.
   * - `height`: Height of the input image.
   * - `TJPF_RGB`: Pixel format of the input image (RGB). Other formats like
   * BGR, RGBA exist (TJPF_BGR, TJPF_RGBX).
   * - `&jpeg_buf`: Address of the pointer where TurboJPEG will store the
   * address of the allocated output buffer.
   * - `jpeg_size`: Pointer to the variable where TurboJPEG will store the size
   * of the compressed output buffer.
   * - `TJSAMP_420`: Desired JPEG chroma subsampling format (YUV 4:2:0). This is
   * common for video and offers good compression. Other options include
   * TJSAMP_444, TJSAMP_422, TJSAMP_GRAY.
   * - `quality`: Desired JPEG quality (1-100).
   * - `flags`: Compression flags (e.g., TJFLAG_FASTDCT).
   * @return 0 on success, -1 on error.
   */
  int result = tjCompress2(handle, rgb_data, width, width * 3, height, TJPF_RGB,
                           &jpeg_buf, jpeg_size, TJSAMP_420, quality, flags);

  /**
   * @brief Check if compression failed.
   */
  if (result != 0) {
    std::cerr << "ERROR: TurboJPEG compression failed: " << tjGetErrorStr()
              << std::endl;
    // Even if compression fails, TurboJPEG might have allocated jpeg_buf, so
    // try to free it.
    tjFree(jpeg_buf);  // Free potentially allocated buffer.
    jpeg_buf = NULL;   // Set pointer back to NULL.
    *jpeg_size = 0;    // Ensure size is 0.
    // Fall through to destroy handle and return NULL.
  }

  /**
   * @brief Destroy the TurboJPEG compressor instance to free associated
   * resources.
   * @details Should be called regardless of compression success or failure once
   * the handle is no longer needed.
   * @return 0 on success, -1 on error. (Error check omitted).
   */
  tjDestroy(handle);

  /**
   * @brief Return the pointer to the compressed JPEG data buffer (or NULL if
   * failed).
   * @details The caller is now responsible for freeing this buffer using
   * `tjFree()`.
   */
  return jpeg_buf;
}

/**
 * @brief Writes a batch of compressed JPEG data buffers to disk files.
 * @details Iterates through a vector of `JpegOutput` structures. For each valid
 * entry (non-NULL data, non-zero size), it constructs an output filename
 * ("jpeg_NUMBER.jpg"), opens the file using standard C I/O, writes the JPEG
 * data, and closes the file. After attempting to write, it frees the JPEG data
 * buffer using `tjFree()`. It counts and returns the number of files
 * successfully written. Error messages are printed for failures, but the
 * function continues processing the rest of the batch. The input vector
 * `jpeg_outputs` is cleared at the end.
 * @param output_dir C-style string (`const char*`) path to the directory where
 * JPEG files should be saved.
 * @param jpeg_outputs A reference to a `std::vector<JpegOutput>` containing the
 * buffered JPEGs to be written. This vector will be cleared by the function.
 * @return `int` The number of JPEG files that were successfully written to disk
 * in this batch.
 */
int write_jpeg_batch(const char* output_dir,
                     std::vector<JpegOutput>& jpeg_outputs) {
  /**
   * @brief Buffer to hold the constructed output path string. Size 512 should
   * be sufficient for typical paths.
   */
  char output_path[512];

  /**
   * @brief Counter for successfully written files in this batch.
   */
  int success_count = 0;

  /**
   * @brief Iterate through the vector of JpegOutput structures using a
   * range-based for loop.
   */
  for (const auto& jpeg : jpeg_outputs) {
    /**
     * @brief Skip entries that represent failed compressions or are otherwise
     * invalid.
     */
    if (!jpeg.jpeg_data || jpeg.jpeg_size == 0) {
      // If jpeg.jpeg_data is non-NULL but size is 0, it should still be freed.
      // However, tjCompress2 likely sets size>0 on success with non-NULL
      // buffer. Let's assume if data is NULL, size is also effectively 0. If
      // data is non-NULL, we *must* free it later, even if size is 0 (though
      // unlikely). The free logic is outside this check.
      continue;  // Skip to the next JPEG in the batch.
    }

    /**
     * @brief Construct the output filename using the frame number.
     * @details `snprintf` safely formats the string into `output_path`,
     * preventing buffer overflows. Format:
     * "<output_dir>/jpeg_<frame_number>.jpg"
     */
    snprintf(output_path, sizeof(output_path), "%s/jpeg_%d.jpg", output_dir,
             jpeg.frame_number);

    /**
     * @brief Open the output file in binary write mode ("wb").
     * @details Returns a FILE pointer on success, NULL on failure.
     */
    FILE* fp = fopen(output_path, "wb");
    if (!fp) {
      std::cerr << "ERROR: Failed to create output file: " << output_path
                << " (" << strerror(errno) << ")" << std::endl;
      // We still need to free the jpeg_data buffer associated with this failed
      // write. The free happens unconditionally after the write block.
      continue;  // Move to the next JPEG in the batch.
    }

    /**
     * @brief Write the compressed JPEG data from the buffer to the file.
     * @details `fwrite(jpeg.jpeg_data, 1, jpeg.jpeg_size, fp)` attempts to
     * write `jpeg.jpeg_size` items of size 1 byte. It returns the number of
     * items successfully written.
     */
    size_t bytes_written = fwrite(jpeg.jpeg_data, 1, jpeg.jpeg_size, fp);

    /**
     * @brief Close the output file stream.
     */
    fclose(fp);  // Returns 0 on success, EOF on error. (Error check omitted).

    /**
     * @brief Check if the write operation was successful (all bytes written).
     */
    if (bytes_written == jpeg.jpeg_size) {
      /**
       * @brief Increment success counter and print a confirmation message.
       */
      success_count++;
      // Optional: Make logging less verbose by removing this success message?
      std::cout << "  Wrote frame " << jpeg.frame_number << " -> "
                << output_path << " (" << jpeg.jpeg_size << " bytes)\n";
    } else {
      /**
       * @brief Print an error message if the number of bytes written does not
       * match the expected size.
       */
      std::cerr << "ERROR: Failed to write all bytes for frame "
                << jpeg.frame_number << " to " << output_path << " (wrote "
                << bytes_written << "/" << jpeg.jpeg_size << ")" << std::endl;
      // Potential issue: disk full, permissions after open, etc.
    }

    /**
     * @brief Free the JPEG buffer allocated by libjpeg-turbo.
     * @details This **must** be done using `tjFree()` and should happen
     * regardless of whether the file write succeeded or failed, as long as
     * `jpeg.jpeg_data` was not NULL.
     */
    if (jpeg.jpeg_data) {  // Check again just in case? Though outer check
                           // should suffice.
      tjFree(jpeg.jpeg_data);
      // Avoid double-free: although JpegOutput is const&, the data it points to
      // is managed. This implies the original JpegOutput in the vector should
      // have its pointer cleared, but clearing the vector later handles this.
      // This const_cast would be bad practice. Let's assume the clear() later
      // is sufficient. The `tjFree` works on the pointer value.
    }

  }  // End of loop through jpeg_outputs

  /**
   * @brief Clear the vector, removing all `JpegOutput` elements after
   * processing the batch.
   * @details This prepares the vector for the next batch. Destructors are
   * called, but since the struct contains raw pointers, this doesn't free the
   * pointed-to memory (which was done manually with `tjFree`).
   */
  jpeg_outputs.clear();

  /**
   * @brief Return the count of successfully written files in this batch.
   */
  return success_count;
}

/**
 * @brief Scans input directory, validates, allocates, sorts, and loads all RGB
 * frames into memory.
 * @details Implements the pre-loading strategy. It performs two main passes
 * over the directory contents:
 * 1. **First Pass (Validation & Allocation):**
 * - Opens the input directory (`opendir`).
 * - Reads directory entries one by one (`readdir`).
 * - Skips special entries "." and "..".
 * - Constructs the full path for each entry.
 * - Uses `stat` to ensure it's a regular file.
 * - Calls `validate_rgb_filename` to check the naming pattern
 * ("frame_NUMBER.rgb") and extract the frame number.
 * - If valid, creates a `FrameInfo` structure, allocates memory for `file_path`
 * (`strdup`) and `rgb_data` (`malloc`), and adds it to a temporary vector.
 * Handles allocation failures gracefully.
 * 2. **Sorting:** Sorts the vector of `FrameInfo` structures based on
 * `frame_number` using `std::sort` and a lambda comparison function.
 * 3. **Second Pass (Data Loading):**
 * - Iterates through the sorted `FrameInfo` vector.
 * - For each frame, calls `read_rgb_frame` to read the actual pixel data from
 * the file (whose path is stored in `frame.file_path`) into the pre-allocated
 * `frame.rgb_data` buffer.
 * - If `read_rgb_frame` fails for any frame, frees the corresponding `rgb_data`
 * buffer and sets the pointer to NULL to mark it as invalid for later
 * processing, printing an error. Finally, returns the vector containing
 * `FrameInfo` structures for all potentially valid frames (valid ones will have
 * non-NULL `rgb_data`).
 * @param input_dir C-style string (`const char*`) path to the directory
 * containing RGB frame files.
 * @param width The expected width (`int`) of the frames in pixels.
 * @param height The expected height (`int`) of the frames in pixels.
 * @return `std::vector<FrameInfo>` A vector containing `FrameInfo` structures
 * for all files in the input directory that matched the validation criteria.
 * Frames that failed the second pass (data loading) will have `rgb_data ==
 * NULL`. The vector might be empty if the directory cannot be opened or no
 * valid files are found. The caller is responsible for iterating through the
 * vector and eventually freeing `file_path` and any non-NULL `rgb_data`
 * pointers within the returned `FrameInfo` elements (though `process_frames`
 * handles this).
 */
std::vector<FrameInfo> load_rgb_frames(const char* input_dir, int width,
                                       int height) {
  /**
   * @brief Vector to store FrameInfo structures for valid frames found.
   */
  std::vector<FrameInfo> frames;

  /**
   * @brief Open the input directory for reading.
   * @details Returns a DIR pointer on success, NULL on failure.
   */
  DIR* dir = opendir(input_dir);
  if (!dir) {
    std::cerr << "ERROR: Cannot open input directory: " << input_dir << " ("
              << strerror(errno) << ")" << std::endl;
    return frames;  // Return empty vector on failure.
  }

  /**
   * @brief Calculate the expected size for each RGB data buffer.
   */
  const size_t rgb_buffer_size = static_cast<size_t>(width) * height * 3;

  /**
   * @brief Buffer for constructing full file paths.
   */
  char full_path[512];

  /**
   * @brief Pointer to hold the current directory entry structure.
   */
  struct dirent* entry;

  std::cout << "Scanning directory: " << input_dir << "..." << std::endl;
  // --- First Pass: Identify, validate, and allocate ---
  /**
   * @brief Read directory entries until the end is reached (readdir returns
   * NULL).
   */
  while ((entry = readdir(dir)) != NULL) {
    /**
     * @brief Skip the special directory entries "." (current) and ".."
     * (parent).
     */
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
      continue;
    }

    /**
     * @brief Construct the full path by combining the input directory path and
     * the entry name.
     */
    snprintf(full_path, sizeof(full_path), "%s/%s", input_dir, entry->d_name);

    // Stat check moved inside validate_rgb_filename, but could be done here too
    // struct stat st;
    // if (stat(full_path, &st) != 0 || !S_ISREG(st.st_mode)) {
    //   continue; // Skip if stat fails or not a regular file
    // }

    /**
     * @brief Variable to store the frame number extracted by the validation
     * function.
     */
    int frame_num;

    /**
     * @brief Call the validation function. Proceed only if it returns 0
     * (success).
     */
    if (validate_rgb_filename(full_path, &frame_num) == 0) {
      // Filename is valid and matches the expected pattern.

      /**
       * @brief Create a FrameInfo structure for this valid frame.
       */
      FrameInfo info;
      info.width = width;
      info.height = height;
      info.frame_number = frame_num;
      info.rgb_size = rgb_buffer_size;
      info.file_path = NULL;  // Initialize pointers to NULL
      info.rgb_data = NULL;

      /**
       * @brief Allocate memory for the file path string using strdup (allocates
       * and copies).
       */
      info.file_path = strdup(full_path);

      /**
       * @brief Allocate memory for the RGB data buffer using malloc.
       */
      info.rgb_data = static_cast<unsigned char*>(malloc(rgb_buffer_size));

      /**
       * @brief Check if both memory allocations were successful.
       */
      if (info.file_path && info.rgb_data) {
        /**
         * @brief Add the populated FrameInfo structure to the vector. Uses move
         * semantics implicitly if available.
         */
        frames.push_back(info);
      } else {
        /**
         * @brief Handle memory allocation failure. Free any successfully
         * allocated memory and report error.
         */
        std::cerr
            << "ERROR: Memory allocation failed for frame info/data (frame "
            << frame_num << ")" << std::endl;
        free(info.file_path);  // Free path buffer if allocated.
        free(info.rgb_data);   // Free data buffer if allocated.
        // Optionally: decide whether to stop the whole process on allocation
        // failure. Here, we just skip this frame.
      }
    }
    // End if (validate_rgb_filename == 0)
  }  // End while (readdir)

  /**
   * @brief Close the directory stream.
   */
  closedir(dir);

  std::cout << "Found " << frames.size() << " potential frame files. Sorting..."
            << std::endl;

  // --- Sort frames by frame number ---
  /**
   * @brief Sort the vector of FrameInfo structures based on the `frame_number`
   * field in ascending order.
   * @details Uses `std::sort` with a lambda expression for the comparison
   * function.
   */
  std::sort(frames.begin(), frames.end(),
            [](const FrameInfo& a, const FrameInfo& b) {
              return a.frame_number < b.frame_number;
            });

  std::cout << "Loading RGB data for " << frames.size() << " frames..."
            << std::endl;
  // --- Second Pass: Load RGB data ---
  /**
   * @brief Counter for successfully loaded frames in the second pass.
   */
  size_t loaded_count = 0;

  /**
   * @brief Iterate through the (now sorted) vector of FrameInfo structures by
   * reference (`auto&`).
   */
  for (auto& frame : frames) {
    // FrameInfo struct already contains allocated buffer frame.rgb_data
    /**
     * @brief Call `read_rgb_frame` to load data from `frame.file_path` into
     * `frame.rgb_data`.
     * @details Check the return code.
     */
    if (read_rgb_frame(frame.file_path, width, height, frame.rgb_data) == 0) {
      // Read was successful.
      loaded_count++;
    } else {
      /**
       * @brief If reading failed, report error, free the buffer associated with
       * this frame, and set the pointer to NULL.
       * @details Setting `rgb_data` to NULL marks this frame as invalid for the
       * subsequent processing stage.
       */
      std::cerr << "ERROR: Failed to read RGB data for frame "
                << frame.frame_number << " from " << frame.file_path
                << std::endl;
      free(frame.rgb_data);   // Free the buffer that failed to be filled.
      frame.rgb_data = NULL;  // Mark as invalid.
    }
  }  // End of loop loading RGB data

  /**
   * @brief Report the number of frames successfully loaded in the second pass.
   */
  std::cout << "Successfully loaded data for " << loaded_count << " of "
            << frames.size() << " validated frames." << std::endl;

  /**
   * @brief Return the vector of FrameInfo structures.
   */
  return frames;
}

/**
 * @brief Main processing function: loads frames, converts RGB to JPEG, and
 * writes JPEGs in batches.
 * @details Orchestrates the core workflow after initial setup.
 * 1. Creates the output directory if it doesn't exist.
 * 2. Calls `load_rgb_frames` to get a vector of all frame info and pixel data.
 * Exits if no valid frames are loaded.
 * 3. Initializes an empty vector (`jpeg_outputs`) to buffer the compressed
 * JPEGs for batch writing. Reserves space for efficiency.
 * 4. Iterates through the loaded `FrameInfo` vector:
 * a. Skips frames marked as invalid (NULL `rgb_data`).
 * b. Calls `rgb_to_yuv420_jpeg` to compress the RGB data.
 * c. Frees the source `rgb_data` buffer immediately after compression (whether
 * successful or not) to reduce memory usage. d. If compression succeeds,
 * creates a `JpegOutput` structure with the JPEG buffer pointer, size, and
 * frame number, and adds it to the `jpeg_outputs` batch vector. e. If the batch
 * vector size reaches `MAX_JPEG_BUFFER`, calls `write_jpeg_batch` to write the
 * buffered JPEGs to disk and clears the batch vector. f. If compression fails,
 * prints an error message. g. Frees the `file_path` string memory for the
 * current frame.
 * 5. After the loop, calls `write_jpeg_batch` one final time to ensure any
 * remaining JPEGs in the buffer are written to disk.
 * @param input_dir C-style string (`const char*`) path to the source directory
 * containing RGB frames.
 * @param output_dir C-style string (`const char*`) path to the target directory
 * for saving JPEG files.
 * @param width The width (`int`) of the frames in pixels.
 * @param height The height (`int`) of the frames in pixels.
 * @param quality The JPEG quality factor (`int`, 1-100) to use for compression.
 */
void process_frames(const char* input_dir, const char* output_dir, int width,
                    int height, int quality) {
  /**
   * @brief Create the output directory using `mkdir`.
   * @details `mkdir(output_dir, 0755)` attempts to create the directory.
   * - `output_dir`: Path to the directory.
   * - `0755`: Permissions mode (owner: rwx, group: r-x, others: r-x).
   * `mkdir` returns 0 on success, -1 on error. If the directory already exists,
   * it returns -1 and sets `errno` to `EEXIST`, which is acceptable here. Other
   * errors are ignored for simplicity, but could be checked.
   */
  // Note: Error checking could be added here (check return value and errno !=
  // EEXIST).
  mkdir(output_dir, 0755);  // Mode 0755: rwxr-xr-x

  /**
   * @brief Load all valid RGB frames from the input directory into memory.
   */
  std::cout << "Loading frames..." << std::endl;
  std::vector<FrameInfo> frames = load_rgb_frames(input_dir, width, height);

  /**
   * @brief Check if any valid frames were loaded. If not, print error and
   * return.
   */
  if (frames.empty()) {
    // load_rgb_frames prints specific errors, so just a general message here.
    std::cerr << "ERROR: No valid RGB frames found or loaded from directory: "
              << input_dir << std::endl;
    return;  // Exit processing.
  }

  /**
   * @brief Vector to buffer JpegOutput structures before writing them in a
   * batch.
   */
  std::vector<JpegOutput> jpeg_outputs;

  /**
   * @brief Reserve space in the vector to avoid reallocations up to
   * MAX_JPEG_BUFFER size.
   * @details This is a minor optimization for vector performance.
   */
  jpeg_outputs.reserve(MAX_JPEG_BUFFER);

  std::cout << "Processing " << frames.size() << " frames..." << std::endl;
  size_t processed_count = 0;
  size_t failed_compression_count = 0;

  /**
   * @brief Iterate through the vector of loaded FrameInfo structures.
   */
  for (auto& frame :
       frames) {  // Iterate by reference to allow modifying (freeing) members.
    /**
     * @brief Skip frames that were marked as invalid during loading (failed to
     * read data).
     */
    if (!frame.rgb_data) {
      // Free the path memory even for frames that failed to load data.
      free(frame.file_path);
      frame.file_path = NULL;  // Mark as fully processed.
      continue;                // Skip to the next frame.
    }

    /**
     * @brief Variable to store the size of the compressed JPEG output.
     */
    unsigned long jpeg_size = 0;

    /**
     * @brief Call the compression function to convert RGB to JPEG.
     */
    unsigned char* jpeg_buf = rgb_to_yuv420_jpeg(
        frame.rgb_data, frame.width, frame.height, quality, &jpeg_size);

    /**
     * @brief **Crucial:** Free the source RGB data buffer *immediately* after
     * compression attempt.
     * @details This minimizes peak memory usage, as the large RGB buffer is no
     * longer needed once `jpeg_buf` (potentially) holds the compressed result.
     * Set pointer to NULL after freeing.
     */
    free(frame.rgb_data);
    frame.rgb_data = NULL;  // Mark RGB data as freed.

    /**
     * @brief Check if the compression was successful (returned non-NULL
     * buffer).
     */
    if (jpeg_buf && jpeg_size > 0) {  // Also check size > 0 for robustness
                                      /**
                                       * @brief Create a JpegOutput structure to hold the result.
                                       * @details Uses designated initializers (C++20 style) or standard struct
                                       * initialization.
                                       */
      // C++11 style initialization
      JpegOutput output;
      output.jpeg_data = jpeg_buf;
      output.jpeg_size = jpeg_size;
      output.frame_number = frame.frame_number;

      /* // C++20 designated initializer style:
      JpegOutput output = {.jpeg_data = jpeg_buf,
                           .jpeg_size = jpeg_size,
                           .frame_number = frame.frame_number};
      */

      /**
       * @brief Add the JpegOutput structure to the batch buffer vector.
       */
      jpeg_outputs.push_back(output);  // Copies the struct (pointers included).

      /**
       * @brief Check if the batch buffer has reached its maximum configured
       * size.
       */
      if (jpeg_outputs.size() >= MAX_JPEG_BUFFER) {
        /**
         * @brief If buffer is full, write the current batch to disk.
         */
        std::cout << "Writing JPEG batch (" << jpeg_outputs.size()
                  << " files)..." << std::endl;
        write_jpeg_batch(
            output_dir,
            jpeg_outputs);  // This function also clears jpeg_outputs.
        // jpeg_outputs should be empty after write_jpeg_batch returns.
      }
    } else {
      /**
       * @brief If compression failed (`jpeg_buf` is NULL), report the error.
       */
      failed_compression_count++;
      std::cerr << "ERROR: Failed to compress frame " << frame.frame_number
                << std::endl;
      // No jpeg_buf to add to the batch or free.
    }

    /**
     * @brief Free the memory allocated for the file path string.
     * @details This is done after processing each frame (compression attempt
     * and adding to batch if successful).
     */
    free(frame.file_path);
    frame.file_path = NULL;  // Mark path as freed.
    processed_count++;

  }  // End of loop through frames

  /**
   * @brief After processing all frames, check if there are any remaining JPEGs
   * in the buffer.
   */
  if (!jpeg_outputs.empty()) {
    /**
     * @brief Write the final batch of JPEGs to disk.
     */
    std::cout << "Writing final JPEG batch (" << jpeg_outputs.size()
              << " files)..." << std::endl;
    write_jpeg_batch(output_dir, jpeg_outputs);
  }

  std::cout << "Processing complete. Processed: " << processed_count
            << ", Failed Compressions: " << failed_compression_count
            << std::endl;
}

/**
 * @brief Main application entry point, optimized for simplicity and reduced
 * overhead.
 * @details Handles command-line argument parsing and basic validation before
 * initiating the frame processing workflow. It avoids complex argument parsing
 * libraries and C++ exceptions.
 *
 * **Command Line Usage:**
 * ```
 * ./rgb2jpeg_batch <input_dir> <output_dir> <width> <height> [quality]
 * ```
 * - `<input_dir>`: Path to directory with "frame_NUMBER.rgb" files.
 * - `<output_dir>`: Path to directory where "jpeg_NUMBER.jpg" files will be
 * saved.
 * - `<width>`: Width of the input images in pixels.
 * - `<height>`: Height of the input images in pixels.
 * - `[quality]`: Optional JPEG quality factor (1-100, default: 75).
 *
 * **Example:**
 * ```
 * ./rgb2jpeg_batch ./input_rgb_frames ./output_jpegs 640 480 85
 * ```
 * @param argc Integer count of command-line arguments (including program name).
 * @param argv Array of C-style strings (`char*`) holding the command-line
 * arguments.
 * @return `int` Exit code: 0 for successful completion, 1 for errors during
 * argument parsing or validation.
 */
int main(int argc, char* argv[]) {
  /**
   * @brief Check if the minimum number of required arguments (5) is provided.
   * @details Program name + 4 required arguments. Quality is optional.
   */
  if (argc < 5) {
    /**
     * @brief Print usage instructions to standard error if arguments are
     * insufficient.
     */
    std::cerr << "ERROR: Incorrect number of arguments." << std::endl;
    std::cerr << "Usage: " << argv[0]  // argv[0] is the program name.
              << " <input_dir> <output_dir> <width> <height> [quality=75]"
              << std::endl;
    std::cerr
        << "Example: ./rgb2jpeg_batch ./rgb_frames ./jpeg_output 640 480 85"
        << std::endl;
    return 1;  // Return error code 1.
  }

  /**
   * @brief Get input directory path from the first argument (argv[1]).
   */
  const char* input_dir = argv[1];

  /**
   * @brief Get output directory path from the second argument (argv[2]).
   */
  const char* output_dir = argv[2];

  /**
   * @brief Parse frame width from the third argument (argv[3]) using `atoi`
   * (ASCII to Integer).
   * @details `atoi` provides basic conversion, returns 0 on error or if input
   * is not numerical.
   */
  int width = atoi(argv[3]);

  /**
   * @brief Parse frame height from the fourth argument (argv[4]) using `atoi`.
   */
  int height = atoi(argv[4]);

  /**
   * @brief Parse optional JPEG quality from the fifth argument (argv[5]) if
   * present, otherwise default to 75.
   * @details Uses the ternary operator `(condition ? value_if_true :
   * value_if_false)`.
   */
  int quality = (argc > 5) ? atoi(argv[5]) : 75;  // Default quality = 75

  /**
   * @brief Perform basic sanity checks on parsed width and height.
   * @details Ensure dimensions are positive. `atoi` returns 0 for non-numeric
   * input, which will also fail this check.
   */
  if (width <= 0 || height <= 0) {
    std::cerr << "ERROR: Invalid width or height provided (" << argv[3] << "x"
              << argv[4] << "). Dimensions must be positive integers."
              << std::endl;
    return 1;  // Return error code 1.
  }

  /**
   * @brief Sanitize the parsed quality value to be within the valid range [1,
   * 100].
   * @details Clamp the value if it falls outside the allowed range.
   */
  if (quality < 1) quality = 1;      // Clamp quality to minimum 1.
  if (quality > 100) quality = 100;  // Clamp quality to maximum 100.

  /**
   * @brief Print the configuration being used.
   */
  std::cout << "Configuration:" << std::endl;
  std::cout << "  Input Directory:  " << input_dir << std::endl;
  std::cout << "  Output Directory: " << output_dir << std::endl;
  std::cout << "  Dimensions:       " << width << "x" << height << std::endl;
  std::cout << "  JPEG Quality:     " << quality << std::endl;
  std::cout << "  JPEG Batch Size:  " << MAX_JPEG_BUFFER << std::endl;
  std::cout << "--------------------------------" << std::endl;

  /**
   * @brief Call the main processing function with the validated parameters.
   */
  process_frames(input_dir, output_dir, width, height, quality);

  /**
   * @brief Return 0 to indicate successful completion (assuming
   * `process_frames` doesn't exit prematurely on error).
   */
  return 0;  // Success.
}

