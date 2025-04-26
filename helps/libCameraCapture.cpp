/**
 * @file libCameraCapture.cpp
 * @author Vaibhav
 * @brief Extremely verbose camera capture application using libcamera.
 * @details This program serves as a detailed example of using the libcamera C++
 * API. It meticulously demonstrates the steps required to capture video frames
 * from a camera device, specifically focusing on:
 * - Initializing and terminating the core libcamera system (`CameraManager`).
 * - Discovering available camera devices connected to the system.
 * - Selecting a specific camera device for operation.
 * - Acquiring exclusive access to the selected camera device.
 * - Generating a valid camera configuration based on desired use cases (e.g.,
 * VideoRecording).
 * - Allowing the user to select a specific video format (pixel format and
 * resolution) from the list of supported formats offered by the camera.
 * - Validating the chosen configuration to ensure compatibility with the
 * hardware.
 * - Applying the validated configuration to the camera device.
 * - Retrieving the configured stream object for further operations.
 * - Allocating necessary memory buffers (`FrameBuffer`) required for capturing
 * frames.
 * - Creating camera request objects (`Request`) and associating them with the
 * allocated buffers.
 * - Setting up asynchronous notification for completed capture requests using
 * signal/slot mechanism.
 * - Starting the camera's streaming process.
 * - Enqueuing the initial set of requests to begin capture.
 * - Implementing the main capture loop that waits for completed requests,
 * processes them, saves frame data and associated metadata (timestamps), and
 * re-queues requests for continuous capture.
 * - Extracting frame pixel data from frame buffers, considering memory layout
 * details like stride.
 * - Extracting metadata, specifically the sensor timestamp, from completed
 * requests.
 * - Saving the captured frame data to individual binary files (e.g.,
 * "frame_0.rgb").
 * - Saving the corresponding capture timestamps to separate text files (e.g.,
 * "frame_0.ts").
 * - Implementing robust error handling at each critical step.
 * - Performing proper cleanup of all allocated resources (requests, buffers,
 * camera access, manager) upon completion or error. The code utilizes standard
 * C++ features like smart pointers, threads, mutexes, condition variables, and
 * file streams alongside the libcamera library.
 */

// ========================================================================== //
// ==                            INCLUDE DIRECTIVES                        == //
// ========================================================================== //

#include <libcamera/control_ids.h>  // Provides definitions for standard camera control identifiers, such as 'libcamera::controls::SensorTimestamp'. Needed to query metadata.
#include <libcamera/formats.h>  // Provides definitions for libcamera pixel formats (e.g., RGB888, YUV420) and related structures. Needed for 'stream_config.pixelFormat'.
#include <libcamera/geometry.h>  // Provides definitions for geometric types like 'libcamera::Size' and 'libcamera::Rectangle'. Needed for 'stream_config.size'.
#include <libcamera/libcamera.h>  // Includes the core libcamera headers, providing access to main classes like 'CameraManager', 'Camera', 'CameraConfiguration', 'Request', 'FrameBuffer', etc. Essential for the entire application.
#include <sys/mman.h>  // Provides declarations for memory mapping functions like 'mmap' and 'munmap'. Required for accessing frame buffer memory directly via file descriptors.
#include <unistd.h>  // Provides access to the POSIX operating system API, including various constants and types. Implicitly used by other headers, potentially for file descriptor operations.

#include <condition_variable>  // Provides 'std::condition_variable' for thread synchronization, allowing threads to wait efficiently until a condition is met (e.g., a request is ready). Used with 'request_cv'.
#include <fstream>  // Provides classes for file stream operations ('std::ofstream'). Needed for saving frame data and timestamps to files.
#include <iomanip>  // Provides stream manipulators like 'std::setprecision'. Although not used in the final timestamp saving (which uses plain integer), it might be useful for formatted output if needed.
#include <iostream>  // Provides standard input/output stream objects ('std::cin', 'std::cout', 'std::cerr'). Used for user interaction and logging messages.
#include <memory>  // Provides smart pointer classes like 'std::unique_ptr' and 'std::shared_ptr'. Used for managing the lifetime of libcamera objects (CameraManager, Camera, Request) and preventing memory leaks.
#include <mutex>  // Provides 'std::mutex' for mutual exclusion locks. Used to protect shared resources ('request_queue') from race conditions in multi-threaded contexts. Used with 'request_mutex'.
#include <optional>  // Provides 'std::optional' template class. Used to safely handle potentially missing metadata values like the sensor timestamp.
#include <queue>  // Provides the 'std::queue' container adapter. Used to store completed camera requests in a First-In, First-Out manner.
#include <string>  // Provides the 'std::string' class for manipulating sequences of characters. Used for filenames and user messages.
#include <vector>  // Provides the 'std::vector' dynamic array container. Used to store lists of cameras, format options, allocated buffers, and requests.

// ========================================================================== //
// ==                      GLOBAL VARIABLES & SYNCHRONIZATION              == //
// ========================================================================== //

/**
 * @brief A mutex to ensure thread-safe access to the shared 'request_queue'.
 * @details This mutex must be locked before accessing 'request_queue' (pushing
 * or popping elements) and unlocked afterwards. This prevents multiple threads
 * from modifying the queue simultaneously, which could lead to data corruption
 * or unpredictable behavior. 'std::lock_guard' or 'std::unique_lock' should be
 * used to manage locking and unlocking automatically.
 */
std::mutex request_mutex;

/**
 * @brief A queue holding pointers to completed libcamera requests.
 * @details When libcamera finishes processing a capture request, the
 * `request_complete` callback function pushes a pointer to the completed
 * `libcamera::Request` object onto this queue. The main processing thread then
 * waits for this queue to become non-empty, dequeues a request, processes its
 * data (frame, metadata), and eventually re-queues it for reuse. Access to this
 * queue is synchronized using 'request_mutex'.
 */
std::queue<libcamera::Request *> request_queue;

/**
 * @brief A condition variable used to signal waiting threads about changes in
 * 'request_queue'.
 * @details The main processing thread waits ('request_cv.wait()') on this
 * condition variable when the 'request_queue' is empty. The 'request_complete'
 * callback function notifies
 * ('request_cv.notify_one()') one waiting thread whenever a new request is
 * pushed onto the queue. This mechanism avoids busy-waiting and allows the
 * processing thread to sleep efficiently until work is available. It must be
 * used in conjunction with 'request_mutex'.
 */
std::condition_variable request_cv;

// ========================================================================== //
// ==                       CALLBACK FUNCTION DEFINITION                   == //
// ========================================================================== //

/**
 * @brief Callback function invoked by libcamera when a request completes
 * processing.
 * @details This function is connected to the `requestCompleted` signal of the
 * `libcamera::Camera` object. libcamera's internal threads call this function
 * asynchronously whenever a capture request (previously queued via
 * `camera->queueRequest()`) finishes. Its primary role is to enqueue the
 * completed request for processing by the main application thread.
 * @param request A pointer to the `libcamera::Request` object that has just
 * completed. The ownership of the request remains with libcamera/the
 * application's request pool at this point; this function only handles
 * notification.
 */
void request_complete(libcamera::Request *request) {
  /**
   * @brief Check the status of the completed request.
   * @details Requests can be cancelled (e.g., during camera stop). If the
   * request was cancelled, we typically don't want to process it further, so we
   * simply return.
   */
  if (request->status() == libcamera::Request::RequestCancelled) {
    // Print a message if needed for debugging cancellation
    // std::cout << "DEBUG: Request cancelled, skipping." << std::endl;
    return;  // Exit the function immediately for cancelled requests.
  }

  /**
   * @brief Acquire a lock on the request queue mutex.
   * @details A 'std::lock_guard' automatically locks the provided mutex
   * ('request_mutex') upon construction and unlocks it upon destruction (when
   * the guard goes out of scope at the end of this block). This ensures
   * exclusive access to the 'request_queue' within this critical section.
   */
  std::lock_guard<std::mutex> lock(request_mutex);

  /**
   * @brief Add the pointer to the completed request object to the end of the
   * queue.
   * @details The main thread will later retrieve this pointer from the front of
   * the queue.
   */
  request_queue.push(request);

  /**
   * @brief Notify one waiting thread that a request has been added to the
   * queue.
   * @details If the main thread is waiting on 'request_cv' (because the queue
   * was empty), this notification will wake it up so it can process the newly
   * added request. If no thread is waiting, the notification has no effect.
   */
  request_cv.notify_one();

  // The lock_guard 'lock' goes out of scope here, automatically releasing the
  // mutex.
}

// ========================================================================== //
// ==                        HELPER FUNCTION DEFINITIONS                   == //
// ========================================================================== //

/**
 * @brief Saves the image data from frame buffer planes to a single binary file.
 * @details This function takes the planes of a completed frame buffer, maps
 * their memory, reads the pixel data row by row (accounting for
 * stride/padding), and writes it to a file. The output file format is raw pixel
 * data (assumed RGB24 for simplicity here, though the pixel format is selected
 * by the user).
 * @param planes A constant reference to a vector containing the
 * `libcamera::FrameBuffer::Plane` objects associated with the captured frame.
 * Each plane might represent a different color component or data segment
 * depending on the pixel format, but for packed formats like RGB, there's often
 * just one plane.
 * @param count An integer representing the sequence number of the frame being
 * saved. Used to generate a unique filename.
 * @param stride A 32-bit unsigned integer representing the number of bytes
 * between the start of consecutive rows in the frame buffer memory. This may be
 * larger than `width * bytes_per_pixel` due to hardware alignment requirements
 * (padding).
 * @param width A 32-bit unsigned integer representing the actual width of the
 * image in pixels.
 * @param height A 32-bit unsigned integer representing the actual height of the
 * image in pixels.
 * @return `true` if the frame data was successfully mapped, written to the
 * file, and unmapped. `false` if any error occurred (e.g., file opening failed,
 * memory mapping failed).
 */
bool save_frame(const std::vector<libcamera::FrameBuffer::Plane> &planes,
                int count, uint32_t stride, uint32_t width, uint32_t height) {
  /**
   * @brief Construct the filename for the output frame file.
   * @details The filename follows the pattern "frame_[count].rgb".
   * `std::to_string` converts the integer count to its string representation.
   */
  std::string filename = "frame_" + std::to_string(count) + ".rgb";

  /**
   * @brief Open the output file in binary write mode.
   * @details 'std::ofstream' is used to create and write to files.
   * 'std::ios::binary' flag ensures data is written byte-by-byte without
   * text-mode translations.
   */
  std::ofstream file(filename, std::ios::binary);

  /**
   * @brief Check if the file was opened successfully.
   * @details If 'file.is_open()' returns false, an error occurred (e.g.,
   * insufficient permissions, invalid path). An error message is printed to
   * 'std::cerr' (standard error stream).
   */
  if (!file.is_open()) {
    std::cerr << "ERROR: Failed to open frame file: " << filename << std::endl;
    return false;  // Return failure indication.
  }

  /**
   * @brief Iterate through each plane in the frame buffer.
   * @details For simple packed formats like RGB24, there will usually be only
   * one plane containing all color data interleaved. For planar formats (like
   * YUV420), there might be multiple planes (e.g., one for Y, one for U, one
   * for V). This code implicitly assumes a single relevant plane or
   * concatenates data from multiple planes if present. Note: For multi-planar
   * formats, specific handling per-plane might be required depending on desired
   * output.
   */
  for (const auto &plane : planes) {
    /**
     * @brief Map the plane's buffer memory into the application's address
     * space.
     * @details 'mmap' is a system call that maps files or devices into memory.
     * Here, it maps the memory associated with the frame buffer's file
     * descriptor ('plane.fd.get()') into the process's virtual address space.
     * - `nullptr`: Suggests the kernel choose the address.
     * - `plane.length`: Specifies the size of the mapping (the entire plane
     * buffer).
     * - `PROT_READ`: Specifies that the mapped memory should be readable.
     * - `MAP_SHARED`: Specifies that modifications to the mapped memory (if
     * writing was allowed) should be visible to other processes mapping the
     * same object, and written back to the underlying file/device (though we
     * only read here).
     * - `plane.fd.get()`: Gets the underlying file descriptor associated with
     * this buffer plane. libcamera often uses DMA-BUF or shared memory
     * mechanisms represented by fds.
     * - `0`: Specifies the offset within the file/device to start the mapping
     * (usually 0 for buffers).
     * @return On success, 'mmap' returns a pointer to the mapped memory. On
     * failure, it returns 'MAP_FAILED'.
     */
    void *data =
        mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);

    /**
     * @brief Check if the memory mapping failed.
     * @details If 'mmap' returned 'MAP_FAILED', print an error and clean up.
     */
    if (data == MAP_FAILED) {
      std::cerr << "ERROR: Failed to mmap buffer for frame " << count
                << std::endl;
      file.close();  // Close the file before returning.
      return false;  // Return failure indication.
    }

    /**
     * @brief Cast the generic 'void*' pointer to a 'char*' pointer for
     * byte-level access.
     * @details 'char*' is commonly used for raw byte manipulation in C++.
     */
    char *ptr = static_cast<char *>(data);

    /**
     * @brief Define the number of bytes per pixel.
     * @details This assumes an RGB24 format (3 bytes: Red, Green, Blue).
     * **Important**: This should ideally be derived from the actual
     * `stream_config.pixelFormat` selected by the user for correctness with
     * other formats. For this example, it's hardcoded for simplicity of the
     * stride calculation logic shown.
     */
    const size_t bytes_per_pixel =
        3;  // FIXME: Hardcoded assumption for RGB24/BGR24

    /**
     * @brief Calculate the number of bytes that constitute one valid row of
     * image data.
     * @details This is the image width multiplied by the bytes per pixel. This
     * value might be less than the `stride`.
     */
    const size_t bytes_per_row = width * bytes_per_pixel;

    /**
     * @brief Loop through each row of the image height.
     * @details The loop iterates 'height' times, processing one row in each
     * iteration.
     */
    for (uint32_t y = 0; y < height; ++y) {
      /**
       * @brief Write one row of valid image data to the file.
       * @details 'file.write()' writes 'bytes_per_row' bytes starting from the
       * current memory location 'ptr' into the output file. This copies only
       * the actual image data, skipping any padding bytes at the end of the row
       * in the source buffer.
       */
      file.write(ptr, bytes_per_row);

      /**
       * @brief Advance the memory pointer to the start of the next row in the
       * source buffer.
       * @details The pointer is advanced by 'stride' bytes, which correctly
       * moves to the beginning of the next line, accounting for any padding.
       */
      ptr += stride;
    }

    /**
     * @brief Unmap the memory region previously mapped by 'mmap'.
     * @details 'munmap' removes the mapping for the specified address range
     * ('data', 'plane.length'). This releases the mapping but doesn't
     * necessarily free the underlying buffer (which is managed by libcamera's
     * allocator). It's crucial to unmap memory that was mapped.
     * @return Returns 0 on success, -1 on failure. (Error checking is omitted
     * here for brevity but recommended in production code).
     */
    munmap(data, plane.length);

    // Note: If handling multi-planar formats, the loop might need adjustments
    // to write data from different planes appropriately (e.g., consecutively or
    // to separate files).
  }  // End of loop through planes

  /**
   * @brief Close the output file stream.
   * @details This flushes any buffered data to the file and releases the file
   * handle. It's good practice to explicitly close, although it would also
   * happen when 'file' goes out of scope.
   */
  file.close();

  /**
   * @brief Return success indication.
   * @details If the function reaches this point, it means all operations
   * completed without returning 'false'.
   */
  return true;
}

/**
 * @brief Saves the capture timestamp value to a separate text file.
 * @details This function takes a timestamp (typically in nanoseconds) and
 * writes it as a plain text number into a file named corresponding to the frame
 * count.
 * @param timestamp_ns An `int64_t` representing the capture timestamp, usually
 * obtained from request metadata (e.g., SensorTimestamp). Units are expected to
 * be nanoseconds. A value of -1 might indicate a missing timestamp.
 * @param count An integer representing the sequence number of the frame this
 * timestamp belongs to. Used to generate a unique filename matching the frame
 * file.
 * @return `true` if the timestamp was successfully written to the file.
 * `false` if any error occurred (e.g., file opening failed, writing failed).
 */
bool save_timestamp(int64_t timestamp_ns, int count) {
  /**
   * @brief Construct the filename for the output timestamp file.
   * @details The filename follows the pattern "frame_[count].ts".
   * `std::to_string` converts the integer count to its string representation.
   */
  std::string filename = "frame_" + std::to_string(count) + ".ts";

  /**
   * @brief Open the output file in text write mode.
   * @details 'std::ofstream' is used. Default mode is text. If the file exists,
   * it will be overwritten.
   */
  std::ofstream file(filename);

  /**
   * @brief Check if the file was opened successfully.
   * @details If 'file.is_open()' returns false, report an error to 'std::cerr'.
   */
  if (!file.is_open()) {
    std::cerr << "ERROR: Failed to open timestamp file: " << filename
              << std::endl;
    return false;  // Return failure indication.
  }

  /**
   * @brief Write the timestamp value to the file as a text string.
   * @details The `<<` operator converts the `int64_t` timestamp into its
   * decimal string representation and writes it to the file stream. Saving as
   * text makes it easily human-readable and simple to parse by other
   * tools/scripts.
   */
  file << timestamp_ns;

  /**
   * @brief Check if the write operation encountered any errors.
   * @details `file.good()` checks if the stream state is still valid after the
   * write. If not (e.g., disk full), report an error.
   */
  if (!file.good()) {
    std::cerr << "ERROR: Failed to write timestamp to file: " << filename
              << std::endl;
    file.close();  // Attempt to close the file even on error.
    return false;  // Return failure indication.
  }

  /**
   * @brief Close the output file stream.
   * @details Flushes the buffer and releases the file handle.
   */
  file.close();

  /**
   * @brief Return success indication.
   * @details Indicates that the timestamp was successfully written and the file
   * was closed.
   */
  return true;
}

// ========================================================================== //
// ==                          MAIN APPLICATION ENTRY POINT                == //
// ========================================================================== //

/**
 * @brief The main function orchestrating the entire camera capture process.
 * @details This function executes the primary logic of the application:
 * 1. Initializes libcamera.
 * 2. Finds and selects a camera.
 * 3. Configures the camera stream.
 * 4. Allocates buffers and creates requests.
 * 5. Starts the camera and manages the capture loop.
 * 6. Processes completed requests (saving frames and timestamps).
 * 7. Cleans up all resources upon completion or error.
 * @param argc The number of command-line arguments (not used in this
 * application).
 * @param argv An array of command-line argument strings (not used in this
 * application).
 * @return `EXIT_SUCCESS` (usually 0) if the program executes successfully.
 * `EXIT_FAILURE` (usually 1) if any critical error occurs during execution.
 */
int main() {
  /**
   * @brief Print a startup message to the standard output console.
   * @details Informs the user that the application is beginning its execution.
   */
  std::cout << "Starting libcamera capture..." << std::endl;

  /**
   * @brief Declare and initialize the CameraManager using a unique pointer.
   * @details `libcamera::CameraManager` is the main entry point for interacting
   * with libcamera. It manages camera discovery and access. `std::unique_ptr`
   * ensures that the CameraManager resources are automatically released (via
   * `cm->stop()` in the destructor if not explicitly called) when `cm` goes out
   * of scope or is reset. `std::make_unique` is the preferred way to create
   * unique pointers.
   */
  std::unique_ptr<libcamera::CameraManager> cm =
      std::make_unique<libcamera::CameraManager>();

  /**
   * @brief Start the CameraManager.
   * @details This initializes the underlying libcamera infrastructure,
   * potentially discovering available cameras.
   * @return Returns 0 on success, or a non-zero error code on failure.
   */
  if (cm->start()) {
    /**
     * @brief Handle failure to start the CameraManager.
     * @details If `cm->start()` returns non-zero, print an error message to
     * `std::cerr` and exit the program with a failure status.
     */
    std::cerr << "ERROR: Failed to start CameraManager" << std::endl;
    return EXIT_FAILURE;  // Indicate program failure.
  }
  /**
   * @brief Print a success message indicating the CameraManager started.
   */
  std::cout << "CameraManager started." << std::endl;

  /**
   * @brief Get the list of available cameras detected by the CameraManager.
   * @details `cm->cameras()` returns a `std::vector` of
   * `std::shared_ptr<libcamera::Camera>`. Using `auto` deduces the type of
   * `cameras`. Shared pointers are used because multiple parts of the system
   * might reference the same camera object.
   */
  auto cameras = cm->cameras();

  /**
   * @brief Check if any cameras were found.
   * @details If the `cameras` vector is empty, it means no compatible camera
   * devices were detected by libcamera.
   */
  if (cameras.empty()) {
    /**
     * @brief Handle the case where no cameras are found.
     * @details Print an error message, stop the CameraManager to clean up, and
     * exit with failure status.
     */
    std::cerr << "ERROR: No cameras found" << std::endl;
    cm->stop();           // Clean up CameraManager resources.
    return EXIT_FAILURE;  // Indicate program failure.
  }
  /**
   * @brief Print the number of cameras found.
   */
  std::cout << "Found " << cameras.size() << " camera(s)." << std::endl;

  /**
   * @brief Select the first camera from the list.
   * @details This application simply chooses the camera at index 0. A more
   * complex application might allow the user to select or identify cameras
   * based on properties (e.g., ID, position). The
   * `std::shared_ptr<libcamera::Camera>` is assigned to the `camera` variable.
   */
  std::shared_ptr<libcamera::Camera> camera = cameras[0];

  /**
   * @brief Print the ID of the selected camera.
   * @details `camera->id()` returns a string identifier for the camera
   * hardware.
   */
  std::cout << "Using camera: " << camera->id() << std::endl;

  /**
   * @brief Attempt to acquire exclusive access to the selected camera.
   * @details Acquiring the camera prevents other applications from using it
   * simultaneously.
   * @return Returns 0 on success, or a non-zero error code if the camera is
   * already in use or cannot be acquired.
   */
  if (camera->acquire()) {
    /**
     * @brief Handle failure to acquire the camera.
     * @details Print an error message, stop the CameraManager, and exit with
     * failure status.
     */
    std::cerr << "ERROR: Failed to acquire camera " << camera->id()
              << std::endl;
    cm->stop();           // Clean up CameraManager resources.
    return EXIT_FAILURE;  // Indicate program failure.
  }
  /**
   * @brief Print a success message indicating the camera was acquired.
   */
  std::cout << "Camera acquired." << std::endl;

  /**
   * @brief Generate a camera configuration suitable for video recording.
   * @details `camera->generateConfiguration()` creates a proposed
   * `libcamera::CameraConfiguration` based on the specified list of stream
   * roles. `libcamera::StreamRole::VideoRecording` indicates the intended use
   * case, helping libcamera choose appropriate default settings. The result is
   * a `std::unique_ptr<libcamera::CameraConfiguration>`.
   */
  auto config =
      camera->generateConfiguration({libcamera::StreamRole::VideoRecording});

  /**
   * @brief Check if configuration generation was successful.
   * @details If `config` is null or the configuration `config->empty()` is true
   * (meaning no streams were configured, which shouldn't happen for
   * VideoRecording if supported), it indicates failure.
   */
  if (!config || config->empty()) {
    /**
     * @brief Handle failure to generate the configuration.
     * @details Print an error, release the camera (allowing others to use it),
     * stop the CameraManager, and exit with failure status.
     */
    std::cerr << "ERROR: Failed to generate camera configuration" << std::endl;
    camera->release();    // Release the acquired camera.
    cm->stop();           // Stop the CameraManager.
    return EXIT_FAILURE;  // Indicate program failure.
  }
  /**
   * @brief Print a success message indicating base configuration was generated.
   */
  std::cout << "Generated base configuration." << std::endl;

  /**
   * @brief Get a reference to the configuration for the first (and likely only)
   * stream.
   * @details The generated configuration (`config`) contains one or more stream
   * configurations (`libcamera::StreamConfiguration`). For a simple
   * VideoRecording role, there's typically just one stream at index 0. We get a
   * reference to modify its properties (like pixel format and size).
   */
  auto &stream_config = config->at(0);

  // --- Begin User Format Selection ---
  /**
   * @brief Retrieve the list of supported pixel formats and sizes for this
   * stream configuration.
   * @details `stream_config.formats()` returns a `libcamera::StreamFormats`
   * object containing the hardware-supported combinations.
   */
  const auto &formats = stream_config.formats();

  /**
   * @brief Create a vector to store the available format/size options for user
   * selection.
   * @details Each element will be a pair containing the
   * `libcamera::PixelFormat` and `libcamera::Size`.
   */
  std::vector<std::pair<libcamera::PixelFormat, libcamera::Size>> options;

  /**
   * @brief Initialize an index counter for displaying options to the user.
   */
  int index = 0;

  /**
   * @brief Print a header for the list of supported formats.
   */
  std::cout << "Supported formats:\n";

  /**
   * @brief Iterate through all supported pixel formats for this stream.
   * @details `formats.pixelformats()` returns a list of
   * `libcamera::PixelFormat` supported by the camera for this stream
   * configuration.
   */
  for (const auto &fmt : formats.pixelformats()) {
    /**
     * @brief For each pixel format, iterate through all supported sizes
     * (resolutions).
     * @details `formats.sizes(fmt)` returns a list of `libcamera::Size` (width,
     * height) that are supported in combination with the current pixel format
     * `fmt`.
     */
    for (const auto &size : formats.sizes(fmt)) {
      /**
       * @brief Print the current option with its index, format, and size.
       * @details `fmt.toString()` gives a human-readable string for the pixel
       * format (e.g., "RGB888", "YUYV").
       */
      std::cout << index << ": " << fmt.toString() << " " << size.width << "x"
                << size.height << "\n";

      /**
       * @brief Add this format/size combination to the `options` vector.
       * @details `emplace_back` constructs the pair directly within the vector.
       */
      options.emplace_back(fmt, size);

      /**
       * @brief Increment the index for the next option.
       */
      index++;
    }
  }

  /**
   * @brief Declare an integer variable to store the user's choice.
   */
  int choice;

  /**
   * @brief Prompt the user to enter the index of the desired format.
   */
  std::cout << "Select format index: ";

  /**
   * @brief Read the user's input from the standard input stream (`std::cin`).
   */
  std::cin >> choice;

  /**
   * @brief Validate the user's input.
   * @details Checks if the input failed (e.g., non-numeric input,
   * 'std::cin.fail()'), if the choice is negative, or if the choice index is
   * out of bounds for the `options` vector.
   */
  if (std::cin.fail() || choice < 0 ||
      static_cast<size_t>(choice) >= options.size()) {
    /**
     * @brief Handle invalid user input.
     * @details Print an error, release the camera, stop the manager, and exit.
     */
    std::cerr << "ERROR: Invalid format selection." << std::endl;
    camera->release();
    cm->stop();
    return EXIT_FAILURE;
  }

  /**
   * @brief Set the chosen pixel format in the stream configuration.
   * @details The `pixelFormat` member of `stream_config` is updated with the
   * format selected by the user from the `options` vector.
   */
  stream_config.pixelFormat = options[choice].first;

  /**
   * @brief Set the chosen size (resolution) in the stream configuration.
   * @details The `size` member of `stream_config` is updated with the size
   * selected by the user from the `options` vector.
   */
  stream_config.size = options[choice].second;

  /**
   * @brief Print the selected format and size to confirm the user's choice.
   * @details `toString()` methods are used for readable output.
   */
  std::cout << "Selected format: " << stream_config.pixelFormat.toString()
            << " " << stream_config.size.toString() << std::endl;
  // --- End User Format Selection ---

  /**
   * @brief Validate the modified configuration.
   * @details `config->validate()` checks if the current settings in the
   * `config` object (including the user-selected format and size) are valid and
   * supported by the hardware. It might adjust some parameters slightly (e.g.,
   * buffer count) if necessary.
   * @return Returns `libcamera::CameraConfiguration::Valid` (or `Adjusted`) on
   * success, or `Invalid` on failure.
   */
  if (config->validate() == libcamera::CameraConfiguration::Invalid) {
    /**
     * @brief Handle configuration validation failure.
     * @details Print an error, release the camera, stop the manager, and exit.
     */
    std::cerr << "ERROR: Failed to validate configuration" << std::endl;
    camera->release();
    cm->stop();
    return EXIT_FAILURE;
  }

  /**
   * @brief Apply the validated configuration to the camera.
   * @details `camera->configure(config.get())` sends the configuration settings
   * to the camera hardware. Note that `config.get()` passes a raw pointer from
   * the unique_ptr.
   * @return Returns 0 on success, or a non-zero error code on failure.
   */
  if (camera->configure(config.get())) {
    /**
     * @brief Handle failure to apply the configuration.
     * @details Print an error, release the camera, stop the manager, and exit.
     */
    std::cerr << "ERROR: Failed to apply configuration" << std::endl;
    camera->release();
    cm->stop();
    return EXIT_FAILURE;
  }
  /**
   * @brief Print a success message indicating the camera is configured.
   */
  std::cout << "Camera configured." << std::endl;

  /**
   * @brief Get a pointer to the configured stream object.
   * @details `stream_config.stream()` returns a pointer to the
   * `libcamera::Stream` object that was created based on the `stream_config`.
   * This stream object is needed for buffer allocation and request creation.
   */
  libcamera::Stream *stream = stream_config.stream();

  /**
   * @brief Create a FrameBufferAllocator for the configured camera.
   * @details The `libcamera::FrameBufferAllocator` is responsible for
   * allocating the actual memory buffers (`libcamera::FrameBuffer`) needed to
   * store captured frames for the specified stream. It's initialized with the
   * camera object.
   */
  libcamera::FrameBufferAllocator allocator(camera);

  /**
   * @brief Allocate buffers for the configured stream.
   * @details `allocator.allocate(stream)` tells the allocator to allocate a
   * suitable number of buffers for the given stream. The number of buffers
   * depends on the configuration and driver requirements.
   * @return Returns the number of buffers allocated on success, or a negative
   * error code on failure.
   */
  if (allocator.allocate(stream) < 0) {
    /**
     * @brief Handle failure to allocate frame buffers.
     * @details Print an error, release the camera, stop the manager, and exit.
     */
    std::cerr << "ERROR: Failed to allocate frame buffers" << std::endl;
    camera->release();
    cm->stop();
    return EXIT_FAILURE;
  }

  /**
   * @brief Get the number of buffers actually allocated.
   * @details `allocator.buffers(stream)` returns a vector of
   * `libcamera::FrameBuffer*` that were allocated. `.size()` gives the count.
   */
  size_t num_buffers = allocator.buffers(stream).size();

  /**
   * @brief Print the number of allocated buffers.
   */
  std::cout << "Allocated " << num_buffers << " buffers." << std::endl;

  /**
   * @brief Create a vector to hold the camera request objects.
   * @details `libcamera::Request` objects encapsulate a capture request,
   * including which buffer(s) to use and any control settings. We use
   * `std::unique_ptr` to manage the lifetime of the requests. A pool of
   * requests (often matching the number of buffers) is typically created.
   */
  std::vector<std::unique_ptr<libcamera::Request>> requests;

  /**
   * @brief Iterate through the allocated frame buffers to create corresponding
   * requests.
   * @details For each allocated buffer, we create one `libcamera::Request` and
   * associate that buffer with it.
   */
  for (const auto &buffer : allocator.buffers(stream)) {
    /**
     * @brief Create a new camera request object.
     * @details `camera->createRequest()` creates an empty request. An optional
     * cookie (not used here) can be passed. Returns a
     * `std::unique_ptr<libcamera::Request>`.
     */
    auto req = camera->createRequest();

    /**
     * @brief Check if request creation failed or if adding the buffer to the
     * request failed.
     * @details `req` might be null if request creation fails.
     * `req->addBuffer(stream, buffer.get())` associates the current frame
     * buffer (`buffer.get()` gives the raw pointer) with this request for the
     * specified stream. It returns 0 on success or < 0 on failure.
     */
    if (!req || req->addBuffer(stream, buffer.get()) < 0) {
      /**
       * @brief Handle failure during request creation or buffer addition.
       * @details Print an error message.
       */
      std::cerr << "ERROR: Failed to create request or add buffer" << std::endl;

      /**
       * @brief Clean up already created requests before exiting.
       * @details `requests.clear()` destroys the unique_ptrs in the vector,
       * automatically deleting the Request objects they manage. This is
       * important because requests might hold references to buffers.
       */
      requests.clear();  // Release unique_ptrs, deleting associated Request
                         // objects.

      /**
       * @brief Free the allocated frame buffers.
       * @details `allocator.free(stream)` releases the buffers previously
       * allocated for this stream.
       */
      allocator.free(stream);

      /**
       * @brief Release the camera.
       */
      camera->release();

      /**
       * @brief Stop the CameraManager.
       */
      cm->stop();

      /**
       * @brief Exit with failure status.
       */
      return EXIT_FAILURE;
    }

    /**
     * @brief Add the newly created and configured request (managed by
     * unique_ptr) to the `requests` vector.
     * @details `std::move(req)` transfers ownership of the Request object from
     * the local `req` unique_ptr to the unique_ptr stored inside the vector.
     */
    requests.push_back(std::move(req));
  }  // End of loop creating requests

  /**
   * @brief Print the number of requests created (should match the number of
   * buffers).
   */
  std::cout << "Created " << requests.size() << " requests." << std::endl;

  /**
   * @brief Connect the `requestCompleted` signal from the camera object to our
   * `request_complete` callback function.
   * @details This establishes the asynchronous notification mechanism. Whenever
   * the camera finishes processing a request, it will emit the
   * `requestCompleted` signal, which in turn calls our `request_complete`
   * function with the completed request as an argument.
   * `libcamera::Signal::connect` is used for this purpose.
   */
  camera->requestCompleted.connect(&request_complete);

  /**
   * @brief Start the camera's capture process.
   * @details `camera->start()` enables the camera hardware and prepares it to
   * receive and process capture requests.
   * @return Returns 0 on success, or a non-zero error code on failure.
   */
  if (camera->start()) {
    /**
     * @brief Handle failure to start the camera.
     * @details Print an error, clean up requests, free buffers, release the
     * camera, stop the manager, and exit.
     */
    std::cerr << "ERROR: Failed to start camera" << std::endl;
    requests.clear();        // Cleanup requests
    allocator.free(stream);  // Free buffers
    camera->release();       // Release camera
    cm->stop();              // Stop manager
    return EXIT_FAILURE;     // Exit
  }
  /**
   * @brief Print a success message indicating the camera has started.
   */
  std::cout << "Camera started." << std::endl;

  /**
   * @brief Queue the initial set of requests to the camera.
   * @details The camera needs requests in its queue to start capturing frames.
   * We iterate through our pool of created requests and queue each one.
   */
  for (auto &req : requests) {
    /**
     * @brief Queue a single request to the camera.
     * @details `camera->queueRequest(req.get())` submits the request (passing
     * the raw pointer via `req.get()`) to the camera's processing queue. The
     * camera will pick up requests from this queue, capture data into the
     * associated buffer, and signal completion when done.
     * @return Returns 0 on success, or a negative error code on failure.
     */
    if (camera->queueRequest(req.get()) < 0) {
      /**
       * @brief Handle failure to queue a request.
       * @details This is a critical error during startup. Print message, stop
       * camera, clean up everything, and exit.
       */
      std::cerr << "ERROR: Failed to queue request" << std::endl;
      camera->stop();          // Stop the camera capturing process.
      requests.clear();        // Cleanup requests.
      allocator.free(stream);  // Free buffers.
      camera->release();       // Release camera.
      cm->stop();              // Stop manager.
      return EXIT_FAILURE;     // Exit.
    }
  }  // End of loop queuing initial requests

  /**
   * @brief Print a message indicating that the initial requests have been
   * successfully queued.
   */
  std::cout << "Initial requests queued." << std::endl;

  // ==========================================================================
  // //
  // ==                           MAIN CAPTURE LOOP                          ==
  // //
  // ==========================================================================
  // //

  /**
   * @brief Initialize a counter for the number of frames processed.
   */
  int frame_count = 0;

  /**
   * @brief Define the maximum number of frames to capture in this run.
   * @details The loop will terminate after capturing and saving this many
   * frames.
   */
  const int MAX_FRAMES = 240;  // Set desired frame limit

  /**
   * @brief Retrieve the stride (bytes per line) from the configured stream.
   * @details This value was determined during configuration and is needed by
   * `save_frame` to correctly interpret the buffer memory layout.
   */
  const uint32_t stride = stream_config.stride;

  /**
   * @brief Retrieve the frame width from the configured stream size.
   * @details Needed by `save_frame`.
   */
  const uint32_t width = stream_config.size.width;

  /**
   * @brief Retrieve the frame height from the configured stream size.
   * @details Needed by `save_frame`.
   */
  const uint32_t height = stream_config.size.height;

  /**
   * @brief Print a message indicating the start of the main capture loop.
   */
  std::cout << "Starting capture loop for " << MAX_FRAMES << " frames..."
            << std::endl;

  /**
   * @brief The main loop that processes completed frames until the desired
   * count is reached.
   * @details This loop waits for completed requests, processes them (saves
   * data), and re-queues them.
   */
  while (frame_count < MAX_FRAMES) {
    /**
     * @brief Declare a pointer to hold the dequeued completed request.
     * Initialize to null.
     */
    libcamera::Request *req = nullptr;

    /**
     * @brief Begin a critical section to safely access the shared request
     * queue.
     * @details A `std::unique_lock` is used here instead of `std::lock_guard`
     * because we need to use it with the `std::condition_variable::wait`. It
     * locks `request_mutex` upon construction.
     */
    {  // Scope for the unique_lock
      std::unique_lock<std::mutex> lock(request_mutex);

      /**
       * @brief Wait on the condition variable until the request queue is not
       * empty.
       * @details `request_cv.wait(lock, lambda)` performs the following
       * atomically:
       * 1. Checks the condition provided by the lambda function (`[] { return
       * !request_queue.empty(); }`).
       * 2. If the condition is true (queue is not empty), the wait returns
       * immediately.
       * 3. If the condition is false (queue is empty), it unlocks the `lock`
       * and puts the current thread to sleep.
       * 4. When the condition variable is notified (by
       * `request_cv.notify_one()` in `request_complete`), the thread wakes up,
       * re-acquires the `lock`, and re-checks the condition. If true, it
       * proceeds; otherwise, it goes back to sleep. This ensures that we don't
       * try to access an empty queue and avoids busy-waiting.
       */
      request_cv.wait(lock, [this] {
        return !request_queue.empty();
      });  // `this` capture not strictly needed here, but harmless habit.

      /**
       * @brief Dequeue the request from the front of the queue.
       * @details `request_queue.front()` gets a reference to the first element
       * (the oldest completed request). `request_queue.pop()` removes the first
       * element from the queue. The lock is still held, ensuring safe access.
       */
      req = request_queue.front();
      request_queue.pop();

    }  // The unique_lock 'lock' goes out of scope here, automatically releasing
       // the mutex.

    /**
     * @brief Check if a valid request pointer was dequeued.
     * @details Although the wait condition ensures the queue wasn't empty, this
     * is a defensive check.
     */
    if (req) {
      /**
       * @brief Print a message indicating which frame number is being
       * processed.
       */
      std::cout << "Processing frame " << frame_count << "..." << std::endl;

      // --- Begin Timestamp Extraction and Saving ---
      /**
       * @brief Initialize the timestamp variable with a default value
       * indicating "not found".
       * @details Using -1 is a common convention for an invalid or missing
       * timestamp.
       */
      int64_t capture_timestamp_ns = -1;  // Default to -1 (invalid/missing)

      /**
       * @brief Get the metadata associated with the completed request.
       * @details `req->metadata()` returns a `libcamera::ControlList`
       * containing various metadata values captured alongside the frame (e.g.,
       * timestamp, exposure time, gain).
       */
      const libcamera::ControlList &metadata = req->metadata();

      /**
       * @brief Attempt to retrieve the SensorTimestamp from the metadata using
       * std::optional for safety.
       * @details `metadata.get<T>(ControlId)` attempts to find and return the
       * value for the specified control ID
       * (`libcamera::controls::SensorTimestamp`) with the expected type
       * (`int64_t`). Using `std::optional<int64_t>` means `ts_opt` will either
       * contain the timestamp value if found, or be empty (`std::nullopt`) if
       * the timestamp control is not present in the metadata for this request.
       * This avoids potential exceptions or undefined behavior if the control
       * is missing.
       */
      std::optional<int64_t> ts_opt =
          metadata.get<int64_t>(libcamera::controls::SensorTimestamp);

      /**
       * @brief Check if the optional object contains a value (i.e., the
       * timestamp was found).
       * @details `ts_opt.has_value()` returns true if `metadata.get`
       * successfully retrieved the timestamp.
       */
      if (ts_opt.has_value()) {
        /**
         * @brief Extract the timestamp value from the optional object.
         * @details `ts_opt.value()` returns the contained `int64_t` timestamp.
         * This is only safe to call if `has_value()` is true.
         */
        capture_timestamp_ns = ts_opt.value();

        /**
         * @brief Call the helper function to save the retrieved timestamp to
         * its file.
         * @details Pass the timestamp value and the current frame count.
         * @return `true` on success, `false` on failure.
         */
        if (!save_timestamp(capture_timestamp_ns, frame_count)) {
          /**
           * @brief Handle failure to save the timestamp.
           * @details Print a warning message. Depending on requirements, the
           * application could choose to stop or continue without the timestamp
           * file for this frame. Here, we just warn.
           */
          std::cerr << "Warning: Failed to save timestamp for frame "
                    << frame_count << std::endl;
          // Decide if you want to `break;` or continue without the timestamp
          // file. Continuing.
        } else {
          /**
           * @brief Optional: Print the saved timestamp for debugging purposes.
           * (Currently commented out)
           */
          // std::cout << "  Timestamp: " << capture_timestamp_ns << " ns" <<
          // std::endl; // Debug output
        }
      } else {
        /**
         * @brief Handle the case where the SensorTimestamp control was not
         * found in the metadata.
         * @details This might happen on some platforms or configurations. Print
         * a warning.
         */
        std::cerr << "Warning: SensorTimestamp metadata not found for frame "
                  << frame_count << std::endl;

        /**
         * @brief Save a placeholder value (-1) to the timestamp file.
         * @details This indicates explicitly in the output file that the
         * timestamp was missing for this frame, which might be more informative
         * than having no file at all. Error checking is still performed.
         */
        if (!save_timestamp(-1, frame_count)) {
          std::cerr
              << "Warning: Failed to save placeholder timestamp for frame "
              << frame_count << std::endl;
        }
      }
      // --- End Timestamp Extraction and Saving ---

      // --- Begin Frame Data Saving ---
      /**
       * @brief Get the map of buffers associated with the completed request.
       * @details `req->buffers()` returns a map where the key is the
       * `libcamera::Stream*` and the value is the `libcamera::FrameBuffer*`
       * containing the captured data for that stream.
       */
      const auto &buffers = req->buffers();

      /**
       * @brief Check if the buffer map is unexpectedly empty.
       * @details A completed request should normally have buffers associated
       * with it.
       */
      if (buffers.empty()) {
        /**
         * @brief Warn if the request has no buffers. This is unusual.
         */
        std::cerr << "Warning: Request for frame " << frame_count
                  << " has no buffers." << std::endl;
        // Consider how to handle this - skip saving? depends on application
        // logic. Here, we'll skip saving.
      } else {
        /**
         * @brief Get the frame buffer corresponding to our configured stream.
         * @details `buffers.at(stream)` retrieves the `libcamera::FrameBuffer*`
         * associated with the `stream` we configured earlier. This buffer
         * contains the actual pixel data. Using `.at()` will throw an exception
         * if the key (`stream`) is not found; assuming it must be present for a
         * valid request.
         */
        libcamera::FrameBuffer *buffer = buffers.at(stream);

        /**
         * @brief Call the helper function to save the frame data from the
         * buffer's planes.
         * @details Pass the buffer's planes (`buffer->planes()`), frame count,
         * and the pre-calculated stride, width, and height.
         * @return `true` on success, `false` on failure.
         */
        if (!save_frame(buffer->planes(), frame_count, stride, width, height)) {
          /**
           * @brief Handle critical failure to save the frame data.
           * @details Print an error message. Since saving the frame is the core
           * purpose, we choose to stop the capture loop on failure.
           */
          std::cerr << "ERROR: Failed to save frame " << frame_count
                    << ", stopping." << std::endl;

          /**
           * @brief Reuse the request's buffer(s) even though saving failed.
           * @details It's generally necessary to call `req->reuse()` before the
           * request object is destroyed or goes out of scope, otherwise the
           * associated buffer might not be properly returned to the allocator's
           * pool. We reuse it here before breaking the loop.
           */
          req->reuse(libcamera::Request::ReuseBuffers);

          /**
           * @brief Exit the main capture loop prematurely due to the error.
           */
          break;  // Exit the while loop.
        }
        /**
         * @brief Optional: Print a message indicating frame data was saved.
         * (Currently commented out)
         */
        // std::cout << "  Saved frame data." << std::endl; // Debug output
      }
      // --- End Frame Data Saving ---

      /**
       * @brief Increment the frame counter only after attempting to process the
       * frame.
       * @details This ensures the count reflects the number of frames processed
       * (or attempted), regardless of minor non-fatal errors like missing
       * timestamps. It's placed after the critical `save_frame` check.
       */
      frame_count++;

      /**
       * @brief Mark the request's buffers as available for reuse.
       * @details `req->reuse(libcamera::Request::ReuseBuffers)` signals to
       * libcamera that the application is finished with the data in the
       * buffer(s) associated with this request, and they can be reused for
       * subsequent captures. This is essential for continuous capture.
       */
      req->reuse(libcamera::Request::ReuseBuffers);

      /**
       * @brief Re-queue the request to the camera for the next capture cycle.
       * @details Submitting the request again allows the camera to capture
       * another frame into the associated buffer once it becomes available.
       * @return 0 on success, negative error code on failure.
       */
      if (camera->queueRequest(req) < 0) {
        /**
         * @brief Handle failure to re-queue the request.
         * @details This indicates a problem with the camera's ongoing
         * operation. Print an error and exit the capture loop.
         */
        std::cerr << "ERROR: Failed to re-queue request, stopping."
                  << std::endl;
        break;  // Exit the while loop.
      }
    }  // End if(req)
  }  // End while loop (main capture loop)

  /**
   * @brief Print a message indicating the capture loop has finished.
   * @details Reports the total number of frames processed before the loop
   * terminated (either by reaching `MAX_FRAMES` or due to an error).
   */
  std::cout << "Capture loop finished after " << frame_count << " frames."
            << std::endl;

  // ==========================================================================
  // //
  // ==                               CLEANUP                                ==
  // //
  // ==========================================================================
  // //

  /**
   * @brief Print a message indicating the start of the cleanup phase.
   */
  std::cout << "Stopping camera..." << std::endl;

  /**
   * @brief Stop the camera's capture process.
   * @details `camera->stop()` signals the camera to cease capturing frames and
   * processing requests. Any requests currently being processed will likely
   * complete, but no new requests will be started. Some completed requests
   * might still be delivered shortly after stop() returns. It may also cancel
   * pending queued requests.
   * @return 0 on success, non-zero on failure (though cleanup often proceeds
   * regardless).
   */
  camera->stop();  // Stop processing new requests.

  /**
   * @brief Optional: Wait for any potentially outstanding requests to complete.
   * @details After `camera->stop()`, some requests might still be in flight. A
   * robust application might implement a mechanism to wait until all initially
   * queued requests have either completed or been cancelled, ensuring all
   * buffers are accounted for before proceeding with deallocation. This simple
   * example omits explicit waiting.
   */
  // TODO: Implement wait for pending requests if needed.

  /**
   * @brief Disconnect the request completion signal handler.
   * @details It's good practice to disconnect signals when they are no longer
   * needed, especially before destroying the objects involved.
   * `camera->requestCompleted.disconnect(&request_complete)` removes the
   * connection established earlier.
   */
  camera->requestCompleted.disconnect(&request_complete);

  /**
   * @brief Clear any remaining completed requests from the queue and release
   * request objects.
   * @details Although the loop processed requests, stopping might leave some
   * completed requests in the `request_queue` or cause cancellations. We need
   * to ensure all `Request` objects are properly managed.
   */
  /**
   * @brief Acquire lock to safely access the request queue during cleanup.
   */
  std::unique_lock<std::mutex> lock(
      request_mutex);  // Using unique_lock for consistency, though guard would
                       // work.
  /**
   * @brief Drain any requests remaining in the queue.
   * @details This loop removes pointers from the queue. Since `req->reuse()`
   * was called in the main loop (and should ideally be called for any cancelled
   * requests too), the buffers associated with these requests should have been
   * returned to the allocator pool. We don't need to do anything with the `req`
   * pointers themselves here other than remove them from our queue.
   */
  while (!request_queue.empty()) {
    libcamera::Request *req_to_discard = request_queue.front();  // Get pointer
    request_queue.pop();  // Remove from queue
    // We don't need to delete req_to_discard; ownership is managed by the
    // unique_ptrs in the 'requests' vector.
  }
  /**
   * @brief Release the mutex before potentially longer operations.
   */
  lock.unlock();

  /**
   * @brief Clear the vector of request unique_ptrs.
   * @details This is the crucial step for releasing the `libcamera::Request`
   * objects themselves. When the `std::unique_ptr`s in the `requests` vector
   * are destroyed (as `requests.clear()` does), their destructors are called,
   * which in turn delete the `libcamera::Request` objects they manage.
   * Libcamera's Request object destructor should handle internal cleanup.
   */
  requests.clear();  // Destroys all unique_ptrs, thereby deleting the Request
                     // objects.

  /**
   * @brief Print message indicating camera release is starting.
   */
  std::cout << "Releasing camera..." << std::endl;

  /**
   * @brief Free the allocated frame buffers.
   * @details `allocator.free(stream)` tells the allocator to release all
   * buffers previously allocated for this stream. This must be done *after*
   * ensuring no requests are using the buffers (which clearing the `requests`
   * vector helps guarantee, assuming proper `reuse` calls).
   * @return 0 on success, negative on error.
   */
  allocator.free(stream);  // Free the buffers associated with the stream.

  /**
   * @brief Release the exclusive access to the camera device.
   * @details `camera->release()` makes the camera available for other
   * applications or processes to use.
   * @return 0 on success, non-zero on error.
   */
  camera->release();

  /**
   * @brief Print message indicating CameraManager stop is starting.
   */
  std::cout << "Stopping CameraManager..." << std::endl;

  /**
   * @brief Stop the CameraManager.
   * @details `cm->stop()` releases resources held by the CameraManager itself.
   * This should be called before the `CameraManager` object is destroyed.
   * @return 0 on success, non-zero on error.
   */
  cm->stop();

  /**
   * @brief Print final cleanup completion message.
   */
  std::cout << "Cleanup complete." << std::endl;

  /**
   * @brief Return success status code.
   * @details Indicates that the program finished its intended execution
   * successfully. The `unique_ptr cm` will also go out of scope here, calling
   * the CameraManager destructor if `stop()` hadn't already been called.
   */
  return EXIT_SUCCESS;
}

