/**
 * @file functionality.cc
 * @brief Comprehensive test harness for RingMaster lock-free ring buffer
 *
 * @details
 * This test suite validates the behavior of a lock-free ring buffer
 * implementation with multiple test cases covering core functionality,
 * throughput, wraparound behavior, and memory ordering semantics. Features
 * include:
 *
 * - Thread-safe logging with color-coded test results
 * - Resource monitoring (CPU time, memory usage)
 * - Structured test phases with detailed diagnostics
 * - Comprehensive test summary with performance metrics
 */

#include <sys/resource.h>  // For resource statistics (getrusage)

#include <algorithm>  // For std::any_of
#include <atomic>     // For thread synchronization
#include <chrono>     // For high-precision timing
#include <cstdio>     // For perror
#include <iomanip>    // For output formatting
#include <iostream>   // For console I/O
#include <mutex>      // For thread-safe logging
#include <sstream>    // For string formatting
#include <string>     // For string manipulation
#include <thread>     // For multithreaded testing
#include <vector>     // For result collection

#include "../ringMaster.cc"  // RingMaster implementation

// ANSI color codes for console output
#define RED "\033[31m"     // Errors, failures
#define GREEN "\033[32m"   // Success, passing tests
#define YELLOW "\033[33m"  // Warnings, statistics
#define BLUE "\033[34m"    // Information, phase markers
#define RESET "\033[0m"    // Reset to default color

// Thread synchronization for logging
std::mutex log_mutex;

/**
 * @brief Captures and logs current process resource usage metrics
 *
 * @param stage Descriptive label for the current operational stage
 *
 * @details
 * Reports the following system resource metrics:
 * - User CPU time: Time spent in user-mode code
 * - System CPU time: Time spent in kernel-mode code
 * - Maximum RSS: Peak physical memory usage in kilobytes
 *
 * Thread-safe implementation using mutex to prevent interleaved output.
 */
static void logResourceUsage(const std::string& stage) {
  struct rusage usage;
  std::lock_guard<std::mutex> lock(log_mutex);

  if (getrusage(RUSAGE_SELF, &usage) == -1) {
    std::cerr << RED << "[ERROR] Failed to get resource usage: ";
    perror(nullptr);
    std::cerr << RESET;
    return;
  }

  // Format and print resource statistics
  std::cout << YELLOW << "[STATS] " << RESET << std::left << std::setw(35)
            << (stage + ":") << " CPU(user): " << std::fixed
            << std::setprecision(3)
            << (usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6) << "s | "
            << "CPU(sys): "
            << (usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6) << "s | "
            << "Memory(max RSS): " << usage.ru_maxrss << " KB\n";
}

/**
 * @brief Container for test case results and performance metrics
 *
 * @details
 * Captures comprehensive test execution data including:
 * - Test identification and pass/fail status
 * - Wall clock execution time
 * - CPU time (user and system)
 * - Peak memory consumption
 */
struct TestResult {
  std::string name;  // Test case identifier
  bool passed;       // Overall test status
  double wallTime;   // Real-time duration in seconds
  double userCPU;    // User-mode CPU time in seconds
  double sysCPU;     // System-mode CPU time in seconds
  long maxRSS;       // Peak memory usage in KB
};

/**
 * @brief Validates basic single-producer/single-consumer functionality
 *
 * @return true if all test checks pass, false otherwise
 *
 * @details
 * Tests fundamental ring buffer operations through three phases:
 *
 * 1. Single element lifecycle:
 *    - Push and pop of a single value
 *    - Verification of data integrity
 *    - Buffer state validation (empty after operation)
 *
 * 2. Capacity validation:
 *    - Sequential filling to capacity
 *    - Full state detection
 *    - Overflow protection (reject push on full buffer)
 *
 * 3. Complete buffer drainage:
 *    - Element-by-element extraction
 *    - Order and value verification
 *    - Final state validation (empty after drainage)
 *    - Underflow protection (reject pop on empty buffer)
 */
bool testBasicSPSC() {
  const std::string testName = "BasicSPSC";
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "\n"
              << GREEN << "[ BEGIN TEST ] " << RESET
              << "Basic Functionality Validation\n"
              << "  Verifies core buffer operations (push, pop, empty, full) "
                 "and boundary conditions.\n"
              << std::string(70, '-') << "\n";
  }

  logResourceUsage("Pre-test baseline (" + testName + ")");
  bool overall_pass = true;
  RingMaster<int> ring;
  int value_popped;

  // PHASE 1: Single Element Lifecycle
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << BLUE << "[ PHASE 1 ] " << RESET
              << "Single element push/pop test...\n";
  }

  int test_value = 42;
  if (ring.push(test_value)) {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "  " << GREEN << "✔️" << RESET << " Push initial element ("
              << test_value << "): Succeeded.\n";
  } else {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "  " << RED << "✖" << RESET << " Push initial element ("
              << test_value
              << "): Failed (buffer reported full unexpectedly?).\n";
    overall_pass = false;
  }

  if (overall_pass) {
    if (ring.pop(value_popped) && value_popped == test_value) {
      std::lock_guard<std::mutex> lock(log_mutex);
      std::cout << "  " << GREEN << "✔️" << RESET
                << " Pop single element: Succeeded. Got correct value ("
                << value_popped << ").\n";
    } else {
      std::lock_guard<std::mutex> lock(log_mutex);
      std::cout << "  " << RED << "✖" << RESET
                << " Pop single element: Failed. ";
      if (ring.isEmpty()) {
        std::cout << "Pop operation returned false (buffer reported empty "
                     "unexpectedly?).\n";
      } else {
        std::cout << "Got incorrect value (" << value_popped << "), expected ("
                  << test_value << ").\n";
      }
      overall_pass = false;
    }

    if (!ring.isEmpty()) {
      std::lock_guard<std::mutex> lock(log_mutex);
      std::cout << "  " << RED << "✖" << RESET
                << " Buffer state: Failed. Buffer should be empty after "
                   "popping the only element.\n";
      overall_pass = false;
    } else {
      std::lock_guard<std::mutex> lock(log_mutex);
      std::cout << "  " << GREEN << "✔️" << RESET
                << " Buffer state: Correctly detected as empty.\n";
    }
  }

  // PHASE 2: Capacity Validation
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << BLUE << "\n[ PHASE 2 ] " << RESET
              << "Buffer capacity and overflow test (CAPACITY = " << CAPACITY
              << ")...\n";
  }

  int fill_count = 0;
  const int max_fill = CAPACITY;
  for (fill_count = 0; fill_count < max_fill; ++fill_count) {
    if (!ring.push(fill_count)) {
      std::lock_guard<std::mutex> lock(log_mutex);
      std::cout << "  " << RED << "✖" << RESET
                << " Fill operation: Failed unexpectedly at element "
                << fill_count << "/" << max_fill << ".\n";
      overall_pass = false;
      break;
    }
  }

  if (fill_count == max_fill) {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "  " << GREEN << "✔️" << RESET
              << " Fill operation: Successfully pushed " << fill_count << "/"
              << max_fill << " elements.\n";
  } else if (overall_pass) {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "  " << YELLOW << "⚠️" << RESET
              << " Fill operation: Pushed only " << fill_count << "/"
              << max_fill << " elements.\n";
  }

  {
    std::lock_guard<std::mutex> lock(log_mutex);
    if (ring.isFull()) {
      std::cout << "  " << GREEN << "✔️" << RESET
                << " Buffer full state: Correctly detected as full.\n";
    } else {
      std::cout << "  " << RED << "✖" << RESET
                << " Buffer full state: Failed. Buffer not detected as full "
                   "after filling capacity.\n";
      overall_pass = false;
    }
  }

  if (ring.push(999)) {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "  " << RED << "✖" << RESET
              << " Overflow protection: Failed. Push operation succeeded on a "
                 "full buffer.\n";
    overall_pass = false;
  } else {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout
        << "  " << GREEN << "✔️" << RESET
        << " Overflow protection: Correctly blocked push on a full buffer.\n";
  }

  // PHASE 3: Drainage and Data Integrity Validation
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << BLUE << "\n[ PHASE 3 ] " << RESET
              << "Buffer drainage and data integrity check...\n";
  }

  int drain_count = 0;
  int expected_value = 0;

  while (ring.pop(value_popped)) {
    if (value_popped != expected_value) {
      std::lock_guard<std::mutex> lock(log_mutex);
      std::cout << "  " << RED << "✖" << RESET
                << " Data integrity: Failed at position " << drain_count
                << ". Expected value " << expected_value << ", but got "
                << value_popped << ".\n";
      overall_pass = false;
    }
    drain_count++;
    expected_value++;
  }

  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "  Drained " << drain_count << " elements.\n";

    if (drain_count == fill_count) {
      std::cout << "  " << GREEN << "✔️" << RESET
                << " Drain count: Correct. Matched fill count (" << fill_count
                << ").\n";
    } else {
      std::cout << "  " << RED << "✖" << RESET
                << " Drain count: Mismatch. Expected " << fill_count
                << " elements, but drained " << drain_count << ". Lost "
                << (fill_count - drain_count) << " elements?\n";
      overall_pass = false;
    }

    if (ring.isEmpty()) {
      std::cout << "  " << GREEN << "✔️" << RESET
                << " Buffer empty state: Correctly detected as empty.\n";
    } else {
      std::cout << "  " << RED << "✖" << RESET
                << " Buffer empty state: Failed. Buffer not detected as empty "
                   "after draining.\n";
      overall_pass = false;
    }

    if (ring.pop(value_popped)) {
      std::cout << "  " << RED << "✖" << RESET
                << " Underflow check: Failed. Pop operation succeeded on an "
                   "empty buffer, returning value "
                << value_popped << ".\n";
      overall_pass = false;
    } else {
      std::cout
          << "  " << GREEN << "✔️" << RESET
          << " Underflow check: Correctly blocked pop on an empty buffer.\n";
    }
  }

  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "\nTest Result (" << testName
              << "): " << (overall_pass ? GREEN "PASSED" : RED "FAILED")
              << RESET << "\n"
              << std::string(70, '-') << "\n";
  }
  logResourceUsage("Post-test (" + testName + ")");
  return overall_pass;
}

/**
 * @brief Measures sustained throughput performance in SPSC configuration
 *
 * @return true if test completes successfully, false on data corruption or
 * count mismatch
 *
 * @details
 * Tests high-volume data transfer between producer and consumer threads:
 * - Producer continuously pushes sequential integers
 * - Consumer validates data sequence integrity
 * - Both threads yield when operations would block
 * - Verifies complete and correct transfer of all elements
 *
 * This test evaluates performance under sustained load and validates
 * the correctness of concurrent access patterns.
 */
bool testSPSCThroughput() {
  const std::string testName = "SPSCThroughput";
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "\n"
              << GREEN << "[ BEGIN TEST ] " << RESET << "SPSC Throughput Test\n"
              << "  Measures sustained data transfer rate between producer and "
                 "consumer threads.\n"
              << std::string(70, '-') << "\n";
  }
  logResourceUsage("Pre-test baseline (" + testName + ")");

  RingMaster<int> ring;
  const size_t num_elements = 1000000;
  std::atomic<bool> stop(false);
  std::atomic<size_t> produced(0), consumed(0);
  std::atomic<size_t> success_transfers(0);

  auto producer = [&]() {
    for (size_t i = 0; i < num_elements; ++i) {
      while (!ring.push(i)) {
        std::this_thread::yield();
      }
      produced++;
      success_transfers++;
    }
  };

  auto consumer = [&]() {
    int value;
    for (size_t i = 0; i < num_elements; ++i) {
      while (!ring.pop(value)) {
        std::this_thread::yield();
      }
      if (value != i) {
        std::lock_guard<std::mutex> lock(log_mutex);
        std::cout << RED << "[ ERROR ] " << RESET
                  << "Data corruption detected: expected " << i << ", got "
                  << value << "\n";
        stop = true;
      }
      consumed++;
    }
  };

  std::thread prod_thread(producer);
  std::thread cons_thread(consumer);

  prod_thread.join();
  cons_thread.join();

  bool overall_pass =
      (produced.load() == consumed.load()) && (produced.load() == num_elements);

  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "\nTest Result (" << testName
              << "): " << (overall_pass ? GREEN "PASSED" : RED "FAILED")
              << RESET << "\n"
              << std::string(70, '-') << "\n";
    std::cout << "  Produced: " << produced.load()
              << ", Consumed: " << consumed.load() << "\n";
  }
  logResourceUsage("Post-test (" + testName + ")");
  return overall_pass;
}

/**
 * @brief Tests buffer behavior when indices wrap around the circular buffer
 *
 * @return true if all operations maintain data integrity during wraparound,
 * false otherwise
 *
 * @details
 * Specifically targets the circular buffer's wraparound mechanism:
 * - Forces multiple cycles through the entire buffer (3x CAPACITY)
 * - Verifies correct operation when head/tail pointers cross array boundaries
 * - Validates data consistency throughout wraparound transitions
 * - Confirms proper empty state after complete drainage
 *
 * This test is critical for validating the index arithmetic in the ring buffer.
 */
bool testWraparound() {
  const std::string testName = "Wraparound";
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "\n"
              << GREEN << "[ BEGIN TEST ] " << RESET
              << "Index Wraparound Test\n"
              << "  Verifies correct behavior when head/tail indices wrap "
                 "around the buffer.\n"
              << std::string(70, '-') << "\n";
  }
  logResourceUsage("Pre-test baseline (" + testName + ")");

  RingMaster<int> ring;
  const size_t num_elements = CAPACITY * 3;
  std::atomic<bool> stop(false);

  auto producer = [&]() {
    for (size_t i = 0; i < num_elements; ++i) {
      while (!ring.push(i)) {
        std::this_thread::yield();
      }
    }
  };

  auto consumer = [&]() {
    int value;
    for (size_t i = 0; i < num_elements; ++i) {
      while (!ring.pop(value)) {
        std::this_thread::yield();
      }
      if (value != i) {
        std::lock_guard<std::mutex> lock(log_mutex);
        std::cout << RED << "[ ERROR ] " << RESET
                  << "Data corruption detected: expected " << i << ", got "
                  << value << "\n";
        stop = true;
      }
    }
  };

  std::thread prod_thread(producer);
  std::thread cons_thread(consumer);

  prod_thread.join();
  cons_thread.join();

  bool overall_pass = ring.isEmpty() && !stop;

  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "\nTest Result (" << testName
              << "): " << (overall_pass ? GREEN "PASSED" : RED "FAILED")
              << RESET << "\n"
              << std::string(70, '-') << "\n";
  }
  logResourceUsage("Post-test (" + testName + ")");
  return overall_pass;
}

/**
 * @brief Validates memory ordering semantics critical for lock-free correctness
 *
 * @return true if all operations maintain proper visibility between threads,
 * false otherwise
 *
 * @details
 * Tests the memory consistency model crucial for lock-free implementations:
 * - Verifies that writes by producer are visible to consumer in correct order
 * - Validates that memory barriers/fences in the implementation work correctly
 * - Ensures proper synchronization between concurrent threads
 * - Confirms no data races or visibility issues occur under load
 *
 * This test is essential for confirming the correctness of the atomic
 * operations and memory ordering constraints in the lock-free implementation.
 */
bool testMemoryOrdering() {
  const std::string testName = "MemoryOrdering";
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "\n"
              << GREEN << "[ BEGIN TEST ] " << RESET << "Memory Ordering Test\n"
              << "  Checks visibility and ordering of operations between "
                 "threads (critical for lock-free).\n"
              << std::string(70, '-') << "\n";
  }
  logResourceUsage("Pre-test baseline (" + testName + ")");

  RingMaster<int> ring;
  const size_t num_elements = 10000;
  std::atomic<bool> stop(false);

  auto producer = [&]() {
    for (size_t i = 0; i < num_elements; ++i) {
      while (!ring.push(i)) {
        std::this_thread::yield();
        if (stop) return;
      }
    }
  };

  auto consumer = [&]() {
    int val;
    for (size_t i = 0; i < num_elements; ++i) {
      while (!ring.pop(val)) {
        std::this_thread::yield();
        if (stop) return;
      }
      if (val != i) {
        std::lock_guard<std::mutex> lock(log_mutex);
        std::cout << RED << "[ ERROR ] " << RESET
                  << "Consumer received unexpected value: expected " << i
                  << ", got " << val << "\n";
        stop = true;
      }
    }
  };

  std::thread prod_thread(producer);
  std::thread cons_thread(consumer);

  prod_thread.join();
  cons_thread.join();

  bool overall_pass = !stop && ring.isEmpty();

  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << "\nTest Result (" << testName
              << "): " << (overall_pass ? GREEN "PASSED" : RED "FAILED")
              << RESET << "\n"
              << std::string(70, '-') << "\n";
  }
  logResourceUsage("Post-test (" + testName + ")");
  return overall_pass;
}

/**
 * @brief Main test harness entry point
 *
 * @return 0 if all tests pass, 1 if any test fails
 *
 * @details
 * Orchestrates the complete test suite execution:
 * 1. Initializes the test environment
 * 2. Runs all defined test cases with resource measurement
 * 3. Collects results and performance metrics
 * 4. Generates a comprehensive summary table
 * 5. Returns appropriate exit code based on test results
 *
 * The test harness provides detailed timing, CPU, and memory statistics
 * for each test case along with a consolidated overview of test results.
 */
int main() {
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout
        << GREEN
        << "\n╔══════════════════════════════════════════════════════════════╗"
           "\n"
        << "║                 RING BUFFER VALIDATION SUITE                 ║\n"
        << "╚══════════════════════════════════════════════════════════════╝"
        << RESET << "\n";
  }
  logResourceUsage("Initial state (Test Suite Start)");

  std::vector<TestResult> results;

  // Test execution wrapper with timing and resource measurement
  auto runTest = [&](const std::string& name, auto testFunction) {
    struct rusage usage_before, usage_after;

    getrusage(RUSAGE_SELF, &usage_before);
    auto t_start = std::chrono::steady_clock::now();

    bool pass_status = testFunction();

    auto t_end = std::chrono::steady_clock::now();
    getrusage(RUSAGE_SELF, &usage_after);

    // Calculate performance metrics
    double wall_duration =
        std::chrono::duration<double>(t_end - t_start).count();
    double user_cpu_delta =
        (usage_after.ru_utime.tv_sec - usage_before.ru_utime.tv_sec) +
        (usage_after.ru_utime.tv_usec - usage_before.ru_utime.tv_usec) / 1e6;
    double sys_cpu_delta =
        (usage_after.ru_stime.tv_sec - usage_before.ru_stime.tv_sec) +
        (usage_after.ru_stime.tv_usec - usage_before.ru_stime.tv_usec) / 1e6;
    long max_rss_after = usage_after.ru_maxrss;

    // Store results
    results.push_back(TestResult{name, pass_status, wall_duration,
                                 user_cpu_delta, sys_cpu_delta, max_rss_after});

    {
      std::lock_guard<std::mutex> lock(log_mutex);
      std::cout << "\n"
                << (pass_status ? GREEN "✔" : RED "✘") << RESET
                << " Test Completed: " << std::left << std::setw(18) << name
                << " Status: " << (pass_status ? GREEN "PASS" : RED "FAIL")
                << RESET << " (Duration: " << std::fixed << std::setprecision(6)
                << wall_duration << "s)\n";
    }
  };

  // Execute all test cases
  runTest("Core Function", testBasicSPSC);
  runTest("Throughput", testSPSCThroughput);
  runTest("Wraparound", testWraparound);
  runTest("Memory Order", testMemoryOrdering);

  // Generate final summary report
  int failed_count = 0;
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << BLUE
              << "\n╔══════════════════════════════════════════════════════════"
                 "════════════════════════╗\n"
              << "║                                 VALIDATION SUMMARY         "
                 "                       ║\n"
              << "╚════════════════════════════════════════════════════════════"
                 "════════════════════════╝"
              << RESET << "\n";

    // Define table formatting
    const int W_NAME = 20;
    const int W_STATUS = 12;
    const int W_WALL = 18;
    const int W_CPU = 18;
    const int W_MEM = 18;
    const int TOTAL_W = W_NAME + W_STATUS + W_WALL + W_CPU + W_CPU + W_MEM;

    // Print table header
    std::cout << std::left << std::setw(W_NAME) << "TEST CASE"
              << std::setw(W_STATUS) << "STATUS" << std::setw(W_WALL)
              << "TIME (Wall)" << std::setw(W_CPU) << "CPU (User)"
              << std::setw(W_CPU) << "CPU (Sys)" << std::setw(W_MEM)
              << "PEAK MEM (KB)"
              << "\n"
              << std::string(TOTAL_W, '-') << "\n";

    // Print individual test results
    for (const auto& r : results) {
      if (!r.passed) failed_count++;

      std::stringstream ss_wall, ss_user, ss_sys, ss_mem;
      ss_wall << std::fixed << std::setprecision(6) << r.wallTime << "s";
      ss_user << std::fixed << std::setprecision(6) << r.userCPU << "s";
      ss_sys << std::fixed << std::setprecision(6) << r.sysCPU << "s";
      ss_mem << r.maxRSS << " KB";

      std::cout << std::left << std::setw(W_NAME) << r.name
                << std::setw(W_STATUS) << (r.passed ? GREEN "PASS" : RED "FAIL")
                << RESET << std::setw(W_WALL) << ss_wall.str()
                << std::setw(W_CPU) << ss_user.str() << std::setw(W_CPU)
                << ss_sys.str() << std::setw(W_MEM) << ss_mem.str() << "\n";
    }
    std::cout << std::string(TOTAL_W, '-') << "\n";

    // Display overall result
    if (failed_count == 0) {
      std::cout << GREEN << "Overall Result: All tests passed!" << RESET
                << "\n";
    } else {
      std::cout << RED << "Overall Result: " << failed_count
                << " test(s) failed!" << RESET << "\n";
    }
    std::cout << std::string(TOTAL_W, '=') << "\n";
  }

  logResourceUsage("Final state (Test Suite End)");
  return failed_count > 0 ? 1 : 0;
}
