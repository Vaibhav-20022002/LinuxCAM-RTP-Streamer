#include <stdio.h>
#include <sys/time.h>

// ************************* LOGS *************************

#define COLOR_RESET "\033[0m"

// Reds
#define COLOR_RED_DARK  "\033[31m" // ERROR
#define COLOR_RED_LIGHT "\033[91m" // FAIL

// Others
#define COLOR_YELLOW  "\033[33m" // WARN
#define COLOR_GREEN   "\033[32m" // INFO
#define COLOR_CYAN    "\033[36m" // HIGH
#define COLOR_MAGENTA "\033[35m" // DEBUG

// Define numeric log levels (must exist at compile-time)
#define LOG_LEVEL_NONE  0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_FAIL  2
#define LOG_LEVEL_WARN  3
#define LOG_LEVEL_INFO  4
#define LOG_LEVEL_HIGH  5
#define LOG_LEVEL_DEBUG 6

// If LOG_LEVEL is not defined (e.g., not passed via -D), default to FAIL
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_FAIL
#endif

#if LOG_LEVEL >= LOG_LEVEL_FAIL
#define FAIL_MSG(fmt, ...) \
  fprintf(stderr, COLOR_RED_LIGHT "[FAIL]" COLOR_RESET "  " fmt "\n", ##__VA_ARGS__)
#else
#define FAIL_MSG(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define ERROR_MSG(fmt, ...) \
  fprintf(stderr, COLOR_RED_DARK "[ERROR]" COLOR_RESET " " fmt "\n", ##__VA_ARGS__)
#else
#define ERROR_MSG(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#define WARN_MSG(fmt, ...) \
  fprintf(stdout, COLOR_YELLOW "[WARN]" COLOR_RESET "  " fmt "\n", ##__VA_ARGS__)
#else
#define WARN_MSG(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define INFO_MSG(fmt, ...) \
  fprintf(stdout, COLOR_GREEN "[INFO]" COLOR_RESET "  " fmt "\n", ##__VA_ARGS__)
#else
#define INFO_MSG(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_HIGH
#define HIGH_MSG(fmt, ...) \
  fprintf(stdout, COLOR_CYAN "[HIGH]" COLOR_RESET "  " fmt "\n", ##__VA_ARGS__)
#else
#define HIGH_MSG(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define DEBUG_MSG(fmt, ...) \
  fprintf(stdout, COLOR_MAGENTA "[DEBUG]" COLOR_RESET " " fmt "\n", ##__VA_ARGS__)
#else
#define DEBUG_MSG(fmt, ...)
#endif

// ************************* ERRorCC *************************

#define ERR(a, b, c, d)                                                                   \
  ((uint32_t(uint8_t(a))) | (uint32_t(uint8_t(b)) << 8) | \(uint32_t(uint8_t(c)) << 16) | \
          (uint32_t(uint8_t(d)) << 24))

#define ERR_CHAR0(code) char((code) & 0xFF)
#define ERR_CHAR1(code) char(((code) >> 8) & 0xFF)
#define ERR_CHAR2(code) char(((code) >> 16) & 0xFF)
#define ERR_CHAR3(code) char(((code) >> 24) & 0xFF)

// ************************* Frame *************************
typedef struct {
  void          *start;     // Pointer to the frame data
  size_t         length;    // Total size of the frame
  struct timeval timestamp; // Capture timestamp
} Frame;
