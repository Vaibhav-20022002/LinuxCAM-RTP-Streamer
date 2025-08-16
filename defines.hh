#include <stdio.h>
#include <sys/time.h>

// ************************* LOGS *************************

#define FAIL_MSG(fmt, ...) fprintf(stdout, "[FAIL]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_MSG(fmt, ...) fprintf(stdout, "[ERROR] " fmt "\n", ##__VA_ARGS__)
#define WARN_MSG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define INFO_MSG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define HIGH_MSG(fmt, ...) fprintf(stdout, "[HIGH]  " fmt "\n", ##__VA_ARGS__)
#define DEBUG_MSG(fmt, ...) fprintf(stdout, "[DEBUG] " fmt "\n", ##__VA_ARGS__)

// ************************* ERRorCC *************************

#define ERR(a, b, c, d)                                                                  \
  ((uint32_t(uint8_t(a))) | (uint32_t(uint8_t(b)) << 8) | (uint32_t(uint8_t(c)) << 16) | \
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
