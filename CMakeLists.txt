# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Set project name and language
project(LinuxCAM-RTP-Streamer LANGUAGES CXX)

# **-------------------- User-configurable options --------------------**
option(ENABLE_LTO "Enable link-time optimization (LTO) if supported" ON)

# Compile-time log level (0=NONE .. 6=DEBUG). Can be overridden with -DLOG_LEVEL=<n>
if(NOT DEFINED LOG_LEVEL)
  set(LOG_LEVEL 6 CACHE STRING "Compile-time log level (0..6)")
else()
  # ensure LOG_LEVEL is cached so subsequent cmake runs see it
  set(LOG_LEVEL ${LOG_LEVEL} CACHE STRING "Compile-time log level (0..6)" FORCE)
endif()

# **-------------------- Basic compiler / flags --------------------**
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default optimization flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

# Conservative default flags; add LTO if requested and supported
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(DEFAULT_OPT_FLAGS "-O3 -march=native -pipe")
  if(ENABLE_LTO)
    list(APPEND DEFAULT_OPT_FLAGS "-flto")
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(DEFAULT_OPT_FLAGS "-O3 -rtlib=compiler-rt -march=native -pipe")
  if(ENABLE_LTO)
    list(APPEND DEFAULT_OPT_FLAGS "-flto=jobserver")
  endif()
else()
  set(DEFAULT_OPT_FLAGS "-O3")
endif()

string(REPLACE ";" " " DEFAULT_OPT_FLAGS_STR "${DEFAULT_OPT_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEFAULT_OPT_FLAGS_STR}")

# Show configuration summary
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
message(STATUS "LOG_LEVEL (compile-time): ${LOG_LEVEL}")
message(STATUS "LTO enabled: ${ENABLE_LTO}")

# **-------------------- Sources and target --------------------**
set(SOURCES
  main.cc
  FrameForger/frameForger.cc
)

add_executable(exe ${SOURCES})

target_include_directories(exe PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/FrameForger
)

# Enforce C++ standard per-target
target_compile_features(exe PRIVATE cxx_std_20)

# Pass compile-time LOG_LEVEL to the target (as integer macro)
target_compile_definitions(exe PRIVATE LOG_LEVEL=${LOG_LEVEL})

# Add extra warning flags for GCC/Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang)")
  target_compile_options(exe PRIVATE -w)
endif()

# Platform-specific linker settings for LTO if needed
if(ENABLE_LTO)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # nothing more required here; -flto is already in CMAKE_CXX_FLAGS
    message(STATUS "LTO requested; ensure your toolchain supports it")
  endif()
endif()

# End of CMakeLists.txt
