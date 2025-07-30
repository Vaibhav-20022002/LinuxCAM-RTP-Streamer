# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Set project name and language
project(LinuxCAM-RTP-Streamer CXX)

# Force using Clang compiler
set(CMAKE_CXX_COMPILER clang++)

# Set C++ standard and other flags
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Add source files
add_executable(exe
    main.cc
    FrameForger/frameForger.cc
)

# Add include directories
target_include_directories(exe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/FrameForger
)
