cmake_minimum_required(VERSION 3.16)

project(BassMINT_Tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Test executable for OctaveFretEstimator
add_executable(test_octave
    TestOctaveFretEstimator.cpp
    ../src/dsp/OctaveFretEstimator.cpp
    ../src/core/Tuning.cpp
)

target_include_directories(test_octave PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

# Compiler warnings
target_compile_options(test_octave PRIVATE
    -Wall -Wextra -O2
)

# Enable address sanitizer for debug builds (if available)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_octave PRIVATE -fsanitize=address -fno-omit-frame-pointer)
        target_link_options(test_octave PRIVATE -fsanitize=address)
    endif()
endif()
