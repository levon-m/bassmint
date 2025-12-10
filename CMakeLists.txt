cmake_minimum_required(VERSION 3.19)

# =============================================================================
# BassMINT - Teensy 4.0 Build Configuration
# =============================================================================
# Build options:
#   cmake -DCMAKE_TOOLCHAIN_FILE=cmake/teensy40.cmake -B build -G Ninja
#   cmake -B build -G Ninja  (auto-detects toolchain)
# =============================================================================

set(TEENSY_VERSION "40" CACHE STRING "Teensy version")
set(CPU_SPEED "600000000" CACHE STRING "CPU speed in Hz")
set(USB_MODE "MIDI" CACHE STRING "USB mode")

# Toolchain setup (if not using external toolchain file)
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR arm)

    find_program(ARM_CC arm-none-eabi-gcc REQUIRED)
    find_program(ARM_CXX arm-none-eabi-g++ REQUIRED)
    find_program(ARM_OBJCOPY arm-none-eabi-objcopy REQUIRED)
    find_program(ARM_SIZE arm-none-eabi-size REQUIRED)

    set(CMAKE_C_COMPILER ${ARM_CC})
    set(CMAKE_CXX_COMPILER ${ARM_CXX})
    set(CMAKE_ASM_COMPILER ${ARM_CC})
    set(CMAKE_OBJCOPY ${ARM_OBJCOPY})
    set(CMAKE_SIZE ${ARM_SIZE})
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
endif()

# Path to Teensy cores - set via -DTEENSY_PATH=... or auto-detect
# Common locations:
#   Windows: C:/Program Files (x86)/Arduino/hardware/teensy/avr
#   macOS: /Applications/Arduino.app/Contents/Java/hardware/teensy/avr
#   Linux: ~/.arduino15/packages/teensy/hardware/avr/*
if(NOT DEFINED TEENSY_PATH)
    if(WIN32)
        set(_SEARCH_PATHS
            "C:/Program Files (x86)/Arduino/hardware/teensy/avr"
            "C:/Program Files/Arduino/hardware/teensy/avr"
        )
    elseif(APPLE)
        set(_SEARCH_PATHS
            "/Applications/Arduino.app/Contents/Java/hardware/teensy/avr"
            "/Applications/Teensyduino.app/Contents/Java/hardware/teensy/avr"
        )
    else()
        set(_SEARCH_PATHS
            "$ENV{HOME}/.arduino15/packages/teensy/hardware/avr"
        )
    endif()

    foreach(_PATH ${_SEARCH_PATHS})
        if(EXISTS "${_PATH}/cores/teensy4")
            set(TEENSY_PATH "${_PATH}")
            break()
        endif()
    endforeach()
endif()

if(NOT TEENSY_PATH OR NOT EXISTS "${TEENSY_PATH}/cores/teensy4")
    message(FATAL_ERROR
        "Teensy cores not found!\n"
        "Install Teensyduino and set TEENSY_PATH:\n"
        "  cmake -DTEENSY_PATH=\"C:/Program Files (x86)/Arduino/hardware/teensy/avr\" -B build -G Ninja"
    )
endif()

message(STATUS "Teensy cores: ${TEENSY_PATH}")

project(BassMINT VERSION 1.0.0 LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

# Teensy paths
set(CORES_PATH "${TEENSY_PATH}/cores/teensy4")

# Cortex-M7 compiler flags
set(CPU_FLAGS "-mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb")
set(CMAKE_C_FLAGS "${CPU_FLAGS} -O2 -g -Wall -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-exceptions -fno-rtti -felide-constructors")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp")

# Use Teensy's linker script
set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS} -Wl,--gc-sections,--relax -T${CORES_PATH}/imxrt1062.ld")

# Teensy 4.0 definitions
add_compile_definitions(
    __IMXRT1062__
    TEENSYDUINO=159
    ARDUINO=10819
    F_CPU=${CPU_SPEED}
    USB_MIDI
    LAYOUT_US_ENGLISH
)

# Teensy core sources
file(GLOB TEENSY_C_SOURCES "${CORES_PATH}/*.c")
file(GLOB TEENSY_CXX_SOURCES "${CORES_PATH}/*.cpp")
file(GLOB TEENSY_ASM_SOURCES "${CORES_PATH}/*.S")

# BassMINT sources
set(BASSMINT_SOURCES
    src/main.cpp
    # App
    src/app/App.cpp
    src/app/StringFretResolver.cpp
    # DSP
    src/dsp/PitchDetector.cpp
    src/dsp/EnvelopeFollower.cpp
    src/dsp/StringActivity.cpp
    # Core
    src/core/Tuning.cpp
    src/core/NoteTables.cpp
    src/core/NoteMapping.cpp
    # HAL
    src/hal/AdcDriver.cpp
    src/hal/LedDriver.cpp
    src/hal/MidiOutput.cpp
)

# Build executable
add_executable(${PROJECT_NAME}
    ${BASSMINT_SOURCES}
    ${TEENSY_C_SOURCES}
    ${TEENSY_CXX_SOURCES}
    ${TEENSY_ASM_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CORES_PATH}
)

<<<<<<< HEAD
# Generate .hex for Teensy Loader
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMENT "Generating ${PROJECT_NAME}.hex"
=======
# Enable USB output, disable UART output for stdio
# (we use UART for MIDI, not debug)
pico_enable_stdio_usb(bassmint 1)
pico_enable_stdio_uart(bassmint 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(bassmint)

# Debug output options
option(BASSMINT_DEBUG_STATS "Enable debug statistics output" OFF)

if(BASSMINT_DEBUG_STATS)
    target_compile_definitions(bassmint PRIVATE BASSMINT_DEBUG_STATS=1)
endif()

# Compiler optimizations for embedded
target_compile_options(bassmint PRIVATE
    -Wall
    -Wextra
    -Werror=return-type
    -O2
    -ffunction-sections
    -fdata-sections
>>>>>>> 491b711ec2cfccc73c60c377d0b2d86c73cefe06
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
)
