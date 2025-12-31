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
set(USB_MODE "MIDI_SERIAL" CACHE STRING "USB mode")

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

# Path to Teensy cores - prefer local libs/ folder, fallback to system install
set(LOCAL_TEENSY_CORES "${CMAKE_CURRENT_SOURCE_DIR}/libs/TeensyCores")

if(EXISTS "${LOCAL_TEENSY_CORES}/teensy4")
    set(TEENSY_PATH "${LOCAL_TEENSY_CORES}")
    set(CORES_SUBDIR "teensy4")
    message(STATUS "Using local Teensy cores: ${TEENSY_PATH}")
elseif(DEFINED TEENSY_PATH AND EXISTS "${TEENSY_PATH}/cores/teensy4")
    set(CORES_SUBDIR "cores/teensy4")
    message(STATUS "Using system Teensy cores: ${TEENSY_PATH}")
else()
    # Auto-detect system installation
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
            set(CORES_SUBDIR "cores/teensy4")
            break()
        endif()
    endforeach()

    if(TEENSY_PATH)
        message(STATUS "Using system Teensy cores: ${TEENSY_PATH}")
    else()
        message(FATAL_ERROR
            "Teensy cores not found!\n"
            "Either place TeensyCores in libs/ or set TEENSY_PATH:\n"
            "  cmake -DTEENSY_PATH=\"C:/Program Files (x86)/Arduino/hardware/teensy/avr\" -B build -G Ninja"
        )
    endif()
endif()

project(BassMINT VERSION 1.0.0 LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

# Teensy paths
set(CORES_PATH "${TEENSY_PATH}/${CORES_SUBDIR}")

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
    ARDUINO_TEENSY40
    TEENSYDUINO=159
    ARDUINO=10819
    F_CPU=${CPU_SPEED}
    USB_MIDI_SERIAL
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
    src/dsp/OctaveFretEstimator.cpp
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

# Generate .hex for Teensy Loader
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMENT "Generating ${PROJECT_NAME}.hex"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
)
