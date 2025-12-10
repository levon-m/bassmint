# External Dependencies

BassMINT uses minimal external dependencies to maintain portability and simplicity.

## Required Dependencies

### 1. Raspberry Pi Pico SDK (Required)

**Version**: 1.5.0 or later
**License**: BSD 3-Clause
**Source**: https://github.com/raspberrypi/pico-sdk

The Pico SDK provides hardware abstraction for the RP2040 microcontroller.

#### Pico SDK Components Used

BassMINT links against these Pico SDK libraries (see [CMakeLists.txt](../CMakeLists.txt)):

```cmake
pico_stdlib       # Core SDK functionality (stdio, time, etc.)
hardware_adc      # Analog-to-Digital Converter driver
hardware_pwm      # PWM for future LED brightness control
hardware_uart     # UART for MIDI DIN output @ 31250 baud
hardware_dma      # DMA support (reserved for future optimization)
hardware_irq      # Interrupt handling
hardware_timer    # Hardware timer for ADC sampling
```

#### Installation

**Option 1: Manual Installation**
```bash
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
export PICO_SDK_PATH=$(pwd)
```

**Option 2: Auto-Download (CMake)**

Set `PICO_SDK_FETCH_FROM_GIT=ON` and CMake will download automatically:

```bash
cmake -DPICO_SDK_FETCH_FROM_GIT=ON ..
```

---

## C++ Standard Library (Included)

BassMINT uses **C++17** standard library features:

- `<array>` - Fixed-size arrays (ring buffers, working buffers)
- `<functional>` - std::function for callbacks
- `<cmath>` - Math functions (log2, exp, sqrt)
- `<algorithm>` - min, max, clamp
- `<cstdint>` - Fixed-width integer types
- `<cstring>` - memset for buffer initialization

These are provided by the `arm-none-eabi-gcc` compiler, no external library needed.

---

## Build Tools (Required)

### ARM GNU Toolchain

**Version**: 10.3.1 or later
**License**: GPL with runtime exception
**Source**: https://developer.arm.com/downloads/-/gnu-rm

Cross-compiler for ARM Cortex-M0+ (RP2040).

#### Installation

**Windows**: Download ARM GNU installer from ARM website
**Linux (Ubuntu/Debian)**:
```bash
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi
```

**macOS (Homebrew)**:
```bash
brew install --cask gcc-arm-embedded
```

### CMake

**Version**: 3.13 or later
**License**: BSD 3-Clause
**Source**: https://cmake.org/

#### Installation

**Windows**: Download CMake installer
**Linux**: `sudo apt install cmake`
**macOS**: `brew install cmake`

---

## Optional Tools

### USB Serial Monitor

For debugging via USB serial output:

**Linux/macOS**: `screen` or `minicom`
```bash
screen /dev/ttyACM0 115200
```

**Windows**: PuTTY, Tera Term, or Arduino Serial Monitor

### MIDI Monitor

To verify MIDI output:

- **Linux**: `aseqdump`, `midisnoop`
- **macOS**: MIDI Monitor app
- **Windows**: MIDI-OX, MIDI Monitor
- **Cross-platform DAW**: Reaper, Ableton Live, etc.

---

## No Third-Party DSP Libraries

BassMINT **intentionally avoids** external DSP libraries for:

1. **Portability**: No complex build dependencies
2. **Simplicity**: All DSP code is readable and modifiable
3. **Size**: Minimal binary footprint (<100KB)
4. **Customization**: Easy to swap algorithms (e.g., replace YIN with ML)

### YIN Algorithm

The YIN pitch detection is **implemented from scratch** in [src/dsp/PitchDetectorYin.cpp](../src/dsp/PitchDetectorYin.cpp) based on:

> de Cheveigné, A., & Kawahara, H. (2002). "YIN, a fundamental frequency estimator for speech and music." *The Journal of the Acoustical Society of America*, 111(4), 1917-1930.

No external pitch detection library required.

---

## Memory Footprint

All dependencies are statically linked into the final `.uf2` binary:

| Component | Estimated Size |
|-----------|----------------|
| Pico SDK runtime | ~40 KB |
| BassMINT application code | ~30 KB |
| DSP buffers (static) | ~32 KB |
| Stack + heap | ~16 KB |
| **Total** | **~120 KB** |

RP2040 has **264 KB SRAM**, leaving plenty of headroom.

---

## Future Dependencies (Planned)

### TensorFlow Lite Micro (Optional)

For machine learning-based pitch detection:

**Version**: TBD
**License**: Apache 2.0
**Source**: https://github.com/tensorflow/tflite-micro

**Status**: Not yet implemented, reserved for future optimization

**Estimated footprint**: +50-100 KB (model + runtime)

---

## Dependency Installation Summary

Quick start for a fresh development environment:

### Linux (Ubuntu 20.04+)
```bash
# Install toolchain
sudo apt update
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi cmake build-essential

# Install Pico SDK
git clone https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
cd ~/pico-sdk
git submodule update --init
echo 'export PICO_SDK_PATH=~/pico-sdk' >> ~/.bashrc
source ~/.bashrc

# Build BassMINT
cd ~/BassMINT
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### macOS
```bash
brew install cmake
brew install --cask gcc-arm-embedded

git clone https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
cd ~/pico-sdk
git submodule update --init
export PICO_SDK_PATH=~/pico-sdk

cd ~/BassMINT
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu)
```

### Windows (MSYS2/MinGW)
```bash
# Install MSYS2 from https://www.msys2.org/
# In MSYS2 terminal:
pacman -S mingw-w64-x86_64-arm-none-eabi-gcc
pacman -S mingw-w64-x86_64-cmake

# Download Pico SDK manually or use auto-fetch in CMake
cd BassMINT
mkdir build && cd build
cmake -G "MinGW Makefiles" -DPICO_SDK_FETCH_FROM_GIT=ON ..
mingw32-make
```

---

## Licenses

All dependencies use permissive open-source licenses:

| Component | License | Commercial Use |
|-----------|---------|----------------|
| Pico SDK | BSD 3-Clause | ✅ Yes |
| ARM GCC Runtime | GPL with exception | ✅ Yes (runtime exception) |
| BassMINT Code | MIT (proposed) | ✅ Yes |

---

## Verification

To verify all dependencies are correctly installed:

```bash
# Check ARM compiler
arm-none-eabi-gcc --version
# Expected: gcc version 10.3.1 or later

# Check CMake
cmake --version
# Expected: cmake version 3.13 or later

# Check Pico SDK
echo $PICO_SDK_PATH
# Expected: /path/to/pico-sdk

# Verify SDK integrity
ls $PICO_SDK_PATH/pico_sdk_init.cmake
# Expected: file exists
```

---

## Troubleshooting

### "PICO_SDK_PATH not found"

Set the environment variable:
```bash
export PICO_SDK_PATH=/path/to/pico-sdk
```

Or use CMake auto-fetch:
```bash
cmake -DPICO_SDK_FETCH_FROM_GIT=ON ..
```

### "arm-none-eabi-gcc not found"

Add to PATH or install toolchain:
```bash
# Linux
sudo apt install gcc-arm-none-eabi

# macOS
brew install --cask gcc-arm-embedded
```

### Link Errors

Ensure all Pico SDK submodules are initialized:
```bash
cd $PICO_SDK_PATH
git submodule update --init
```

---

## Contact

For dependency-related issues, consult:
- Pico SDK: https://github.com/raspberrypi/pico-sdk/issues
- BassMINT: (Open issue on project repo)
