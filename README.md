# BassMINT

**Bass Guitar MIDI Interface using Teensy 4.0**

BassMINT is a real-time pitch-to-MIDI system optimized for bass guitar. It uses optical string sensors to accurately detect which string is being played and what note is being fretted.

## Why BassMINT?

Traditional pitch-to-MIDI systems struggle with bass guitar due to:
- Long wavelengths requiring larger analysis windows
- Sympathetic string vibration causing false triggers
- Difficulty distinguishing which string produced a note (same pitch can exist on multiple strings)

BassMINT solves these problems with **OPT101 optical sensors** that serve dual purposes:
1. **String detection** - Per-string envelope tracking determines which string is active
2. **Pitch detection** - The string vibration modulates the light beam, encoding pitch information in the same signal

## Hardware Requirements

- **Teensy 4.0** (IMXRT1062 ARM Cortex-M7 @ 600MHz)
- **4x OPT101 sensors** - One per string for activity and pitch detection
- **4x IR LEDs** - Paired with OPT101s for break-beam sensing
- **MIDI FeatherWing** - Hardware MIDI DIN output

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        STAGE 1                               │
│                   String Activity                            │
│                                                              │
│  OPT101 Sensors ──► Envelope Followers ──► Active String    │
│  (A5-A8)            (per string)            Selection       │
│                                                              │
│  Output: Which string is currently being played              │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                        STAGE 2                               │
│                   Pitch Detection                            │
│                                                              │
│  Active String's ──► DC Removal ──► High-Pass ──► YIN       │
│  OPT101 Signal                      Filter        Algorithm │
│                                                              │
│  Output: Frequency (Hz) + Confidence                         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                        STAGE 3                               │
│                   String+Fret Resolver                       │
│                                                              │
│  Active String + Frequency ──► Fret Lookup ──► MIDI Note    │
│                                (constrained to string)       │
│                                                              │
│  Output: StringFretDecision {string, fret, confidence}       │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                      MIDI Output                             │
│                                                              │
│  • Note On/Off (channel = string + 1)                       │
│  • CC 20: Envelope                                           │
│  • CC 21: State (Idle/Active)                               │
│  • CC 22: Pitch Confidence                                   │
└─────────────────────────────────────────────────────────────┘
```

## MIDI Encoding

### Channel-Per-String

Each string outputs on its own MIDI channel:

| String | MIDI Channel | Open Note | MIDI Note # |
|--------|--------------|-----------|-------------|
| E (0)  | 1 | E1 | 28 |
| A (1)  | 2 | A1 | 33 |
| D (2)  | 3 | D2 | 38 |
| G (3)  | 4 | G2 | 43 |

**Note Number** = Open String MIDI Note + Fret Number

### Control Change Messages

| CC | Name | Range | Description |
|----|------|-------|-------------|
| 20 | Envelope | 0-127 | String envelope level |
| 21 | State | 0/64 | 0=Idle, 64=Active |
| 22 | Confidence | 0-127 | Pitch detection confidence |

## Building

### Prerequisites

- CMake 3.19+
- Ninja build system
- ARM GNU Toolchain (`arm-none-eabi-gcc`)
- Teensy Loader for flashing

### Build Commands

```bash
# Option 1: Using the toolchain file
cmake -DCMAKE_TOOLCHAIN_FILE=cmake/teensy40.cmake -B build -G Ninja

# Option 2: Auto-detect (CMakeLists.txt has inline toolchain setup)
cmake -B build -G Ninja

# Build
cmake --build build

# Output: build/BassMINT.hex
```

If Teensy cores are not auto-detected, specify the path:

```bash
cmake -DTEENSY_PATH="C:/Program Files (x86)/Arduino/hardware/teensy/avr" -B build -G Ninja
```

### Flashing

Use Teensy Loader to flash `build/BassMINT.hex` to your Teensy 4.0.

## Configuration

Key parameters in `src/hal/BoardConfig.h`:

```cpp
static constexpr float SampleRate = 44100.0f;       // Audio sample rate
static constexpr size_t PitchFrameSize = 2048;      // ~46ms analysis window
```

DSP parameters in respective headers:

- `PitchDetector.h`: YIN threshold (0.15), frequency range (30-300 Hz)
- `StringActivity.h`: Attack/release thresholds (0.1/0.05)
- `EnvelopeFollower.h`: Attack/release times (5ms/100ms)

### Debug Output (Teleplot)

Enable real-time visualization in `BoardConfig.h`:

```cpp
static constexpr bool DebugEnabled = true;
static constexpr uint32_t DebugOutputHz = 60;  // Update rate
```

Plots sent over Serial (Teleplot format):

- `env_E`, `env_A`, `env_D`, `env_G` - String envelope levels
- `pitch_hz` - Detected frequency
- `confidence` - Pitch detection confidence
- `string` - Active string (-1 if none, 0-3 for E/A/D/G)
- `fret` - Detected fret (-1 if none)
- `midi_note` - Current MIDI note being played

Use the [Teleplot VSCode extension](https://marketplace.visualstudio.com/items?itemName=niceprogrammer.teleplot) to visualize.

## Theory of Operation

### String Activity Detection

Each string has an OPT101 photodiode + IR LED pair mounted as a break-beam sensor. When the string vibrates, it modulates the light reaching the photodiode, creating an AC signal proportional to string activity.

The envelope follower tracks the amplitude of this signal:
- Fast attack (5ms) captures pluck transients
- Slow release (100ms) maintains sustain

The **active string** is selected as:
1. The string with highest envelope above threshold
2. If multiple strings are close, the most recently activated wins

This prevents false triggers from sympathetic vibration.

### Pitch Detection (YIN Algorithm)

The same OPT101 signal used for string detection also contains the string's vibration waveform. Once the active string is identified, its signal is fed to the YIN pitch detector.

YIN is a time-domain pitch detection algorithm ideal for monophonic signals:

1. **Difference Function** - Autocorrelation-based comparison
2. **Cumulative Mean Normalized Difference** - Normalization for threshold
3. **Absolute Threshold** - Find first dip below threshold
4. **Parabolic Interpolation** - Sub-sample precision

Configuration for bass:
- Frequency range: 30-300 Hz
- Frame size: 2048 samples (~46ms at 44.1kHz)
- High-pass filter at 20Hz removes mechanical noise

### Fret Resolution

Once we know the active string and detected frequency, we search only that string's fret range:

```
frequency → nearest fret on active string → MIDI note
```

This constraint dramatically improves accuracy compared to searching all strings.

## License

MIT License - See LICENSE file for details.
