# BassMINT Architecture

This document describes the detailed architecture of the BassMINT bass guitar MIDI interface.

## System Overview

BassMINT operates as a real-time signal processing pipeline on the Teensy 4.0 microcontroller. The system processes audio at 44.1kHz, with pitch estimates produced every ~23ms (50% overlap on 2048-sample frames).

## Hardware Layer

### Sensors

**Piezo Pickup (A0)**
- Bridge-mounted contact microphone
- AC-coupled signal centered at VRef/2 (1.65V)
- Captures string vibration for pitch detection
- Wide bandwidth captures fundamental and harmonics

**OPT101 Break-Beam Sensors (A1-A4)**
- Integrated photodiode + transimpedance amplifier
- Paired with IR LEDs for per-string detection
- DC output varies with string position
- String vibration creates AC component proportional to activity

### ADC Configuration

- 12-bit resolution (0-4095)
- Hardware averaging (4 samples) for noise reduction
- ~100kHz effective sample rate per channel
- Multiplexed reading: piezo first, then strings in sequence

## Software Architecture

### Module Hierarchy

```
main.cpp
    └── App
        ├── AdcDriver (HAL)
        ├── LedDriver (HAL)
        ├── MidiOutput (HAL)
        ├── StringActivity (DSP)
        │   └── EnvelopeFollower (DSP) x4
        ├── PitchDetector (DSP)
        └── StringFretResolver (App)
            ├── NoteTables (Core)
            ├── NoteMapping (Core)
            └── Tuning (Core)
```

### Data Flow

```
Sample n arrives:
    │
    ├──► OPT101[0..3] ──► EnvelopeFollower ──► StringActivityState
    │                                              │
    │                                              ▼
    │                                        getActiveString()
    │                                              │
    └──► Piezo ──► HighPass ──► Window ──► YIN ──┤
                                             │    │
                                             ▼    ▼
                                        PitchResult
                                             │
                                             ▼
                               StringFretResolver.resolve()
                                             │
                                             ▼
                                    StringFretDecision
                                             │
                               ┌─────────────┴─────────────┐
                               ▼                           ▼
                          Note On/Off                 CC Messages
                          (if changed)            (envelope, state,
                                                   confidence)
```

## DSP Module Details

### EnvelopeFollower

Simple asymmetric low-pass filter for amplitude tracking:

```cpp
if (|x| > envelope) {
    envelope = attack * envelope + (1 - attack) * |x|;
} else {
    envelope = release * envelope + (1 - release) * |x|;
}
```

**Parameters:**
- Attack: 5ms (coefficient ~0.9973 at 44.1kHz)
- Release: 100ms (coefficient ~0.99977)

### StringActivity

Maintains state for all 4 strings:

**State Machine (per string):**
```
        ┌──────────────────────────────────────┐
        │                                      │
        ▼                                      │
    ┌───────┐    envelope > attackThresh   ┌───────┐
    │ IDLE  │ ─────────────────────────►   │ACTIVE │
    └───────┘                               └───────┘
        ▲                                      │
        │     envelope < releaseThresh         │
        └──────────────────────────────────────┘
```

**Active String Selection:**
1. Find all strings in ACTIVE state
2. Select string with highest envelope
3. Tie-breaker: most recent state transition wins

### PitchDetector

YIN algorithm implementation optimized for bass:

**Frame Processing:**
1. High-pass filter (20Hz cutoff) - removes DC, subsonic rumble
2. Hann window - reduces spectral leakage
3. Difference function - autocorrelation variant
4. CMNDF - cumulative mean normalized difference
5. Threshold search - find first dip below threshold
6. Parabolic interpolation - sub-sample precision

**Bass Optimizations:**
- Frequency range: 30-300 Hz
- minLag = sampleRate / 300 = 147 samples
- maxLag = sampleRate / 30 = 1470 samples
- Frame size: 2048 samples (~46ms) - captures ~1.5 cycles at 30Hz

## Core Module Details

### Tuning

Standard 4-string bass tuning (E-A-D-G):

| String | Note | Frequency | MIDI Note |
|--------|------|-----------|-----------|
| 0 | E1 | 41.20 Hz | 28 |
| 1 | A1 | 55.00 Hz | 33 |
| 2 | D2 | 73.42 Hz | 38 |
| 3 | G2 | 98.00 Hz | 43 |

### NoteTables

Pre-computed frequency table for all frets (0-24) on all strings:

```
f(string, fret) = openFrequency[string] * 2^(fret/12)
```

Generated at startup to avoid runtime calculation.

### NoteMapping

Maps detected frequency to nearest fret:

```cpp
FretMatch findNearestFret(float hz, size_t string) {
    for (fret = 0; fret <= 24; fret++) {
        cents = 1200 * log2(hz / table[string][fret]);
        if (|cents| < bestCents) {
            bestFret = fret;
            bestCents = |cents|;
        }
    }
    return {bestFret, cents};
}
```

Rejects matches > 100 cents (1 semitone) from any valid fret.

### StringFretResolver

Combines string activity and pitch detection:

```cpp
resolve() {
    if (!pitch.isValid || pitch.confidence < threshold)
        return nullopt;

    activeString = getActiveString();
    if (!activeString)
        return nullopt;

    fret = findNearestFret(pitch.hz, activeString);
    if (!fret.isValid)
        return nullopt;

    return StringFretDecision{activeString, fret, ...};
}
```

## HAL Module Details

### AdcDriver

Direct register access for IMXRT1062 ADC:

- Single-ended mode
- 12-bit resolution
- Hardware averaging (configurable 1-32)
- ~10μs per conversion

**Pin Mapping:**
```cpp
uint8_t pinToChannel(uint8_t pin) {
    // Teensy 4.0 analog pin to ADC channel
    14 → 7, 15 → 8, 16 → 12, 17 → 11, 18 → 6
}
```

### MidiOutput

USB MIDI using Teensy's built-in USB stack:

**Packet Format (USB-MIDI):**
```
[Cable/CIN][Status][Data1][Data2]
    │          │       │      └── Data byte 2
    │          │       └── Data byte 1
    │          └── MIDI status byte
    └── Cable number (4 bits) + Code Index Number (4 bits)
```

**CIN Values:**
- 0x08: Note Off
- 0x09: Note On
- 0x0B: Control Change
- 0x0E: Pitch Bend

## App Module

### Main Loop Timing

Uses ARM cycle counter for precise sample rate:

```cpp
while (true) {
    uint32_t now = ARM_DWT_CYCCNT;
    if ((now - lastCycle) >= cyclesPerSample) {
        lastCycle = now;
        app.update();
    }
}
```

At 600MHz CPU, 44.1kHz sample rate:
- cyclesPerSample = 600,000,000 / 44,100 = 13,605 cycles
- ~22.7μs between samples

### Note Management

State machine for MIDI note output:

```
                      ┌─────────────────────────┐
                      │                         │
        resolve()     ▼     resolve()          │
        returns    ┌──────┐  returns           │
        nullopt    │ OFF  │  decision          │
       ┌───────────│      │◄───────────────────┤
       │           └──────┘                    │
       │              │                        │
       │              │ resolve() returns      │
       │              │ new decision           │
       │              ▼                        │
       │           ┌──────┐                    │
       │           │  ON  │────────────────────┘
       └──────────►│      │  same note continues
                   └──────┘
                      │
                      │ resolve() returns
                      │ different note
                      ▼
              [Note Off old, Note On new]
```

## Performance Considerations

### CPU Budget (per sample @ 44.1kHz)

| Component | Estimated Cycles |
|-----------|------------------|
| ADC reads (5 channels) | ~1000 |
| Envelope followers (4x) | ~200 |
| High-pass filter | ~50 |
| Buffer management | ~100 |
| **Total per sample** | **~1350** |

| Component | Per-frame (~23ms) |
|-----------|-------------------|
| Hann window | ~2000 |
| Difference function | ~4M |
| CMNDF | ~3000 |
| Tau search | ~1500 |
| **Total per frame** | **~4M** |

With 600MHz CPU and 1000 samples per frame:
- Per-sample budget: 13,605 cycles × 1000 = 13.6M cycles
- Available for pitch detection: ~9M cycles
- Actual usage: ~4M cycles (comfortable margin)

### Memory Usage

| Component | Size |
|-----------|------|
| Input buffer | 4096 × 4 = 16KB |
| Difference buffer | 2048 × 4 = 8KB |
| CMNDF buffer | 2048 × 4 = 8KB |
| Window coefficients | 4096 × 4 = 16KB |
| Note tables | 4 × 25 × 4 = 400B |
| Stack | ~2KB |
| **Total** | **~50KB** |

Teensy 4.0 has 512KB DTCM - plenty of headroom.
