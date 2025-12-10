#pragma once

#include <cstdint>

namespace bassmint {

/**
 * MIDI CC numbers used by BassMINT
 */
namespace MidiCC {
    static constexpr uint8_t Envelope   = 20;  // Per-string envelope (0-127)
    static constexpr uint8_t State      = 21;  // String state (0=Idle, 64=Active)
    static constexpr uint8_t Confidence = 22;  // Pitch confidence (0-127)
}

/**
 * MIDI state values for string state CC
 */
namespace MidiState {
    static constexpr uint8_t Idle   = 0;
    static constexpr uint8_t Active = 64;
}

/**
 * Hardware abstraction for MIDI 1.0 output via USB
 *
 * Uses Teensy 4.0's built-in USB MIDI support.
 * Channel-per-string encoding:
 * - Channel 1: String 0 (E)
 * - Channel 2: String 1 (A)
 * - Channel 3: String 2 (D)
 * - Channel 4: String 3 (G)
 */
class MidiOutput {
public:
    /**
     * Initialize MIDI output
     */
    void init();

    /**
     * Send Note On message
     * @param channel MIDI channel (1-16)
     * @param note MIDI note number (0-127)
     * @param velocity Note velocity (0-127)
     */
    void sendNoteOn(uint8_t channel, uint8_t note, uint8_t velocity);

    /**
     * Send Note Off message
     * @param channel MIDI channel (1-16)
     * @param note MIDI note number (0-127)
     * @param velocity Release velocity (0-127, typically 0)
     */
    void sendNoteOff(uint8_t channel, uint8_t note, uint8_t velocity = 0);

    /**
     * Send Control Change message
     * @param channel MIDI channel (1-16)
     * @param cc Controller number (0-127)
     * @param value Controller value (0-127)
     */
    void sendControlChange(uint8_t channel, uint8_t cc, uint8_t value);

    /**
     * Send Pitch Bend message
     * @param channel MIDI channel (1-16)
     * @param value Pitch bend value (0-16383, 8192 = center)
     */
    void sendPitchBend(uint8_t channel, uint16_t value);

    /**
     * Flush any pending MIDI messages
     */
    void flush();

    // Convenience methods for string-based MIDI

    /**
     * Send note on for a specific string
     * @param stringIndex String index (0-3)
     * @param note MIDI note number
     * @param velocity Note velocity
     */
    void sendStringNoteOn(uint8_t stringIndex, uint8_t note, uint8_t velocity);

    /**
     * Send note off for a specific string
     */
    void sendStringNoteOff(uint8_t stringIndex, uint8_t note, uint8_t velocity = 0);

    /**
     * Send envelope CC for a specific string
     * @param stringIndex String index (0-3)
     * @param envelope Envelope value (0.0-1.0, will be scaled to 0-127)
     */
    void sendStringEnvelope(uint8_t stringIndex, float envelope);

    /**
     * Send state CC for a specific string
     * @param stringIndex String index (0-3)
     * @param active True if string is active
     */
    void sendStringState(uint8_t stringIndex, bool active);

    /**
     * Send confidence CC for a specific string
     * @param stringIndex String index (0-3)
     * @param confidence Confidence value (0.0-1.0, will be scaled to 0-127)
     */
    void sendStringConfidence(uint8_t stringIndex, float confidence);

private:
    /**
     * Convert string index to MIDI channel (1-based)
     */
    static uint8_t stringToChannel(uint8_t stringIndex);

    /**
     * Clamp value to MIDI range (0-127)
     */
    static uint8_t clampMidi(int value);

    /**
     * Scale float (0-1) to MIDI range (0-127)
     */
    static uint8_t floatToMidi(float value);
};

} // namespace bassmint
