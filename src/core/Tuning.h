#pragma once

#include <cstddef>
#include <cstdint>

namespace bassmint {

/**
 * Standard bass guitar tuning constants
 *
 * Standard 4-string bass tuning (low to high):
 * - String 0: E1 (41.20 Hz) - MIDI note 28
 * - String 1: A1 (55.00 Hz) - MIDI note 33
 * - String 2: D2 (73.42 Hz) - MIDI note 38
 * - String 3: G2 (98.00 Hz) - MIDI note 43
 */
namespace Tuning {

// Number of strings on a standard bass
static constexpr size_t NumStrings = 4;

// Maximum fret number (typical bass has 20-24 frets)
static constexpr int MaxFret = 24;

// Open string frequencies in Hz (standard tuning)
static constexpr float OpenStringHz[NumStrings] = {
    41.203f,   // E1 - String 0
    55.000f,   // A1 - String 1
    73.416f,   // D2 - String 2
    97.999f    // G2 - String 3
};

// Open string MIDI note numbers
static constexpr uint8_t OpenStringMidiNote[NumStrings] = {
    28,  // E1 - String 0
    33,  // A1 - String 1
    38,  // D2 - String 2
    43   // G2 - String 3
};

// String names for debugging/display
static constexpr const char* StringNames[NumStrings] = {
    "E", "A", "D", "G"
};

/**
 * Get the open string frequency for a given string
 */
inline float getOpenStringHz(size_t stringIndex) {
    if (stringIndex >= NumStrings) return 0.0f;
    return OpenStringHz[stringIndex];
}

/**
 * Get the open string MIDI note for a given string
 */
inline uint8_t getOpenStringMidiNote(size_t stringIndex) {
    if (stringIndex >= NumStrings) return 0;
    return OpenStringMidiNote[stringIndex];
}

/**
 * Get the string name
 */
inline const char* getStringName(size_t stringIndex) {
    if (stringIndex >= NumStrings) return "?";
    return StringNames[stringIndex];
}

/**
 * Equal temperament semitone ratio
 * Each semitone up multiplies frequency by 2^(1/12)
 */
static constexpr float SemitoneRatio = 1.059463094359f;

/**
 * Calculate frequency at a given fret on a given string
 */
float getFretFrequency(size_t stringIndex, int fret);

/**
 * Get MIDI note number for a string and fret combination
 */
uint8_t getMidiNote(size_t stringIndex, int fret);

} // namespace Tuning
} // namespace bassmint
