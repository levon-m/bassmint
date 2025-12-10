#pragma once

#include "Tuning.h"
#include <array>

namespace bassmint {

/**
 * Pre-computed frequency tables for all frets on all strings
 *
 * Using equal temperament tuning (A4 = 440 Hz reference)
 * Tables are computed at compile time where possible
 */
class NoteTables {
public:
    static constexpr size_t NumStrings = Tuning::NumStrings;
    static constexpr int MaxFret = Tuning::MaxFret;
    static constexpr int NumFrets = MaxFret + 1; // 0-24 inclusive

    /**
     * Initialize the frequency tables
     * Call once at startup
     */
    static void init();

    /**
     * Get the frequency for a specific string and fret
     */
    static float getFrequency(size_t stringIndex, int fret);

    /**
     * Get the MIDI note for a specific string and fret
     */
    static uint8_t getMidiNote(size_t stringIndex, int fret);

    /**
     * Get the minimum frequency for a string (open string)
     */
    static float getMinFrequency(size_t stringIndex);

    /**
     * Get the maximum frequency for a string (highest fret)
     */
    static float getMaxFrequency(size_t stringIndex);

    /**
     * Check if tables are initialized
     */
    static bool isInitialized();

private:
    // Frequency table: [string][fret] -> Hz
    static std::array<std::array<float, NumFrets>, NumStrings> frequencyTable_;

    // MIDI note table: [string][fret] -> MIDI note
    static std::array<std::array<uint8_t, NumFrets>, NumStrings> midiNoteTable_;

    static bool initialized_;
};

} // namespace bassmint
