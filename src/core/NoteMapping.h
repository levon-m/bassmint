#pragma once

#include "Tuning.h"
#include <cstddef>
#include <cstdint>

namespace bassmint {

/**
 * Maps frequencies to frets and MIDI notes
 *
 * Provides functions to:
 * - Convert detected frequency + string to nearest fret
 * - Convert (string, fret) to MIDI note number
 * - Calculate cents deviation from nearest note
 */
namespace NoteMapping {

/**
 * Result of frequency-to-fret mapping
 */
struct FretMatch {
    int fret = -1;           // Matched fret (0-24), -1 if invalid
    float expectedHz = 0.0f; // Expected frequency at matched fret
    float centsOff = 0.0f;   // Deviation in cents (+/- 50 max for same note)

    bool isValid() const { return fret >= 0; }
};

/**
 * Find the nearest fret on a specific string for a given frequency
 *
 * @param frequencyHz Detected frequency in Hz
 * @param stringIndex String index (0-3: E, A, D, G)
 * @return FretMatch with fret number and deviation info
 */
FretMatch findNearestFret(float frequencyHz, size_t stringIndex);

/**
 * Find the nearest fret on a specific string, restricting search range
 *
 * @param frequencyHz Detected frequency in Hz
 * @param stringIndex String index (0-3: E, A, D, G)
 * @param minFret Minimum fret to consider
 * @param maxFret Maximum fret to consider
 * @return FretMatch with fret number and deviation info
 */
FretMatch findNearestFretInRange(float frequencyHz, size_t stringIndex,
                                  int minFret, int maxFret);

/**
 * Get MIDI note number for a string and fret
 *
 * @param stringIndex String index (0-3: E, A, D, G)
 * @param fret Fret number (0-24)
 * @return MIDI note number (0-127)
 */
uint8_t getMidiNote(size_t stringIndex, int fret);

/**
 * Get MIDI channel for a string (1-based, per-string encoding)
 *
 * @param stringIndex String index (0-3)
 * @return MIDI channel (1-4)
 */
uint8_t getMidiChannel(size_t stringIndex);

/**
 * Calculate cents deviation between two frequencies
 *
 * @param measuredHz Measured frequency
 * @param referenceHz Reference frequency
 * @return Cents deviation (positive = sharp, negative = flat)
 */
float calculateCents(float measuredHz, float referenceHz);

/**
 * Check if a frequency is within valid bass range
 */
bool isValidBassFrequency(float frequencyHz);

} // namespace NoteMapping
} // namespace bassmint
