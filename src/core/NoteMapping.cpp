#include "NoteMapping.h"
#include "NoteTables.h"
#include <cmath>
#include <limits>

namespace bassmint {
namespace NoteMapping {

// Minimum/maximum valid bass frequencies
static constexpr float MinBassHz = 30.0f;   // Below lowest E1
static constexpr float MaxBassHz = 400.0f;  // Above highest fret on G string

FretMatch findNearestFret(float frequencyHz, size_t stringIndex) {
    return findNearestFretInRange(frequencyHz, stringIndex, 0, Tuning::MaxFret);
}

FretMatch findNearestFretInRange(float frequencyHz, size_t stringIndex,
                                  int minFret, int maxFret) {
    FretMatch result;

    // Validate inputs
    if (stringIndex >= Tuning::NumStrings) {
        return result;
    }
    if (!isValidBassFrequency(frequencyHz)) {
        return result;
    }

    // Ensure note tables are initialized
    if (!NoteTables::isInitialized()) {
        NoteTables::init();
    }

    // Clamp fret range
    minFret = (minFret < 0) ? 0 : minFret;
    maxFret = (maxFret > Tuning::MaxFret) ? Tuning::MaxFret : maxFret;

    // Find nearest fret by comparing frequencies
    float minCentsAbs = std::numeric_limits<float>::max();
    int bestFret = -1;
    float bestExpectedHz = 0.0f;

    for (int fret = minFret; fret <= maxFret; ++fret) {
        float fretHz = NoteTables::getFrequency(stringIndex, fret);
        float cents = calculateCents(frequencyHz, fretHz);
        float centsAbs = std::abs(cents);

        if (centsAbs < minCentsAbs) {
            minCentsAbs = centsAbs;
            bestFret = fret;
            bestExpectedHz = fretHz;
        }
    }

    // Only accept if reasonably close (within ~1 semitone = 100 cents)
    if (bestFret >= 0 && minCentsAbs < 100.0f) {
        result.fret = bestFret;
        result.expectedHz = bestExpectedHz;
        result.centsOff = calculateCents(frequencyHz, bestExpectedHz);
    }

    return result;
}

uint8_t getMidiNote(size_t stringIndex, int fret) {
    return NoteTables::getMidiNote(stringIndex, fret);
}

uint8_t getMidiChannel(size_t stringIndex) {
    // Channel-per-string: String 0 -> Channel 1, etc.
    if (stringIndex >= Tuning::NumStrings) {
        return 1;
    }
    return static_cast<uint8_t>(stringIndex + 1);
}

float calculateCents(float measuredHz, float referenceHz) {
    if (referenceHz <= 0.0f || measuredHz <= 0.0f) {
        return 0.0f;
    }
    // cents = 1200 * log2(measured / reference)
    return 1200.0f * std::log2(measuredHz / referenceHz);
}

bool isValidBassFrequency(float frequencyHz) {
    return frequencyHz >= MinBassHz && frequencyHz <= MaxBassHz;
}

} // namespace NoteMapping
} // namespace bassmint
