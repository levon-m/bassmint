#include "Tuning.h"
#include <cmath>

namespace bassmint {
namespace Tuning {

float getFretFrequency(size_t stringIndex, int fret) {
    if (stringIndex >= NumStrings || fret < 0 || fret > MaxFret) {
        return 0.0f;
    }

    float openHz = OpenStringHz[stringIndex];
    // f = f0 * 2^(fret/12)
    return openHz * std::pow(2.0f, static_cast<float>(fret) / 12.0f);
}

uint8_t getMidiNote(size_t stringIndex, int fret) {
    if (stringIndex >= NumStrings || fret < 0 || fret > MaxFret) {
        return 0;
    }

    return OpenStringMidiNote[stringIndex] + static_cast<uint8_t>(fret);
}

} // namespace Tuning
} // namespace bassmint
