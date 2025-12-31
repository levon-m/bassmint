#pragma once

#include "../core/Tuning.h"
#include "../core/NoteMapping.h"
#include "../dsp/StringActivity.h"
#include <array>
#include <optional>

namespace bassmint {

/**
 * Result of pitch detection
 * Used to pass pitch estimates from OctaveFretEstimator to StringFretResolver
 */
struct PitchResult {
    float frequencyHz = 0.0f;
    float confidence  = 0.0f;

    bool isValid() const {
        return confidence > 0.0f && frequencyHz > 0.0f;
    }
};

/**
 * Final decision output combining string and fret information
 */
struct StringFretDecision {
    size_t stringIndex = 0;    // String (0-3: E, A, D, G)
    int    fret        = -1;   // Fret number (0-24), -1 if invalid
    float  measuredHz  = 0.0f; // Measured frequency
    float  expectedHz  = 0.0f; // Expected frequency at fret
    float  confidence  = 0.0f; // Pitch confidence (0-1)
    float  centsOff    = 0.0f; // Tuning deviation in cents

    bool isValid() const { return fret >= 0; }
};

/**
 * Combines string activity and pitch detection to produce final note decisions
 *
 * Algorithm:
 * 1. Get active string from StringActivity
 * 2. Map detected pitch to nearest fret on active string
 * 3. Output final decision with confidence
 */
class StringFretResolver {
public:
    static constexpr size_t NumStrings = Tuning::NumStrings;

    explicit StringFretResolver(float sampleRate);

    void updatePitch(const PitchResult& pitch);
    void updateStringActivity(size_t stringIndex, const StringActivityState& state);
    void setActiveString(std::optional<size_t> activeString);
    std::optional<StringFretDecision> resolve() const;
    void reset();
    void setMinConfidence(float confidence);

private:
    float sampleRate_;
    float minConfidence_;

    PitchResult latestPitch_;
    bool hasPitch_;

    std::array<StringActivityState, NumStrings> stringStates_;
    std::optional<size_t> activeString_;

    std::optional<size_t> findActiveString() const;
};

} // namespace bassmint
