#include "StringFretResolver.h"
#include "../core/NoteTables.h"
#include <cmath>

namespace bassmint {

static constexpr float DefaultMinConfidence = 0.5f;

StringFretResolver::StringFretResolver(float sampleRate)
    : sampleRate_(sampleRate)
    , minConfidence_(DefaultMinConfidence)
    , latestPitch_{}
    , hasPitch_(false)
    , stringStates_{}
    , activeString_(std::nullopt)
{
    NoteTables::init();
}

void StringFretResolver::updatePitch(const PitchResult& pitch) {
    latestPitch_ = pitch;
    hasPitch_ = pitch.isValid();
}

void StringFretResolver::updateStringActivity(size_t stringIndex,
                                               const StringActivityState& state) {
    if (stringIndex < NumStrings) {
        stringStates_[stringIndex] = state;
    }
}

void StringFretResolver::setActiveString(std::optional<size_t> activeString) {
    activeString_ = activeString;
}

std::optional<size_t> StringFretResolver::findActiveString() const {
    // If explicitly set, use that
    if (activeString_.has_value()) {
        return activeString_;
    }

    // Otherwise, find highest envelope above threshold
    static constexpr float EnvelopeCloseThreshold = 0.02f;

    size_t best = NumStrings;
    float bestEnvelope = 0.0f;
    uint32_t bestTime = 0;

    for (size_t i = 0; i < NumStrings; ++i) {
        const auto& state = stringStates_[i];
        if (!state.isActive()) {
            continue;
        }

        if (best >= NumStrings) {
            best = i;
            bestEnvelope = state.envelope;
            bestTime = state.lastActiveTime;
        } else if (state.envelope > bestEnvelope + EnvelopeCloseThreshold) {
            best = i;
            bestEnvelope = state.envelope;
            bestTime = state.lastActiveTime;
        } else if (std::abs(state.envelope - bestEnvelope) <= EnvelopeCloseThreshold) {
            if (state.lastActiveTime > bestTime) {
                best = i;
                bestEnvelope = state.envelope;
                bestTime = state.lastActiveTime;
            }
        }
    }

    if (best >= NumStrings) {
        return std::nullopt;
    }
    return best;
}

std::optional<StringFretDecision> StringFretResolver::resolve() const {
    // Step 1: Need valid pitch
    if (!hasPitch_ || !latestPitch_.isValid()) {
        return std::nullopt;
    }

    // Check confidence threshold
    if (latestPitch_.confidence < minConfidence_) {
        return std::nullopt;
    }

    // Step 2: Get active string
    auto activeStr = findActiveString();
    if (!activeStr.has_value()) {
        return std::nullopt;
    }
    size_t stringIdx = activeStr.value();

    // Step 3: Map frequency to fret on active string
    auto fretMatch = NoteMapping::findNearestFret(latestPitch_.frequencyHz, stringIdx);
    if (!fretMatch.isValid()) {
        return std::nullopt;
    }

    // Build decision
    StringFretDecision decision;
    decision.stringIndex = stringIdx;
    decision.fret = fretMatch.fret;
    decision.measuredHz = latestPitch_.frequencyHz;
    decision.expectedHz = fretMatch.expectedHz;
    decision.confidence = latestPitch_.confidence;
    decision.centsOff = fretMatch.centsOff;

    return decision;
}

void StringFretResolver::reset() {
    latestPitch_ = PitchResult{};
    hasPitch_ = false;
    activeString_ = std::nullopt;
    for (auto& state : stringStates_) {
        state = StringActivityState{};
    }
}

void StringFretResolver::setMinConfidence(float confidence) {
    minConfidence_ = std::max(0.0f, std::min(1.0f, confidence));
}

} // namespace bassmint
