#include "StringActivity.h"
#include <algorithm>
#include <cmath>

namespace bassmint {

// Envelope follower parameters for OPT101 string sensors
// Fast attack to catch plucks, moderate release for sustain
static constexpr float EnvelopeAttackMs  = 5.0f;
static constexpr float EnvelopeReleaseMs = 100.0f;

StringActivity::StringActivity(float sampleRate)
    : followers_{
        EnvelopeFollower(sampleRate, EnvelopeAttackMs, EnvelopeReleaseMs),
        EnvelopeFollower(sampleRate, EnvelopeAttackMs, EnvelopeReleaseMs),
        EnvelopeFollower(sampleRate, EnvelopeAttackMs, EnvelopeReleaseMs),
        EnvelopeFollower(sampleRate, EnvelopeAttackMs, EnvelopeReleaseMs)
    }
    , states_{}
    , attackThreshold_(DefaultAttackThreshold)
    , releaseThreshold_(DefaultReleaseThreshold)
    , lastActiveString_(-1)
    , currentTime_(0)
{
}

void StringActivity::processSample(size_t stringIndex, float sample) {
    if (stringIndex >= NumStrings) {
        return;
    }

    // Update envelope
    float envelope = followers_[stringIndex].processSample(sample);

    // Update string state
    updateStringState(stringIndex, envelope);
}

void StringActivity::updateStringState(size_t stringIndex, float envelope) {
    auto& state = states_[stringIndex];
    state.envelope = envelope;

    StringState prevState = state.state;

    // Hysteresis: different thresholds for attack and release
    if (state.state == StringState::Idle) {
        if (envelope > attackThreshold_) {
            state.state = StringState::Active;
            state.lastActiveTime = currentTime_;
            lastActiveString_ = static_cast<int>(stringIndex);
        }
    } else {
        // Currently Active
        if (envelope < releaseThreshold_) {
            state.state = StringState::Idle;
        }
    }
}

const StringActivityState& StringActivity::getString(size_t stringIndex) const {
    static const StringActivityState emptyState{};
    if (stringIndex >= NumStrings) {
        return emptyState;
    }
    return states_[stringIndex];
}

std::optional<size_t> StringActivity::getActiveString() const {
    // Find the string with highest envelope that is active
    // If envelopes are close, prefer the most recently activated

    size_t bestString = NumStrings;
    float bestEnvelope = 0.0f;
    uint32_t bestTime = 0;

    for (size_t i = 0; i < NumStrings; ++i) {
        if (!states_[i].isActive()) {
            continue;
        }

        float env = states_[i].envelope;
        uint32_t time = states_[i].lastActiveTime;

        if (bestString >= NumStrings) {
            // First active string found
            bestString = i;
            bestEnvelope = env;
            bestTime = time;
        } else if (env > bestEnvelope + EnvelopeCloseThreshold) {
            // This string has significantly higher envelope
            bestString = i;
            bestEnvelope = env;
            bestTime = time;
        } else if (std::abs(env - bestEnvelope) <= EnvelopeCloseThreshold) {
            // Envelopes are close, prefer more recent
            if (time > bestTime) {
                bestString = i;
                bestEnvelope = env;
                bestTime = time;
            }
        }
        // else: current best has higher envelope, keep it
    }

    if (bestString >= NumStrings) {
        return std::nullopt;
    }

    return bestString;
}

std::optional<size_t> StringActivity::getLastPluckedString() const {
    if (lastActiveString_ < 0) {
        return std::nullopt;
    }
    return static_cast<size_t>(lastActiveString_);
}

void StringActivity::reset() {
    for (size_t i = 0; i < NumStrings; ++i) {
        followers_[i].reset();
        states_[i] = StringActivityState{};
    }
    lastActiveString_ = -1;
    currentTime_ = 0;
}

void StringActivity::setAttackThreshold(float threshold) {
    attackThreshold_ = std::max(0.0f, threshold);
}

void StringActivity::setReleaseThreshold(float threshold) {
    releaseThreshold_ = std::max(0.0f, threshold);
}

void StringActivity::tick() {
    ++currentTime_;
}

} // namespace bassmint
