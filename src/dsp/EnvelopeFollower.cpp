#include "EnvelopeFollower.h"
#include <cmath>
#include <algorithm>

namespace bassmint {

EnvelopeFollower::EnvelopeFollower(float sampleRate, float attackMs, float releaseMs)
    : sampleRate_(sampleRate)
    , env_(0.0f)
{
    attackCoeff_ = msToCoeff(attackMs);
    releaseCoeff_ = msToCoeff(releaseMs);
}

float EnvelopeFollower::msToCoeff(float ms) const {
    // Time constant: coeff = exp(-1 / (time_in_samples))
    // time_in_samples = ms * sampleRate / 1000
    if (ms <= 0.0f) {
        return 0.0f; // Instant
    }
    float timeInSamples = ms * sampleRate_ / 1000.0f;
    return std::exp(-1.0f / timeInSamples);
}

float EnvelopeFollower::processSample(float x) {
    // Rectify input (take absolute value)
    float rectified = std::abs(x);

    // Apply attack or release coefficient based on signal level
    if (rectified > env_) {
        // Attack: envelope rises toward input
        env_ = attackCoeff_ * env_ + (1.0f - attackCoeff_) * rectified;
    } else {
        // Release: envelope falls toward input
        env_ = releaseCoeff_ * env_ + (1.0f - releaseCoeff_) * rectified;
    }

    return env_;
}

void EnvelopeFollower::reset() {
    env_ = 0.0f;
}

void EnvelopeFollower::setAttackMs(float attackMs) {
    attackCoeff_ = msToCoeff(attackMs);
}

void EnvelopeFollower::setReleaseMs(float releaseMs) {
    releaseCoeff_ = msToCoeff(releaseMs);
}

} // namespace bassmint
