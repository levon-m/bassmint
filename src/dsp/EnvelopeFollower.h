#pragma once

namespace bassmint {

/**
 * Envelope follower with independent attack and release times
 *
 * Uses exponential smoothing for natural-sounding envelope tracking.
 * Fast attack captures transients, slow release maintains sustain.
 */
class EnvelopeFollower {
public:
    /**
     * Construct envelope follower
     * @param sampleRate Audio sample rate in Hz
     * @param attackMs Attack time in milliseconds (how fast envelope rises)
     * @param releaseMs Release time in milliseconds (how fast envelope falls)
     */
    EnvelopeFollower(float sampleRate, float attackMs, float releaseMs);

    /**
     * Process a single sample
     * @param x Input sample (typically rectified or squared)
     * @return Current envelope value
     */
    float processSample(float x);

    /**
     * Get current envelope value without processing
     */
    float getValue() const { return env_; }

    /**
     * Reset envelope to zero
     */
    void reset();

    /**
     * Update attack time
     */
    void setAttackMs(float attackMs);

    /**
     * Update release time
     */
    void setReleaseMs(float releaseMs);

private:
    float sampleRate_;
    float env_;
    float attackCoeff_;
    float releaseCoeff_;

    /**
     * Convert time constant to exponential coefficient
     */
    float msToCoeff(float ms) const;
};

} // namespace bassmint
