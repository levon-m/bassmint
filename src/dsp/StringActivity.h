#pragma once

#include "EnvelopeFollower.h"
#include <array>
#include <optional>
#include <cstddef>
#include <cstdint>

namespace bassmint {

/**
 * Simple DC blocker (1-pole high-pass filter)
 * Removes DC offset to extract AC component (string vibration)
 * Cutoff is approximately (1-coeff) * sampleRate / (2*pi)
 * With coeff=0.995 at 44.1kHz, cutoff ~35Hz
 */
class DcBlocker {
public:
    explicit DcBlocker(float coefficient = 0.995f)
        : coeff_(coefficient), x1_(0.0f), y1_(0.0f) {}

    float process(float x) {
        // y[n] = x[n] - x[n-1] + coeff * y[n-1]
        float y = x - x1_ + coeff_ * y1_;
        x1_ = x;
        y1_ = y;
        return y;
    }

    void reset() {
        x1_ = 0.0f;
        y1_ = 0.0f;
    }

private:
    float coeff_;
    float x1_;
    float y1_;
};

/**
 * String state: Idle or Active
 */
enum class StringState {
    Idle,
    Active
};

/**
 * Activity state for a single string
 */
struct StringActivityState {
    StringState state    = StringState::Idle;
    float       envelope = 0.0f;
    float       acValue  = 0.0f;     // Last AC value (after DC blocking) for debug
    uint32_t    lastActiveTime = 0;  // Timestamp when string became active

    bool isActive() const { return state == StringState::Active; }
};

/**
 * Manages activity detection for all 4 bass strings using OPT101 sensors
 *
 * Determines:
 * - Per-string envelope values
 * - Which string is currently active (above threshold)
 * - Which string was most recently plucked (monophonic assumption)
 *
 * Active string selection logic:
 * - Highest envelope above attack threshold wins
 * - If envelopes are close, most recently activated string wins
 * - Prevents false triggers from sympathetic vibration
 */
class StringActivity {
public:
    static constexpr size_t NumStrings = 4;

    // Default thresholds - per-string due to varying sensor sensitivity
    // E string has larger AC swings (~0.25), D string has smaller (~0.01)
    static constexpr float DefaultAttackThresholds[NumStrings] = {
        0.05f,   // E - AC swings ~0.25, threshold at 20%
        0.05f,   // A - (disabled for now, using E value)
        0.002f,  // D - AC swings ~0.01, threshold at 20%
        0.002f   // G - (disabled for now, using D value)
    };
    static constexpr float DefaultReleaseThresholds[NumStrings] = {
        0.025f,  // E - half of attack
        0.025f,  // A
        0.001f,  // D - half of attack
        0.001f   // G
    };
    static constexpr float EnvelopeCloseThreshold  = 0.005f;  // Consider envelopes "close"

    /**
     * Construct string activity tracker
     * @param sampleRate Audio sample rate in Hz
     */
    explicit StringActivity(float sampleRate);

    /**
     * Process a sample from a specific string's OPT101 sensor
     * @param stringIndex String index (0-3: E, A, D, G)
     * @param sample Raw sensor sample
     */
    void processSample(size_t stringIndex, float sample);

    /**
     * Get the activity state for a specific string
     */
    const StringActivityState& getString(size_t stringIndex) const;

    /**
     * Get the currently active string (highest envelope, most recent if tied)
     * @return String index (0-3) or nullopt if no string is active
     */
    std::optional<size_t> getActiveString() const;

    /**
     * Get the most recently plucked string (even if now idle)
     * @return String index or nullopt if never plucked
     */
    std::optional<size_t> getLastPluckedString() const;

    /**
     * Reset all string states
     */
    void reset();

    /**
     * Set attack threshold for a specific string
     */
    void setAttackThreshold(size_t stringIndex, float threshold);

    /**
     * Set release threshold for a specific string
     */
    void setReleaseThreshold(size_t stringIndex, float threshold);

    /**
     * Get attack threshold for a specific string (for debugging)
     */
    float getAttackThreshold(size_t stringIndex) const;

    /**
     * Increment internal timestamp (call once per sample)
     */
    void tick();

private:
    std::array<DcBlocker, NumStrings> dcBlockers_;         // Remove DC bias
    std::array<EnvelopeFollower, NumStrings> followers_;   // Track AC envelope
    std::array<StringActivityState, NumStrings> states_;

    std::array<float, NumStrings> attackThresholds_;
    std::array<float, NumStrings> releaseThresholds_;

    int lastActiveString_;   // -1 if none
    uint32_t currentTime_;   // Sample counter for timing

    /**
     * Update the state of a single string based on its envelope
     */
    void updateStringState(size_t stringIndex, float envelope);
};

} // namespace bassmint
