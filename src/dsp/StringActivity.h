#pragma once

#include "EnvelopeFollower.h"
#include <array>
#include <optional>
#include <cstddef>
#include <cstdint>

namespace bassmint {

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

    // Default thresholds (can be calibrated)
    static constexpr float DefaultAttackThreshold  = 0.1f;   // Become active above this
    static constexpr float DefaultReleaseThreshold = 0.05f;  // Become idle below this
    static constexpr float EnvelopeCloseThreshold  = 0.02f;  // Consider envelopes "close"

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
     * Set attack threshold (string becomes active above this)
     */
    void setAttackThreshold(float threshold);

    /**
     * Set release threshold (string becomes idle below this)
     */
    void setReleaseThreshold(float threshold);

    /**
     * Increment internal timestamp (call once per sample)
     */
    void tick();

private:
    std::array<EnvelopeFollower, NumStrings> followers_;
    std::array<StringActivityState, NumStrings> states_;

    float attackThreshold_;
    float releaseThreshold_;

    int lastActiveString_;   // -1 if none
    uint32_t currentTime_;   // Sample counter for timing

    /**
     * Update the state of a single string based on its envelope
     */
    void updateStringState(size_t stringIndex, float envelope);
};

} // namespace bassmint
