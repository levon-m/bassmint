#pragma once

#include "core/Types.h"
#include "dsp/RingBuffer.h"
#include "dsp/EnvelopeFollower.h"
#include "dsp/PitchDetectorYin.h"
#include <array>

namespace BassMINT {

/**
 * @brief Complete DSP processing chain for a single bass string
 *
 * Combines:
 * - Ring buffer (receives samples from ADC ISR)
 * - Envelope follower (detects string activity)
 * - YIN pitch detector (estimates fundamental frequency)
 *
 * Designed to be instantiated once per string (4 instances total).
 */
class StringProcessor {
public:
    /**
     * @brief Constructor
     * @param stringId Which string this processor handles
     * @param sampleRate Sample rate in Hz
     */
    explicit StringProcessor(StringId stringId, float sampleRate = SAMPLE_RATE_HZ);

    /**
     * @brief Push new ADC sample (called from ISR context)
     * @param rawSample 12-bit ADC value (0-4095)
     * @return true if sample accepted, false if buffer full
     */
    bool pushSample(uint16_t rawSample);

    /**
     * @brief Process available samples (main loop context)
     * Updates envelope follower and runs pitch detection if enough samples available
     */
    void process();

    /**
     * @brief Get current string state
     */
    StringState getState() const { return state_; }

    /**
     * @brief Check if string is active
     */
    bool isActive() const {
        return state_ == StringState::Active || state_ == StringState::Attack;
    }

    /**
     * @brief Get latest pitch estimate
     * @return Most recent pitch estimate (may be invalid if confidence low)
     */
    const PitchEstimate& getLatestPitch() const { return latestPitch_; }

    /**
     * @brief Get string ID
     */
    StringId getStringId() const { return stringId_; }

    /**
     * @brief Reset processor state
     */
    void reset();

    /**
     * @brief Get number of samples in buffer
     */
    size_t getBufferLevel() const { return sampleBuffer_.getAvailable(); }

    /**
     * @brief Get current envelope value (for debugging/plotting)
     */
    float getEnvelope() const { return envelopeFollower_.getEnvelope(); }

    /**
     * @brief Get accumulated sample count in float buffer (for debugging)
     */
    size_t getAccumulatedSamples() const { return floatBufferIndex_; }

    /**
     * @brief Get latest raw ADC value (for debugging sensor signal)
     */
    uint16_t getLatestRawAdc() const { return latestRawAdc_; }

    /**
     * @brief Get ADC min/max from current frame (shows signal swing)
     */
    uint16_t getAdcMin() const { return adcMin_; }
    uint16_t getAdcMax() const { return adcMax_; }
    uint32_t getSampleCount() const { return sampleCount_; }
    float getDcEstimate() const { return dcEstimate_; }

private:
    StringId stringId_;
    float sampleRate_;
    StringState state_;

    // DSP components
    RingBuffer<uint16_t, RING_BUFFER_SIZE> sampleBuffer_;
    EnvelopeFollower envelopeFollower_;
    PitchDetectorYin pitchDetector_;

    // Working buffers
    std::array<float, PITCH_FRAME_SIZE> floatBuffer_;
    std::array<uint16_t, PITCH_FRAME_SIZE> rawBuffer_;
    size_t floatBufferIndex_;  // Current write position in floatBuffer_

    // State tracking
    PitchEstimate latestPitch_;
    bool wasActive_;
    uint16_t latestRawAdc_;  // For debugging sensor signal
    uint16_t adcMin_;        // Min ADC value in current frame
    uint16_t adcMax_;        // Max ADC value in current frame
    uint32_t sampleCount_;   // Total samples processed (for debugging)
    float dcEstimate_;       // Adaptive DC baseline estimate (tracks actual ADC center)

    /**
     * @brief Convert raw ADC sample to normalized float
     * @param raw 12-bit ADC value (0-4095)
     * @return Normalized value (-1.0 to 1.0, DC-centered)
     */
    float normalizeAdcSample(uint16_t raw) const;

    /**
     * @brief Update state machine based on envelope
     */
    void updateState();
};

} // namespace BassMINT
