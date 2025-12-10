#pragma once

#include "RingBuffer.h"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <array>

namespace bassmint {

/**
 * Result of pitch detection
 */
struct PitchResult {
    float frequencyHz = 0.0f;
    float confidence  = 0.0f;

    bool isValid() const {
        return confidence > 0.0f && frequencyHz > 0.0f;
    }
};

/**
 * Bass-optimized YIN pitch detector
 *
 * Implements the YIN algorithm with optimizations for bass guitar:
 * - Frequency range: 30-300 Hz (bass fundamentals)
 * - Frame size: 40-80ms depending on sample rate
 * - Pre-processing with high-pass filter to remove DC/mechanical drift
 * - Hann windowing for spectral leakage reduction
 */
class PitchDetector {
public:
    // Bass frequency range
    static constexpr float MinFrequency = 30.0f;   // ~B0 (lowest bass note)
    static constexpr float MaxFrequency = 300.0f;  // Upper harmonics/high frets

    // YIN algorithm parameters
    static constexpr float DefaultThreshold = 0.15f;
    static constexpr size_t MaxFrameSize = 4096;

    /**
     * Construct pitch detector
     * @param sampleRate Audio sample rate in Hz
     * @param frameSize Number of samples per analysis frame
     */
    PitchDetector(float sampleRate, size_t frameSize);

    /**
     * Process a single sample and return pitch result when frame is ready
     * @param x Input sample from piezo sensor
     * @return PitchResult when a full frame has been analyzed, nullopt otherwise
     */
    std::optional<PitchResult> processSample(float x);

    /**
     * Process entire frame at once (alternative to sample-by-sample)
     * @param samples Pointer to sample buffer
     * @param count Number of samples
     * @return PitchResult result
     */
    PitchResult processFrame(const float* samples, size_t count);

    /**
     * Reset internal state
     */
    void reset();

    /**
     * Set YIN threshold (lower = stricter pitch detection)
     */
    void setThreshold(float threshold);

    /**
     * Get current threshold
     */
    float getThreshold() const { return threshold_; }

    /**
     * Get frame size
     */
    size_t getFrameSize() const { return frameSize_; }

    /**
     * Get current sample count in buffer
     */
    size_t getSampleCount() const { return inputBuffer_.size(); }

private:
    // Configuration
    float sampleRate_;
    size_t frameSize_;
    float threshold_;

    // Lag bounds (computed from frequency range)
    size_t minLag_;
    size_t maxLag_;

    // Ring buffer for sample accumulation
    RingBuffer<float, MaxFrameSize> inputBuffer_;

    // Hop size for overlap processing (50% overlap)
    size_t hopSize_;
    size_t samplesSinceLastFrame_;

    // YIN working buffers
    std::array<float, MaxFrameSize / 2> differenceBuffer_;
    std::array<float, MaxFrameSize / 2> cmndfBuffer_;

    // High-pass filter state (removes DC/sub-20Hz)
    float hpPrevInput_;
    float hpPrevOutput_;
    static constexpr float HpCutoff = 20.0f;
    float hpCoeff_;

    // Window function (Hann)
    std::array<float, MaxFrameSize> window_;

    /**
     * Apply high-pass filter to remove DC and sub-bass rumble
     */
    float applyHighPass(float x);

    /**
     * Apply Hann window to frame
     */
    void applyWindow(float* frame, size_t size);

    /**
     * YIN Step 1: Compute difference function
     */
    void computeDifference(const float* frame, size_t size);

    /**
     * YIN Step 2: Compute cumulative mean normalized difference function
     */
    void computeCMNDF(size_t size);

    /**
     * YIN Step 3: Find best tau using absolute threshold
     */
    size_t findBestTau();

    /**
     * YIN Step 4: Parabolic interpolation for sub-sample precision
     */
    float parabolicInterpolation(size_t tau);
};

} // namespace bassmint
