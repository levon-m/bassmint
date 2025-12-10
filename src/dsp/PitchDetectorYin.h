#pragma once

#include "core/Types.h"
#include <cstddef>
#include <cstdint>
#include <array>

namespace BassMINT {

/**
 * @brief YIN pitch detection algorithm optimized for bass guitar
 *
 * Implements the YIN algorithm (de Cheveigné & Kawahara, 2002) with
 * optimizations for bass frequency range (30-400 Hz).
 *
 * YIN improvements over autocorrelation:
 * - Difference function instead of correlation
 * - Cumulative mean normalized difference (CMND)
 * - Parabolic interpolation for sub-sample accuracy
 *
 * Tuned for bass:
 * - Min freq ~30 Hz (below E1 for headroom)
 * - Max freq ~400 Hz (above typical bass range)
 * - Window size chosen for low latency while resolving E1
 */
class PitchDetectorYin {
public:
    /**
     * @brief Constructor
     * @param sampleRate Sample rate in Hz (e.g., 8000)
     * @param bufferSize Analysis window size in samples (e.g., 512)
     * @param minFreq Minimum detectable frequency in Hz (e.g., 30)
     * @param maxFreq Maximum detectable frequency in Hz (e.g., 400)
     */
    PitchDetectorYin(float sampleRate = SAMPLE_RATE_HZ,
                     size_t bufferSize = PITCH_FRAME_SIZE,
                     float minFreq = 30.0f,
                     float maxFreq = 400.0f);

    /**
     * @brief Estimate pitch from audio buffer
     * @param samples Input audio samples (length = bufferSize)
     * @param count Number of samples (must equal bufferSize)
     * @return Pitch estimate with frequency and confidence
     */
    PitchEstimate estimate(const float* samples, size_t count);

    /**
     * @brief Set confidence threshold for valid pitch
     * @param threshold Minimum confidence (0.0-1.0)
     */
    void setConfidenceThreshold(float threshold) {
        confidenceThreshold_ = threshold;
    }

    /**
     * @brief Get current confidence threshold
     */
    float getConfidenceThreshold() const {
        return confidenceThreshold_;
    }

private:
    float sampleRate_;
    size_t bufferSize_;
    float minFreq_;
    float maxFreq_;
    float confidenceThreshold_;

    // Pre-calculated lag bounds
    size_t minLag_;
    size_t maxLag_;

    // Working buffers (avoid dynamic allocation)
    // MAX_LAG must accommodate PITCH_FRAME_SIZE/2 for full frequency range
    static constexpr size_t MAX_LAG = 1024;
    std::array<float, MAX_LAG> differenceFunction_;
    std::array<float, MAX_LAG> cmndf_;

    /**
     * @brief Compute difference function
     * @param samples Input samples
     */
    void computeDifference(const float* samples);

    /**
     * @brief Compute cumulative mean normalized difference
     */
    void computeCMNDF();

    /**
     * @brief Find absolute threshold minimum in CMNDF
     * @return Lag of minimum, or 0 if none found
     */
    size_t absoluteThreshold();

    /**
     * @brief Parabolic interpolation for sub-sample accuracy
     * @param tau Integer lag estimate
     * @return Refined lag with fractional part
     */
    float parabolicInterpolation(size_t tau);
};

} // namespace BassMINT
