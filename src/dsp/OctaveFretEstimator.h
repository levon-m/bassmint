#pragma once

#include "../core/Tuning.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>

namespace bassmint {

/**
 * Configuration constants for OctaveFretEstimator
 */
namespace OctaveConfig {
    // Decimation settings
    static constexpr size_t DecimationFactor = 10;
    static constexpr float SourceSampleRate = 44100.0f;
    static constexpr float DecimatedSampleRate = SourceSampleRate / DecimationFactor;  // 4410 Hz

    // Analysis window sizes (in decimated samples)
    // E string needs longer window due to low frequency
    static constexpr size_t WindowSizeE = 530;       // ~120ms at 4410 Hz
    static constexpr size_t WindowSizeDefault = 424; // ~96ms at 4410 Hz
    static constexpr size_t MaxWindowSize = 530;

    // Hop size (~10ms)
    static constexpr size_t HopSize = 44;  // ~10ms at 4410 Hz

    // Decimated ring buffer capacity (enough for longest window + some margin)
    static constexpr size_t RingBufferSize = 600;

    // Filter cutoffs
    static constexpr float LowPassCutoff = 800.0f;   // Hz, before decimation
    static constexpr float HighPassCutoff = 20.0f;   // Hz, DC removal

    // NSDF peak detection
    static constexpr float MinPeakHeight = 0.3f;     // Minimum valid peak
    static constexpr float ConfidenceThreshold = 0.6f; // Required for output

    // Bayesian filter parameters
    static constexpr float StayProbability = 0.95f;  // Probability of staying in same octave
    static constexpr float LikelihoodAlpha = 10.0f;  // Exponent for likelihood: exp(alpha * peak)

    // Hand-span prior (frets)
    static constexpr int HandSpanFrets = 6;          // Max expected fret jump

    // Temporal stability
    static constexpr int StabilityFrames = 2;        // Consecutive frames required

    // Onset skip
    static constexpr float OnsetSkipMs = 8.0f;       // Skip first N ms after onset
    static constexpr size_t OnsetSkipSamples =
        static_cast<size_t>(OnsetSkipMs * SourceSampleRate / 1000.0f);

    // Maximum frets per string
    static constexpr int MaxFret = 24;
}

/**
 * Result of fret estimation
 */
struct FretEstimate {
    int stringIndex = -1;       // Active string (0-3, -1 if none)
    int fret = -1;              // Estimated fret (0-24, -1 if invalid)
    float confidence = 0.0f;    // Confidence [0,1]
    float frequencyHz = 0.0f;   // Estimated frequency
    int octaveState = -1;       // 0=LOW (frets 0-11), 1=HIGH (frets 12-24)

    bool isValid() const {
        return stringIndex >= 0 && fret >= 0 && confidence > 0.0f;
    }
};

/**
 * Debug/instrumentation data for each frame
 */
struct OctaveDebugInfo {
    float peakLow = 0.0f;       // NSDF peak in low octave band
    float peakHigh = 0.0f;      // NSDF peak in high octave band
    float beliefLow = 0.5f;     // Bayesian belief: low octave
    float beliefHigh = 0.5f;    // Bayesian belief: high octave
    float freqLow = 0.0f;       // Frequency estimate from low band
    float freqHigh = 0.0f;      // Frequency estimate from high band
    float freqChosen = 0.0f;    // Frequency from chosen band
    int fretChosen = -1;        // Final fret estimate
    int rawFret = -1;           // Fret before stability filter
};

/**
 * Per-string frequency band configuration
 * Computed at initialization based on string tuning
 */
struct StringBandConfig {
    // Low octave band (frets 0-11)
    float lowBandMinFreq;      // Open string frequency
    float lowBandMaxFreq;      // Fret 11 frequency
    size_t lowBandMinLag;      // Corresponding lag (higher freq = lower lag)
    size_t lowBandMaxLag;      // Corresponding lag (lower freq = higher lag)

    // High octave band (frets 12-max)
    float highBandMinFreq;     // Fret 12 frequency
    float highBandMaxFreq;     // Max fret frequency
    size_t highBandMinLag;
    size_t highBandMaxLag;

    // Does this string have a high band? (depends on MaxFret)
    bool hasHighBand;

    // Analysis window size for this string
    size_t windowSize;
};

/**
 * Octave-aware fret estimator using NSDF and Bayesian filtering
 *
 * Designed to solve the octave ambiguity problem in bass pitch detection.
 * Uses NSDF (Normalized Square Difference Function) with separate analysis
 * of low octave (frets 0-11) and high octave (frets 12-24) bands, followed
 * by a 2-state Bayesian filter to track which octave is being played.
 *
 * Key features:
 * - Decimation for efficient low-frequency analysis
 * - Dual-band NSDF peak finding for octave disambiguation
 * - Online Bayesian belief update (no Viterbi needed)
 * - Hand-span prior to constrain unlikely fret jumps
 * - Temporal stability filter to avoid spurious changes
 */
class OctaveFretEstimator {
public:
    static constexpr size_t NumStrings = Tuning::NumStrings;

    /**
     * Construct estimator
     * @param sampleRate Input sample rate (typically 44100)
     */
    explicit OctaveFretEstimator(float sampleRate = OctaveConfig::SourceSampleRate);

    /**
     * Reset all state - call when starting fresh
     * @param sampleRate Optionally change sample rate
     */
    void reset(float sampleRate = OctaveConfig::SourceSampleRate);

    /**
     * Notify that a string has become active (onset detected)
     * Resets per-string state and enables processing
     * @param stringIndex String index (0-3 for E-A-D-G)
     */
    void onStringActive(int stringIndex);

    /**
     * Notify that the string has become inactive (release)
     */
    void onStringInactive();

    /**
     * Process a single raw sample at the source rate (44.1kHz)
     * Handles all preprocessing internally
     * @param rawSample ADC sample from OPT101
     */
    void processSample(float rawSample);

    /**
     * Check if a new frame analysis is ready
     * Call after processSample to get results
     * @return True if new estimate available
     */
    bool hasNewEstimate() const { return hasNewEstimate_; }

    /**
     * Get the latest fret estimate
     * Only valid when hasNewEstimate() returns true
     */
    const FretEstimate& getEstimate() const { return currentEstimate_; }

    /**
     * Get debug info for the latest frame
     */
    const OctaveDebugInfo& getDebugInfo() const { return debugInfo_; }

    /**
     * Get the currently active string index (-1 if none)
     */
    int getActiveString() const { return activeString_; }

    /**
     * Check if estimator is actively processing
     */
    bool isActive() const { return activeString_ >= 0; }

private:
    // Configuration
    float sampleRate_;
    float decimatedRate_;
    std::array<StringBandConfig, NumStrings> stringConfigs_;

    // State: active string
    int activeString_ = -1;

    // Preprocessing: DC removal (1st order high-pass)
    float hpState_ = 0.0f;
    float hpCoeff_ = 0.0f;

    // Preprocessing: Anti-aliasing low-pass (2nd order IIR)
    float lpState1_ = 0.0f;
    float lpState2_ = 0.0f;
    float lpA1_ = 0.0f, lpA2_ = 0.0f;
    float lpB0_ = 0.0f, lpB1_ = 0.0f, lpB2_ = 0.0f;

    // Decimation
    size_t decimationCounter_ = 0;

    // Decimated sample buffer (ring buffer)
    std::array<float, OctaveConfig::RingBufferSize> ringBuffer_;
    size_t ringWriteIndex_ = 0;
    size_t ringCount_ = 0;

    // Hop counter for frame timing
    size_t hopCounter_ = 0;

    // Onset skip counter
    size_t onsetSkipCounter_ = 0;

    // NSDF working buffer
    std::array<float, OctaveConfig::MaxWindowSize> nsdfBuffer_;

    // Bayesian filter state
    float beliefLow_ = 0.5f;
    float beliefHigh_ = 0.5f;

    // Temporal stability state
    int lastFret_ = -1;
    int consecutiveFretCount_ = 0;
    int stableFret_ = -1;

    // Output state
    FretEstimate currentEstimate_;
    OctaveDebugInfo debugInfo_;
    bool hasNewEstimate_ = false;

    // Initialization
    void initStringConfigs();
    void initFilters();

    // Preprocessing
    float applyHighPass(float x);
    float applyLowPass(float x);

    // Ring buffer operations
    void pushDecimatedSample(float x);
    float getRingSample(size_t samplesAgo) const;

    // Core algorithm
    void runFrameAnalysis();
    void computeNSDF(size_t windowSize);

    /**
     * Find best peak in a lag range using parabolic interpolation
     * @param minLag Minimum lag to search
     * @param maxLag Maximum lag to search
     * @param outPeakHeight Output: peak height [0,1]
     * @param outFrequency Output: interpolated frequency
     * @return True if valid peak found
     */
    bool findBandPeak(size_t minLag, size_t maxLag,
                      float& outPeakHeight, float& outFrequency);

    /**
     * Parabolic interpolation around a peak
     * @param index Index of peak
     * @return Interpolated fractional index
     */
    float parabolicInterpolation(size_t index);

    /**
     * Update Bayesian belief with new observations
     */
    void updateBayesianBelief(float peakLow, float peakHigh);

    /**
     * Apply hand-span prior to transitions
     */
    void applyHandSpanPrior(int prevFret, float& transitionLowLow,
                            float& transitionLowHigh, float& transitionHighLow,
                            float& transitionHighHigh);

    /**
     * Convert frequency to fret on active string
     */
    int frequencyToFret(float freq) const;

    /**
     * Apply temporal stability filter
     */
    int applyStabilityFilter(int rawFret);
};

} // namespace bassmint
