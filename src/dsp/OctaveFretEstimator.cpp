#include "OctaveFretEstimator.h"
#include <algorithm>
#include <cmath>

namespace bassmint {

// ============================================================================
// Construction & Initialization
// ============================================================================

OctaveFretEstimator::OctaveFretEstimator(float sampleRate)
    : sampleRate_(sampleRate)
    , decimatedRate_(sampleRate / OctaveConfig::DecimationFactor)
{
    initStringConfigs();
    initFilters();
    ringBuffer_.fill(0.0f);
    nsdfBuffer_.fill(0.0f);
}

void OctaveFretEstimator::reset(float sampleRate) {
    sampleRate_ = sampleRate;
    decimatedRate_ = sampleRate / OctaveConfig::DecimationFactor;

    // Reset filter state
    hpState_ = 0.0f;
    lpState1_ = 0.0f;
    lpState2_ = 0.0f;

    // Reset decimation
    decimationCounter_ = 0;

    // Reset ring buffer
    ringBuffer_.fill(0.0f);
    ringWriteIndex_ = 0;
    ringCount_ = 0;

    // Reset hop counter
    hopCounter_ = 0;
    onsetSkipCounter_ = 0;

    // Reset Bayesian state
    beliefLow_ = 0.5f;
    beliefHigh_ = 0.5f;

    // Reset stability state
    lastFret_ = -1;
    consecutiveFretCount_ = 0;
    stableFret_ = -1;

    // Reset output
    activeString_ = -1;
    currentEstimate_ = FretEstimate{};
    debugInfo_ = OctaveDebugInfo{};
    hasNewEstimate_ = false;

    initFilters();
}

void OctaveFretEstimator::initStringConfigs() {
    for (size_t s = 0; s < NumStrings; ++s) {
        StringBandConfig& cfg = stringConfigs_[s];
        float openFreq = Tuning::OpenStringHz[s];

        // Low octave band: frets 0-11
        cfg.lowBandMinFreq = openFreq;
        cfg.lowBandMaxFreq = openFreq * std::pow(2.0f, 11.0f / 12.0f);

        // Convert frequency to lag: lag = fs / f
        // Higher freq = lower lag, so min/max swap
        cfg.lowBandMinLag = static_cast<size_t>(decimatedRate_ / cfg.lowBandMaxFreq) - 1;
        cfg.lowBandMaxLag = static_cast<size_t>(std::ceil(decimatedRate_ / cfg.lowBandMinFreq)) + 1;

        // High octave band: frets 12-MaxFret
        cfg.highBandMinFreq = openFreq * 2.0f;  // Octave up = fret 12
        cfg.highBandMaxFreq = openFreq * std::pow(2.0f, OctaveConfig::MaxFret / 12.0f);

        // Check if high band exists on this string (fret 12+ available)
        cfg.hasHighBand = (OctaveConfig::MaxFret >= 12);

        if (cfg.hasHighBand) {
            cfg.highBandMinLag = static_cast<size_t>(decimatedRate_ / cfg.highBandMaxFreq);
            if (cfg.highBandMinLag < 2) cfg.highBandMinLag = 2;  // Minimum practical lag
            cfg.highBandMaxLag = static_cast<size_t>(std::ceil(decimatedRate_ / cfg.highBandMinFreq)) + 1;
        } else {
            cfg.highBandMinLag = 0;
            cfg.highBandMaxLag = 0;
        }

        // Window size: E string gets longer window
        cfg.windowSize = (s == 0) ? OctaveConfig::WindowSizeE : OctaveConfig::WindowSizeDefault;
    }
}

void OctaveFretEstimator::initFilters() {
    // High-pass filter for DC removal (1st order IIR)
    // Coefficient: alpha = 1 / (1 + 2*pi*fc/fs)
    // For fc = 20Hz at 44100Hz: alpha ≈ 0.9986
    float omega_hp = 2.0f * 3.14159265f * OctaveConfig::HighPassCutoff / sampleRate_;
    hpCoeff_ = 1.0f / (1.0f + omega_hp);

    // Low-pass filter (2nd order Butterworth) for anti-aliasing
    // Design for fc = 800Hz at 44100Hz
    float fc = OctaveConfig::LowPassCutoff;
    float omega = 2.0f * 3.14159265f * fc / sampleRate_;
    float cos_w = std::cos(omega);
    float sin_w = std::sin(omega);
    float alpha = sin_w / (2.0f * 0.7071f);  // Q = sqrt(2)/2 for Butterworth

    float a0 = 1.0f + alpha;
    lpA1_ = (-2.0f * cos_w) / a0;
    lpA2_ = (1.0f - alpha) / a0;
    lpB0_ = ((1.0f - cos_w) / 2.0f) / a0;
    lpB1_ = (1.0f - cos_w) / a0;
    lpB2_ = ((1.0f - cos_w) / 2.0f) / a0;
}

// ============================================================================
// String Activity Callbacks
// ============================================================================

void OctaveFretEstimator::onStringActive(int stringIndex) {
    if (stringIndex < 0 || stringIndex >= static_cast<int>(NumStrings)) {
        return;
    }

    activeString_ = stringIndex;

    // Reset per-note state
    beliefLow_ = 0.5f;
    beliefHigh_ = 0.5f;
    lastFret_ = -1;
    consecutiveFretCount_ = 0;
    stableFret_ = -1;
    hopCounter_ = 0;
    onsetSkipCounter_ = 0;

    // Don't reset filter state or ring buffer - we want continuous processing
    // But mark that we need to skip onset transient
    hasNewEstimate_ = false;
    currentEstimate_ = FretEstimate{};
}

void OctaveFretEstimator::onStringInactive() {
    activeString_ = -1;
    hasNewEstimate_ = false;
    currentEstimate_ = FretEstimate{};
}

// ============================================================================
// Preprocessing Filters
// ============================================================================

float OctaveFretEstimator::applyHighPass(float x) {
    // 1st order high-pass: y[n] = alpha * (y[n-1] + x[n] - x[n-1])
    // Simplified as: y[n] = alpha * y[n-1] + alpha * (x[n] - x[n-1])
    float y = hpCoeff_ * (hpState_ + x);
    hpState_ = y - x;  // Store for next iteration (this is equivalent)
    return y;
}

float OctaveFretEstimator::applyLowPass(float x) {
    // 2nd order IIR (biquad) direct form II transposed
    float y = lpB0_ * x + lpState1_;
    lpState1_ = lpB1_ * x - lpA1_ * y + lpState2_;
    lpState2_ = lpB2_ * x - lpA2_ * y;
    return y;
}

// ============================================================================
// Ring Buffer Operations
// ============================================================================

void OctaveFretEstimator::pushDecimatedSample(float x) {
    ringBuffer_[ringWriteIndex_] = x;
    ringWriteIndex_ = (ringWriteIndex_ + 1) % OctaveConfig::RingBufferSize;
    if (ringCount_ < OctaveConfig::RingBufferSize) {
        ++ringCount_;
    }
}

float OctaveFretEstimator::getRingSample(size_t samplesAgo) const {
    if (samplesAgo >= ringCount_) return 0.0f;
    size_t idx = (ringWriteIndex_ + OctaveConfig::RingBufferSize - 1 - samplesAgo)
                 % OctaveConfig::RingBufferSize;
    return ringBuffer_[idx];
}

// ============================================================================
// Main Sample Processing
// ============================================================================

void OctaveFretEstimator::processSample(float rawSample) {
    hasNewEstimate_ = false;

    // Track onset skip
    if (activeString_ >= 0 && onsetSkipCounter_ < OctaveConfig::OnsetSkipSamples) {
        ++onsetSkipCounter_;
    }

    // Stage 1: DC removal (high-pass)
    float hp = applyHighPass(rawSample);

    // Stage 2: Anti-aliasing low-pass
    float lp = applyLowPass(hp);

    // Stage 3: Decimation
    ++decimationCounter_;
    if (decimationCounter_ >= OctaveConfig::DecimationFactor) {
        decimationCounter_ = 0;

        // Store decimated sample
        pushDecimatedSample(lp);

        // Only process if string is active and past onset transient
        if (activeString_ >= 0 && onsetSkipCounter_ >= OctaveConfig::OnsetSkipSamples) {
            ++hopCounter_;
            if (hopCounter_ >= OctaveConfig::HopSize) {
                hopCounter_ = 0;

                // Check we have enough samples for analysis
                const StringBandConfig& cfg = stringConfigs_[activeString_];
                if (ringCount_ >= cfg.windowSize) {
                    runFrameAnalysis();
                }
            }
        }
    }
}

// ============================================================================
// Frame Analysis (NSDF + Bayesian + Fret Estimation)
// ============================================================================

void OctaveFretEstimator::runFrameAnalysis() {
    const StringBandConfig& cfg = stringConfigs_[activeString_];

    // Step 1: Compute NSDF
    computeNSDF(cfg.windowSize);

    // Step 2: Find peak in low octave band
    float peakLow = 0.0f, freqLow = 0.0f;
    bool foundLow = findBandPeak(cfg.lowBandMinLag, cfg.lowBandMaxLag, peakLow, freqLow);

    // Step 3: Find peak in high octave band (if exists)
    float peakHigh = 0.0f, freqHigh = 0.0f;
    bool foundHigh = false;
    if (cfg.hasHighBand && cfg.highBandMinLag < cfg.highBandMaxLag) {
        foundHigh = findBandPeak(cfg.highBandMinLag, cfg.highBandMaxLag, peakHigh, freqHigh);
    }

    // Store debug info
    debugInfo_.peakLow = peakLow;
    debugInfo_.peakHigh = peakHigh;
    debugInfo_.freqLow = freqLow;
    debugInfo_.freqHigh = freqHigh;

    // Step 4: Update Bayesian belief
    updateBayesianBelief(peakLow, peakHigh);
    debugInfo_.beliefLow = beliefLow_;
    debugInfo_.beliefHigh = beliefHigh_;

    // Step 5: Choose octave based on belief
    int octaveState = (beliefHigh_ > beliefLow_) ? 1 : 0;
    float chosenPeak = (octaveState == 1) ? peakHigh : peakLow;
    float chosenFreq = (octaveState == 1) ? freqHigh : freqLow;
    bool validPeak = (octaveState == 1) ? foundHigh : foundLow;

    debugInfo_.freqChosen = chosenFreq;

    // Step 6: Convert frequency to fret
    int rawFret = -1;
    if (validPeak && chosenPeak >= OctaveConfig::MinPeakHeight) {
        rawFret = frequencyToFret(chosenFreq);

        // Validate fret is in correct octave band
        if (octaveState == 0 && rawFret > 11) rawFret = 11;
        if (octaveState == 1 && rawFret < 12) rawFret = 12;
        if (rawFret > OctaveConfig::MaxFret) rawFret = OctaveConfig::MaxFret;
        if (rawFret < 0) rawFret = 0;
    }
    debugInfo_.rawFret = rawFret;

    // Step 7: Apply temporal stability filter
    int stableFret = applyStabilityFilter(rawFret);
    debugInfo_.fretChosen = stableFret;

    // Step 8: Build output estimate
    float confidence = std::max(beliefLow_, beliefHigh_) * chosenPeak;

    currentEstimate_.stringIndex = activeString_;
    currentEstimate_.fret = stableFret;
    currentEstimate_.confidence = confidence;
    currentEstimate_.frequencyHz = chosenFreq;
    currentEstimate_.octaveState = octaveState;

    hasNewEstimate_ = true;
}

// ============================================================================
// NSDF Computation (McLeod Pitch Method)
// ============================================================================

void OctaveFretEstimator::computeNSDF(size_t windowSize) {
    // NSDF(tau) = 2 * r(tau) / (m(0) + m(tau))
    // where r(tau) = sum_{i} x[i] * x[i+tau]  (autocorrelation)
    //       m(tau) = sum_{i} x[i]^2 + sum_{i} x[i+tau]^2

    // Compute m(0) first (sum of squares over window)
    float sumSq = 0.0f;
    for (size_t i = 0; i < windowSize; ++i) {
        float sample = getRingSample(i);
        sumSq += sample * sample;
    }

    // For each lag, compute NSDF
    size_t maxLag = windowSize / 2;
    nsdfBuffer_[0] = 1.0f;  // NSDF(0) = 1 by definition

    float runningSumSqLeft = sumSq;
    float runningSumSqRight = sumSq;

    for (size_t tau = 1; tau < maxLag; ++tau) {
        // Update running sums (remove one element from each end)
        float leftSample = getRingSample(windowSize - tau);
        float rightSample = getRingSample(tau - 1);
        runningSumSqLeft -= leftSample * leftSample;
        runningSumSqRight -= rightSample * rightSample;

        // Compute autocorrelation at lag tau
        float r = 0.0f;
        size_t N = windowSize - tau;
        for (size_t i = 0; i < N; ++i) {
            r += getRingSample(i + tau) * getRingSample(i);
        }

        // Compute normalization: m(0) + m(tau) for the overlapping region
        // Actually using the incremental approach is tricky, so compute directly
        float sumSqL = 0.0f, sumSqR = 0.0f;
        for (size_t i = 0; i < N; ++i) {
            float sl = getRingSample(i);
            float sr = getRingSample(i + tau);
            sumSqL += sl * sl;
            sumSqR += sr * sr;
        }

        float denom = sumSqL + sumSqR;
        if (denom > 1e-10f) {
            nsdfBuffer_[tau] = 2.0f * r / denom;
        } else {
            nsdfBuffer_[tau] = 0.0f;
        }
    }
}

// ============================================================================
// Peak Finding with Parabolic Interpolation
// ============================================================================

bool OctaveFretEstimator::findBandPeak(size_t minLag, size_t maxLag,
                                        float& outPeakHeight, float& outFrequency) {
    outPeakHeight = 0.0f;
    outFrequency = 0.0f;

    if (minLag >= maxLag || minLag < 1) return false;

    // Clamp to buffer bounds
    size_t bufferMax = OctaveConfig::MaxWindowSize / 2 - 1;
    if (maxLag > bufferMax) maxLag = bufferMax;
    if (minLag > bufferMax) return false;

    // Find the best peak (highest local maximum above zero crossing)
    float bestPeak = -1.0f;
    size_t bestLag = 0;
    bool wasNegative = nsdfBuffer_[minLag] < 0.0f;

    for (size_t tau = minLag + 1; tau < maxLag; ++tau) {
        float val = nsdfBuffer_[tau];
        float prevVal = nsdfBuffer_[tau - 1];

        // Detect zero crossing (negative to positive)
        if (wasNegative && val >= 0.0f) {
            wasNegative = false;
        }

        // After a zero crossing, look for peaks
        if (!wasNegative) {
            // Check for local maximum
            if (tau + 1 < maxLag) {
                float nextVal = nsdfBuffer_[tau + 1];
                if (val > prevVal && val >= nextVal && val > bestPeak) {
                    bestPeak = val;
                    bestLag = tau;
                }
            }
        }

        if (val < 0.0f) {
            wasNegative = true;
        }
    }

    // If no peak found after zero crossing, just take the maximum
    if (bestPeak < 0.0f) {
        for (size_t tau = minLag; tau < maxLag; ++tau) {
            if (nsdfBuffer_[tau] > bestPeak) {
                bestPeak = nsdfBuffer_[tau];
                bestLag = tau;
            }
        }
    }

    if (bestLag < minLag || bestPeak < OctaveConfig::MinPeakHeight) {
        return false;
    }

    // Parabolic interpolation for sub-sample precision
    float interpLag = parabolicInterpolation(bestLag);
    if (interpLag < 1.0f) interpLag = 1.0f;  // Avoid division by zero

    outPeakHeight = bestPeak;
    outFrequency = decimatedRate_ / interpLag;

    return true;
}

float OctaveFretEstimator::parabolicInterpolation(size_t index) {
    if (index == 0 || index >= OctaveConfig::MaxWindowSize / 2 - 1) {
        return static_cast<float>(index);
    }

    float y0 = nsdfBuffer_[index - 1];
    float y1 = nsdfBuffer_[index];
    float y2 = nsdfBuffer_[index + 1];

    float denom = 2.0f * (2.0f * y1 - y0 - y2);
    if (std::abs(denom) < 1e-10f) {
        return static_cast<float>(index);
    }

    float delta = (y0 - y2) / denom;

    // Clamp interpolation to reasonable range
    if (delta > 0.5f) delta = 0.5f;
    if (delta < -0.5f) delta = -0.5f;

    return static_cast<float>(index) + delta;
}

// ============================================================================
// Bayesian Belief Update
// ============================================================================

void OctaveFretEstimator::updateBayesianBelief(float peakLow, float peakHigh) {
    const StringBandConfig& cfg = stringConfigs_[activeString_];

    // Observation likelihoods: p(obs|state) ∝ exp(alpha * peak)
    float alpha = OctaveConfig::LikelihoodAlpha;

    // Clamp peaks to [0, 1] before exponentiation
    peakLow = std::max(0.0f, std::min(1.0f, peakLow));
    peakHigh = std::max(0.0f, std::min(1.0f, peakHigh));

    float obsLow = std::exp(alpha * peakLow);
    float obsHigh = cfg.hasHighBand ? std::exp(alpha * peakHigh) : 0.001f;

    // Transition matrix
    float stay = OctaveConfig::StayProbability;
    float switchP = 1.0f - stay;

    // Apply hand-span prior if we have a previous fret
    float tLL = stay, tLH = switchP;
    float tHL = switchP, tHH = stay;

    if (lastFret_ >= 0) {
        applyHandSpanPrior(lastFret_, tLL, tLH, tHL, tHH);
    }

    // Prediction step: b' = T^T * b
    float predLow = tLL * beliefLow_ + tHL * beliefHigh_;
    float predHigh = tLH * beliefLow_ + tHH * beliefHigh_;

    // Update step: b'' = O * b'
    float updLow = obsLow * predLow;
    float updHigh = obsHigh * predHigh;

    // Normalize
    float sum = updLow + updHigh;
    if (sum > 1e-10f) {
        beliefLow_ = updLow / sum;
        beliefHigh_ = updHigh / sum;
    } else {
        // Reset to uniform if degenerate
        beliefLow_ = 0.5f;
        beliefHigh_ = 0.5f;
    }
}

void OctaveFretEstimator::applyHandSpanPrior(int prevFret, float& tLL, float& tLH,
                                              float& tHL, float& tHH) {
    // Determine which octave the previous fret is in
    bool prevInLow = (prevFret <= 11);
    bool prevInHigh = (prevFret >= 12);

    // For each octave band, check if any frets are within hand-span distance
    // Low band: frets 0-11
    // High band: frets 12-24

    int handSpan = OctaveConfig::HandSpanFrets;

    // Can we reach any fret in low band from prevFret?
    int closestLow = std::max(0, std::min(11, prevFret));
    int distToLow = std::abs(prevFret - closestLow);

    // Can we reach any fret in high band from prevFret?
    int closestHigh = std::max(12, std::min(OctaveConfig::MaxFret, prevFret));
    int distToHigh = std::abs(prevFret - closestHigh);

    // If only one band is reachable within hand span, strongly prefer it
    bool lowReachable = (distToLow <= handSpan);
    bool highReachable = (distToHigh <= handSpan);

    if (lowReachable && !highReachable) {
        // Can only reach low band
        tLL = 0.98f;
        tLH = 0.02f;
        tHL = 0.90f;  // Even if in high, likely moving to low
        tHH = 0.10f;
    } else if (highReachable && !lowReachable) {
        // Can only reach high band
        tLL = 0.10f;
        tLH = 0.90f;
        tHL = 0.02f;
        tHH = 0.98f;
    }
    // If both reachable, leave transitions at base values
}

// ============================================================================
// Fret Estimation
// ============================================================================

int OctaveFretEstimator::frequencyToFret(float freq) const {
    if (activeString_ < 0) return -1;

    float openFreq = Tuning::OpenStringHz[activeString_];
    if (freq < openFreq * 0.9f) return -1;  // Below open string

    // fret = 12 * log2(freq / openFreq)
    float fretFloat = 12.0f * std::log2(freq / openFreq);
    int fret = static_cast<int>(std::round(fretFloat));

    // Validate: check if frequency is within ~50 cents of the fret
    float expectedFreq = openFreq * std::pow(2.0f, fret / 12.0f);
    float centsDiff = 1200.0f * std::log2(freq / expectedFreq);

    if (std::abs(centsDiff) > 50.0f) {
        return -1;  // Too far from any valid fret
    }

    return fret;
}

int OctaveFretEstimator::applyStabilityFilter(int rawFret) {
    if (rawFret < 0) {
        // Invalid input - keep previous stable fret
        return stableFret_;
    }

    if (rawFret == lastFret_) {
        ++consecutiveFretCount_;
    } else {
        lastFret_ = rawFret;
        consecutiveFretCount_ = 1;
    }

    // Require K consecutive frames with same fret
    if (consecutiveFretCount_ >= OctaveConfig::StabilityFrames) {
        stableFret_ = rawFret;
    }

    return stableFret_;
}

} // namespace bassmint
