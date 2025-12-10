#include "PitchDetector.h"
#include <cmath>
#include <algorithm>

namespace bassmint {

PitchDetector::PitchDetector(float sampleRate, size_t frameSize)
    : sampleRate_(sampleRate)
    , frameSize_(std::min(frameSize, MaxFrameSize))
    , threshold_(DefaultThreshold)
    , hopSize_(frameSize_ / 2)
    , samplesSinceLastFrame_(0)
    , hpPrevInput_(0.0f)
    , hpPrevOutput_(0.0f)
{
    // Compute lag bounds from frequency range
    // tau = sampleRate / frequency
    // minLag corresponds to maxFrequency, maxLag to minFrequency
    minLag_ = static_cast<size_t>(sampleRate_ / MaxFrequency);
    maxLag_ = static_cast<size_t>(sampleRate_ / MinFrequency);

    // Clamp maxLag to half frame size (YIN requirement)
    maxLag_ = std::min(maxLag_, frameSize_ / 2);

    // High-pass filter coefficient (1-pole)
    // fc = HpCutoff Hz
    float dt = 1.0f / sampleRate_;
    float rc = 1.0f / (2.0f * 3.14159265f * HpCutoff);
    hpCoeff_ = rc / (rc + dt);

    // Pre-compute Hann window
    for (size_t i = 0; i < frameSize_; ++i) {
        window_[i] = 0.5f * (1.0f - std::cos(2.0f * 3.14159265f * i / (frameSize_ - 1)));
    }

    // Initialize working buffers
    differenceBuffer_.fill(0.0f);
    cmndfBuffer_.fill(0.0f);
}

void PitchDetector::reset() {
    inputBuffer_.clear();
    samplesSinceLastFrame_ = 0;
    hpPrevInput_ = 0.0f;
    hpPrevOutput_ = 0.0f;
}

void PitchDetector::setThreshold(float threshold) {
    threshold_ = std::clamp(threshold, 0.01f, 1.0f);
}

float PitchDetector::applyHighPass(float x) {
    // 1-pole high-pass filter: y[n] = alpha * (y[n-1] + x[n] - x[n-1])
    float out = hpCoeff_ * (hpPrevOutput_ + x - hpPrevInput_);
    hpPrevInput_ = x;
    hpPrevOutput_ = out;
    return out;
}

void PitchDetector::applyWindow(float* frame, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        frame[i] *= window_[i];
    }
}

std::optional<PitchResult> PitchDetector::processSample(float x) {
    // Apply high-pass filter
    float filtered = applyHighPass(x);

    // Push to ring buffer
    inputBuffer_.push(filtered);
    ++samplesSinceLastFrame_;

    // Check if we have enough samples and it's time for a new frame (hop size reached)
    if (inputBuffer_.size() >= frameSize_ && samplesSinceLastFrame_ >= hopSize_) {
        // Copy samples from ring buffer to contiguous array for processing
        std::array<float, MaxFrameSize> frame;
        inputBuffer_.copyNewest(frame.data(), frameSize_);

        // Process the frame
        PitchResult result = processFrame(frame.data(), frameSize_);

        // Reset hop counter
        samplesSinceLastFrame_ = 0;

        return result;
    }

    return std::nullopt;
}

PitchResult PitchDetector::processFrame(const float* samples, size_t count) {
    PitchResult result;

    if (count < minLag_ * 2) {
        return result; // Frame too small
    }

    // Copy and window the frame
    std::array<float, MaxFrameSize> frame;
    size_t actualSize = std::min(count, MaxFrameSize);
    for (size_t i = 0; i < actualSize; ++i) {
        frame[i] = samples[i];
    }
    applyWindow(frame.data(), actualSize);

    // YIN Step 1: Compute difference function
    computeDifference(frame.data(), actualSize);

    // YIN Step 2: Compute CMNDF
    computeCMNDF(actualSize / 2);

    // YIN Step 3: Find best tau
    size_t bestTau = findBestTau();

    if (bestTau == 0) {
        return result; // No valid pitch found
    }

    // YIN Step 4: Parabolic interpolation
    float refinedTau = parabolicInterpolation(bestTau);

    // Convert tau to frequency
    result.frequencyHz = sampleRate_ / refinedTau;

    // Confidence = 1 - CMNDF value at best tau
    result.confidence = 1.0f - cmndfBuffer_[bestTau];

    // Validate frequency is in bass range
    if (result.frequencyHz < MinFrequency || result.frequencyHz > MaxFrequency) {
        result.confidence = 0.0f;
        result.frequencyHz = 0.0f;
    }

    return result;
}

void PitchDetector::computeDifference(const float* frame, size_t size) {
    // d(tau) = sum((x[i] - x[i+tau])^2) for i = 0 to W-tau-1
    // where W = size/2 (we analyze first half against second half)

    size_t halfSize = size / 2;

    for (size_t tau = 0; tau < halfSize; ++tau) {
        float sum = 0.0f;
        for (size_t i = 0; i < halfSize; ++i) {
            float diff = frame[i] - frame[i + tau];
            sum += diff * diff;
        }
        differenceBuffer_[tau] = sum;
    }
}

void PitchDetector::computeCMNDF(size_t size) {
    // CMNDF: d'(tau) = d(tau) / ((1/tau) * sum(d(j)) for j=1 to tau)
    // Special case: d'(0) = 1

    cmndfBuffer_[0] = 1.0f;

    float runningSum = 0.0f;
    for (size_t tau = 1; tau < size; ++tau) {
        runningSum += differenceBuffer_[tau];
        if (runningSum > 0.0f) {
            cmndfBuffer_[tau] = differenceBuffer_[tau] * tau / runningSum;
        } else {
            cmndfBuffer_[tau] = 1.0f;
        }
    }
}

size_t PitchDetector::findBestTau() {
    // Find first tau where CMNDF dips below threshold
    // then find the minimum in that dip

    size_t bestTau = 0;
    float bestValue = 1.0f;

    // Start searching from minLag (corresponding to maxFrequency)
    bool foundDip = false;

    for (size_t tau = minLag_; tau < maxLag_; ++tau) {
        if (cmndfBuffer_[tau] < threshold_) {
            // Found a dip below threshold
            if (!foundDip) {
                foundDip = true;
                bestTau = tau;
                bestValue = cmndfBuffer_[tau];
            } else if (cmndfBuffer_[tau] < bestValue) {
                bestTau = tau;
                bestValue = cmndfBuffer_[tau];
            }
        } else if (foundDip) {
            // We've exited the dip, stop searching
            break;
        }
    }

    // If no dip found below threshold, find global minimum in range
    if (!foundDip) {
        for (size_t tau = minLag_; tau < maxLag_; ++tau) {
            if (cmndfBuffer_[tau] < bestValue) {
                bestTau = tau;
                bestValue = cmndfBuffer_[tau];
            }
        }
        // Only accept if reasonably confident
        if (bestValue > 0.5f) {
            return 0;
        }
    }

    return bestTau;
}

float PitchDetector::parabolicInterpolation(size_t tau) {
    // Fit parabola through points at tau-1, tau, tau+1
    // Return the x value of the minimum

    if (tau < 1 || tau >= maxLag_ - 1) {
        return static_cast<float>(tau);
    }

    float y0 = cmndfBuffer_[tau - 1];
    float y1 = cmndfBuffer_[tau];
    float y2 = cmndfBuffer_[tau + 1];

    // Parabola: y = a*x^2 + b*x + c
    // Minimum at x = -b/(2a)
    // Using finite differences:
    float denom = y0 - 2.0f * y1 + y2;
    if (std::abs(denom) < 1e-10f) {
        return static_cast<float>(tau);
    }

    float delta = 0.5f * (y0 - y2) / denom;
    return static_cast<float>(tau) + delta;
}

} // namespace bassmint
