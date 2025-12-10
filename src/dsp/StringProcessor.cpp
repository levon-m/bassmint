#include "dsp/StringProcessor.h"
#include <algorithm>

namespace BassMINT {

// ADC midpoint for DC removal (12-bit ADC, centered at 2048)
static constexpr float ADC_MIDPOINT = 2048.0f;
static constexpr float ADC_SCALE = 1.0f / 2048.0f;

StringProcessor::StringProcessor(StringId stringId, float sampleRate)
    : stringId_(stringId)
    , sampleRate_(sampleRate)
    , state_(StringState::Idle)
    , envelopeFollower_(sampleRate)
    , pitchDetector_(sampleRate, PITCH_FRAME_SIZE)
    , floatBufferIndex_(0)
    , wasActive_(false)
    , latestRawAdc_(2048)
    , adcMin_(2048)  // Start at midpoint, will find true min
    , adcMax_(2048)  // Start at midpoint, will find true max
    , sampleCount_(0)
    , dcEstimate_(2048.0f)  // Will adapt to actual baseline
{
    floatBuffer_.fill(0.0f);
    rawBuffer_.fill(0);

    // TODO: Tune envelope follower parameters per-string if needed
    // Different strings may have different optical characteristics
    envelopeFollower_.setThreshold(0.15f);  // Adjust based on testing
    envelopeFollower_.setHysteresis(0.6f);
}

bool StringProcessor::pushSample(uint16_t rawSample) {
    // ISR context - must be fast!
    return sampleBuffer_.push(rawSample);
}

void StringProcessor::process() {
    // Main loop context

    // Read available samples from ring buffer
    size_t available = sampleBuffer_.getAvailable();

    if (available == 0) {
        return; // Nothing to process
    }

    // Process samples in chunks up to remaining buffer space
    size_t spaceRemaining = PITCH_FRAME_SIZE - floatBufferIndex_;
    size_t toProcess = std::min(available, spaceRemaining);

    // Read from ring buffer
    size_t read = sampleBuffer_.read(rawBuffer_.data(), toProcess);

    if (read == 0) {
        return;
    }

    // Convert to float, accumulate in floatBuffer_, and update envelope
    // DC tracking coefficient: very slow adaptation (~0.5 second time constant at 8kHz)
    // This tracks thermal drift but doesn't follow the AC signal
    constexpr float DC_ALPHA = 0.0005f;  // ~2000 samples to converge = 250ms

    for (size_t i = 0; i < read; ++i) {
        uint16_t raw = rawBuffer_[i];
        latestRawAdc_ = raw;
        sampleCount_++;

        // Track min/max with decay toward current value (sliding window effect)
        adcMin_ = (adcMin_ < raw) ? adcMin_ : raw;
        adcMax_ = (adcMax_ > raw) ? adcMax_ : raw;
        adcMin_ = adcMin_ + ((raw > adcMin_) ? 1 : 0);
        adcMax_ = adcMax_ - ((raw < adcMax_) ? 1 : 0);

        // Adaptive DC tracking - slowly follow the signal baseline
        // This handles thermal drift and different sensor setups
        dcEstimate_ = dcEstimate_ + DC_ALPHA * (static_cast<float>(raw) - dcEstimate_);

        // Normalize using adaptive DC estimate instead of fixed 2048
        float centered = static_cast<float>(raw) - dcEstimate_;
        float sample = centered * ADC_SCALE;

        floatBuffer_[floatBufferIndex_++] = sample;
        envelopeFollower_.update(sample);
    }

    // Update state machine
    updateState();

    // Run pitch detection when we have a full frame
    if (floatBufferIndex_ >= PITCH_FRAME_SIZE) {
        // Only run pitch detection if string is active
        if (isActive()) {
            latestPitch_ = pitchDetector_.estimate(floatBuffer_.data(), PITCH_FRAME_SIZE);

            // Reject low-confidence estimates
            if (latestPitch_.confidence < MIN_PITCH_CONFIDENCE) {
                latestPitch_ = PitchEstimate(); // Invalidate
            }
        }

        // Reset buffer for next frame (sliding window could be used for lower latency)
        floatBufferIndex_ = 0;
        // Note: adcMin_/adcMax_ use decay, not reset, for continuous monitoring
    }
}

void StringProcessor::reset() {
    sampleBuffer_.clear();
    envelopeFollower_.reset();
    state_ = StringState::Idle;
    floatBufferIndex_ = 0;
    wasActive_ = false;
    latestPitch_ = PitchEstimate();
}

float StringProcessor::normalizeAdcSample(uint16_t raw) const {
    // Remove DC bias and normalize to [-1.0, 1.0]
    // OPT101 output is biased around Vcc/2, so ADC reads ~2048 at rest
    float centered = static_cast<float>(raw) - ADC_MIDPOINT;
    return centered * ADC_SCALE;
}

void StringProcessor::updateState() {
    bool currentlyActive = envelopeFollower_.isActive();

    // State transitions
    if (!wasActive_ && currentlyActive) {
        // Idle -> Attack
        state_ = StringState::Attack;
    } else if (wasActive_ && !currentlyActive) {
        // Active -> Release
        state_ = StringState::Release;
    } else if (currentlyActive) {
        // Attack -> Active (after first frame)
        if (state_ == StringState::Attack) {
            state_ = StringState::Active;
        }
    } else {
        // Release -> Idle
        if (state_ == StringState::Release) {
            state_ = StringState::Idle;
            latestPitch_ = PitchEstimate(); // Clear pitch on idle
        }
    }

    wasActive_ = currentlyActive;
}

} // namespace BassMINT
