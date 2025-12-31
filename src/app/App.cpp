#include "App.h"
#include "StringFretResolver.h"  // For PitchResult struct
#include <Arduino.h>
#include <cmath>
#include <algorithm>

namespace bassmint {

App::App(const AppConfig& config)
    : config_(config)
    , fretEstimator_(config.sampleRate)
    , stringActivity_(config.sampleRate)
    , resolver_(config.sampleRate)
{
    resolver_.setMinConfidence(config.minPitchConfidence);
}

void App::init() {
    // Initialize hardware
    adc_.init();
    leds_.init();          // Turn on IR LEDs
    leds_.initStatusLed(); // Status LED for debugging
    midi_.init();

    // Initialize note tables
    NoteTables::init();

    // Reset state
    reset();
}

void App::reset() {
    stringActivity_.reset();
    resolver_.reset();
    fretEstimator_.reset(config_.sampleRate);

    noteState_ = NoteState{};
    lastEnvelopes_.fill(0.0f);
    lastStates_.fill(false);
}

void App::update() {
    // Stage 1: Update string activity from OPT101 sensors
    updateStringActivity();

    // Check if any string is active before doing pitch detection
    auto activeString = stringActivity_.getActiveString();

    // Handle string active/inactive transitions for fret estimator
    if (activeString.has_value()) {
        int strIdx = static_cast<int>(activeString.value());
        if (strIdx != lastActiveString_) {
            // String changed - notify estimator
            fretEstimator_.onStringActive(strIdx);
        }
        lastActiveString_ = strIdx;
    } else {
        if (lastActiveString_ >= 0) {
            // String went inactive
            fretEstimator_.onStringInactive();
        }
        lastActiveString_ = -1;
    }

    // Stage 2: Update fret estimation from active string's OPT101 signal
    if (activeString.has_value()) {
        updatePitchDetection();
    }

    // Update resolver with string activity states
    for (size_t s = 0; s < NumStrings; ++s) {
        resolver_.updateStringActivity(s, stringActivity_.getString(s));
    }

    // Set active string in resolver
    resolver_.setActiveString(activeString);

    // Stage 3: Resolve and output MIDI
    resolveAndOutput();

    // Send CC updates
    sendControlChanges();

    // Debug output (Teleplot)
    sendDebugOutput();

    // Increment time for string activity tracking
    stringActivity_.tick();

    // Flush MIDI output
    midi_.flush();
}

void App::updateStringActivity() {
    // Read all OPT101 string sensors
    for (size_t s = 0; s < NumStrings; ++s) {
        float sample = adc_.readStringSample(s);
        stringActivity_.processSample(s, sample);
    }
}

void App::updatePitchDetection() {
    // Get the active string and use its OPT101 signal for fret estimation
    auto activeString = stringActivity_.getActiveString();
    if (!activeString.has_value()) {
        return;
    }

    // Read the active string's OPT101 sample for fret estimation
    // The OPT101 signal contains the string vibration waveform
    // The estimator does its own preprocessing (DC removal, filtering, decimation)
    float stringSample = adc_.readStringSample(activeString.value());

    // Process through the octave-aware fret estimator
    fretEstimator_.processSample(stringSample);

    // If we got a new estimate, update resolver and debug state
    if (fretEstimator_.hasNewEstimate()) {
        const auto& estimate = fretEstimator_.getEstimate();
        const auto& debug = fretEstimator_.getDebugInfo();

        // Update resolver with the fret estimate
        // Create a PitchResult-compatible structure for the resolver
        PitchResult pitchResult;
        pitchResult.frequencyHz = estimate.frequencyHz;
        pitchResult.confidence = estimate.confidence;
        resolver_.updatePitch(pitchResult);

        // Update debug state
        lastPitchHz_ = estimate.frequencyHz;
        lastConfidence_ = estimate.confidence;
        lastFret_ = estimate.fret;
        lastOctaveState_ = estimate.octaveState;

        // Store octave debug info
        lastPeakLow_ = debug.peakLow;
        lastPeakHigh_ = debug.peakHigh;
        lastBeliefLow_ = debug.beliefLow;
        lastBeliefHigh_ = debug.beliefHigh;
    }
}

void App::resolveAndOutput() {
    // Try to resolve current state
    auto decision = resolver_.resolve();

    if (decision.has_value() && decision->isValid()) {
        // We have a valid note - update debug state
        lastFret_ = decision->fret;

        // Get MIDI note for this string+fret
        uint8_t midiNote = NoteMapping::getMidiNote(
            decision->stringIndex, decision->fret);
        uint8_t channel = NoteMapping::getMidiChannel(decision->stringIndex);

        // Check if this is a new note or same note
        if (noteState_.isPlaying) {
            if (midiNote != noteState_.currentNote ||
                channel != noteState_.currentChannel) {
                // Different note - send note off for old, note on for new
                triggerNoteOff();
                triggerNoteOn(*decision);
            }
            // Same note - no action needed (sustain)
        } else {
            // No note playing - start new note
            triggerNoteOn(*decision);
        }
    } else {
        // No valid note
        lastFret_ = -1;

        // Stop any playing note
        if (noteState_.isPlaying) {
            // Check if string is still active (don't immediately stop on brief pitch loss)
            auto activeString = stringActivity_.getActiveString();
            if (!activeString.has_value()) {
                // No active string - stop note
                triggerNoteOff();
            }
        }
    }
}

void App::triggerNoteOn(const StringFretDecision& decision) {
    uint8_t channel = NoteMapping::getMidiChannel(decision.stringIndex);
    uint8_t note = NoteMapping::getMidiNote(decision.stringIndex, decision.fret);

    // Get velocity from string envelope
    const auto& state = stringActivity_.getString(decision.stringIndex);
    uint8_t velocity = envelopeToVelocity(state.envelope);

    // Ensure minimum velocity
    if (velocity < 1) velocity = 1;

    // Send note on
    midi_.sendNoteOn(channel, note, velocity);

    // Update state
    noteState_.isPlaying = true;
    noteState_.currentNote = note;
    noteState_.currentChannel = channel;
}

void App::triggerNoteOff() {
    if (!noteState_.isPlaying) return;

    midi_.sendNoteOff(noteState_.currentChannel, noteState_.currentNote, 0);

    noteState_.isPlaying = false;
    noteState_.currentNote = 0;
    noteState_.currentChannel = 0;
}

void App::sendControlChanges() {
    // Send CC updates for each string
    for (size_t s = 0; s < NumStrings; ++s) {
        const auto& state = stringActivity_.getString(s);

        // Envelope CC (only send if changed significantly)
        float envelope = state.envelope;
        if (std::abs(envelope - lastEnvelopes_[s]) > 0.01f) {
            midi_.sendStringEnvelope(static_cast<uint8_t>(s), envelope);
            lastEnvelopes_[s] = envelope;
        }

        // State CC (only send on change)
        bool active = state.isActive();
        if (active != lastStates_[s]) {
            midi_.sendStringState(static_cast<uint8_t>(s), active);
            lastStates_[s] = active;
        }
    }

    // Send confidence CC for active string if note is playing
    if (noteState_.isPlaying) {
        auto decision = resolver_.resolve();
        if (decision.has_value()) {
            uint8_t stringIdx = static_cast<uint8_t>(decision->stringIndex);
            midi_.sendStringConfidence(stringIdx, decision->confidence);
        }
    }
}

uint8_t App::envelopeToVelocity(float envelope) const {
    // Non-linear mapping for more natural velocity response
    // envelope 0.0-1.0 -> velocity 1-127

    // Apply curve (sqrt for softer response)
    float curved = std::sqrt(envelope);

    // Scale to MIDI range
    int velocity = static_cast<int>(curved * 126.0f) + 1;

    return static_cast<uint8_t>(std::clamp(velocity, 1, 127));
}

void App::sendDebugOutput() {
    if constexpr (!BoardConfig::DebugEnabled) {
        return;
    }

    // Rate limit debug output to DebugOutputHz
    static constexpr uint32_t samplesPerDebug =
        static_cast<uint32_t>(BoardConfig::SampleRate / BoardConfig::DebugOutputHz);

    ++debugSampleCounter_;
    if (debugSampleCounter_ < samplesPerDebug) {
        return;
    }
    debugSampleCounter_ = 0;

    // Raw ADC values (0-1 normalized) - E and D strings only
    Serial.print(">adc_E:");
    Serial.println(adc_.readStringSample(0), 4);
    Serial.print(">adc_D:");
    Serial.println(adc_.readStringSample(2), 4);

    // AC values after DC blocking - E and D strings only
    Serial.print(">ac_E:");
    Serial.println(stringActivity_.getString(0).acValue, 4);
    Serial.print(">ac_D:");
    Serial.println(stringActivity_.getString(2).acValue, 4);

    // String envelopes - E and D strings only
    Serial.print(">env_E:");
    Serial.println(stringActivity_.getString(0).envelope, 4);
    Serial.print(">env_D:");
    Serial.println(stringActivity_.getString(2).envelope, 4);

    // Per-string attack thresholds (envelope must exceed this to become active)
    Serial.print(">thresh_E:");
    Serial.println(stringActivity_.getAttackThreshold(0), 4);
    Serial.print(">thresh_D:");
    Serial.println(stringActivity_.getAttackThreshold(2), 4);

    // String active states (0=idle, 1=active) - E and D only
    Serial.print(">active_E:");
    Serial.println(stringActivity_.getString(0).isActive() ? 1 : 0);
    Serial.print(">active_D:");
    Serial.println(stringActivity_.getString(2).isActive() ? 1 : 0);

    // Pitch detection results
    Serial.print(">pitch_hz:");
    Serial.println(lastPitchHz_);
    Serial.print(">confidence:");
    Serial.println(lastConfidence_);

    // Active string (-1 if none, 0-3 for E/A/D/G)
    Serial.print(">string:");
    Serial.println(lastActiveString_);

    // Detected fret (-1 if none, 0-24 for fret number)
    Serial.print(">fret:");
    Serial.println(lastFret_);

    // Octave disambiguation debug info
    Serial.print(">octave_state:");
    Serial.println(lastOctaveState_);  // 0=LOW, 1=HIGH
    Serial.print(">peak_low:");
    Serial.println(lastPeakLow_);
    Serial.print(">peak_high:");
    Serial.println(lastPeakHigh_);
    Serial.print(">belief_low:");
    Serial.println(lastBeliefLow_);
    Serial.print(">belief_high:");
    Serial.println(lastBeliefHigh_);

    // Current MIDI note being played (0 if none)
    Serial.print(">midi_note:");
    Serial.println(noteState_.isPlaying ? noteState_.currentNote : 0);
}

} // namespace bassmint
