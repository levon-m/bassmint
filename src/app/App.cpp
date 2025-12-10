<<<<<<< HEAD
#include "App.h"
#include <cmath>
#include <algorithm>
=======
#include "app/App.h"
#include "hal/BoardConfig.h"
#include <cstdio>
#include "pico/stdlib.h"  // For stdio_flush()
>>>>>>> 491b711ec2cfccc73c60c377d0b2d86c73cefe06

#if BoardConfig::DebugEnabled
#include <Arduino.h>
#endif

<<<<<<< HEAD
namespace bassmint {

App::App(const AppConfig& config)
    : config_(config)
    , pitchDetector_(config.sampleRate, config.pitchFrameSize)
    , stringActivity_(config.sampleRate)
    , resolver_(config.sampleRate)
=======
App::App()
    : stringProcessors_{
        StringProcessor(StringId::E),
        StringProcessor(StringId::A),
        StringProcessor(StringId::D),
        StringProcessor(StringId::G)
    }
    , stringManagers_{
        StringManager(StringId::E, midiOut_),
        StringManager(StringId::A, midiOut_),
        StringManager(StringId::D, midiOut_),
        StringManager(StringId::G, midiOut_)
    }
    , loopCounter_(0)
    , lastStatsTime_(0)
    , lastTeleplotTime_(0)
>>>>>>> 491b711ec2cfccc73c60c377d0b2d86c73cefe06
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
    pitchDetector_.reset();

    noteState_ = NoteState{};
    lastEnvelopes_.fill(0.0f);
    lastStates_.fill(false);
}

void App::update() {
    // Stage 1: Update string activity from OPT101 sensors
    updateStringActivity();

    // Check if any string is active before doing pitch detection
    auto activeString = stringActivity_.getActiveString();

    // Stage 2: Update pitch detection from piezo (only if string active)
    if (activeString.has_value()) {
        updatePitchDetection();
        lastActiveString_ = static_cast<int>(activeString.value());
    } else {
        lastActiveString_ = -1;
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
    // Read piezo sample
    float piezoSample = adc_.readPiezoSample();

    // Process through pitch detector
    auto pitchResult = pitchDetector_.processSample(piezoSample);

    // If we got a pitch result, update resolver and debug state
    if (pitchResult.has_value()) {
        resolver_.updatePitch(*pitchResult);
        lastPitchHz_ = pitchResult->frequencyHz;
        lastConfidence_ = pitchResult->confidence;
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

<<<<<<< HEAD
        // Stop any playing note
        if (noteState_.isPlaying) {
            // Check if string is still active (don't immediately stop on brief pitch loss)
            auto activeString = stringActivity_.getActiveString();
            if (!activeString.has_value()) {
                // No active string - stop note
                triggerNoteOff();
            }
        }
=======
    // Start ADC sampling
    adcDriver_.startSampling();

    lastStatsTime_ = Timer::getTimeMillis();

    printf("BassMINT initialized successfully!\n");
    printf("Sample rate: %lu Hz\n", SAMPLE_RATE_HZ);
    printf("Frame size: %lu samples\n", PITCH_FRAME_SIZE);
    printf("Ready to rock.\n");

    return true;
}

void App::run() {
    // Main loop - runs forever
    while (true) {
        tick();
>>>>>>> 491b711ec2cfccc73c60c377d0b2d86c73cefe06
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

<<<<<<< HEAD
    // Send note on
    midi_.sendNoteOn(channel, note, velocity);

    // Update state
    noteState_.isPlaying = true;
    noteState_.currentNote = note;
    noteState_.currentChannel = channel;
=======
    uint32_t now = Timer::getTimeMillis();

    #ifdef BASSMINT_DEBUG_STATS
    // Teleplot output at ~100Hz for real-time visualization
    if (now - lastTeleplotTime_ >= 10) {
        printTeleplot();
        lastTeleplotTime_ = now;
    }
    #endif

    // Print text stats every 5 seconds (goes to Teleplot console)
    if (now - lastStatsTime_ >= 5000) {
        printStats();
        lastStatsTime_ = now;
        loopCounter_ = 0;
    }
>>>>>>> 491b711ec2cfccc73c60c377d0b2d86c73cefe06
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

<<<<<<< HEAD
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
#if BoardConfig::DebugEnabled
    // Rate limit debug output to DebugOutputHz
    static constexpr uint32_t samplesPerDebug =
        static_cast<uint32_t>(BoardConfig::SampleRate / BoardConfig::DebugOutputHz);

    ++debugSampleCounter_;
    if (debugSampleCounter_ < samplesPerDebug) {
        return;
    }
    debugSampleCounter_ = 0;

    // String envelopes (one plot per string)
    Serial.print(">env_E:");
    Serial.println(stringActivity_.getString(0).envelope);
    Serial.print(">env_A:");
    Serial.println(stringActivity_.getString(1).envelope);
    Serial.print(">env_D:");
    Serial.println(stringActivity_.getString(2).envelope);
    Serial.print(">env_G:");
    Serial.println(stringActivity_.getString(3).envelope);

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

    // Current MIDI note being played (0 if none)
    Serial.print(">midi_note:");
    Serial.println(noteState_.isPlaying ? noteState_.currentNote : 0);
#endif
}

} // namespace bassmint
=======
void App::printStats() {
    // Print text stats to Teleplot console (non-plot messages)
    #ifdef BASSMINT_DEBUG_STATS
    const auto& proc = stringProcessors_[static_cast<uint8_t>(StringId::A)];
    const auto& mgr = stringManagers_[static_cast<uint8_t>(StringId::A)];

    printf("[A] loops/sec:%lu acc:%zu pitch:%.1fHz conf:%.2f midi:%u fret:%d %s\n",
           loopCounter_ / 5,  // Average over 5 seconds
           proc.getAccumulatedSamples(),
           proc.getLatestPitch().frequencyHz,
           proc.getLatestPitch().confidence,
           mgr.getCurrentMidiNote(),
           mgr.getCurrentFret(),
           mgr.isNoteOn() ? "[ON]" : "");
    #else
    (void)loopCounter_;
    #endif
}

void App::printTeleplot() {
    // Output Teleplot-compatible telemetry for string A only
    // Format: >varName:value\n
    const auto& proc = stringProcessors_[static_cast<uint8_t>(StringId::A)];

    // Current raw ADC value
    printf(">adc:%u\n", proc.getLatestRawAdc());

    // Adaptive DC baseline (should track the ADC center automatically)
    printf(">dc:%.0f\n", proc.getDcEstimate());

    // ADC signal swing (max - min)
    uint16_t adcMin = proc.getAdcMin();
    uint16_t adcMax = proc.getAdcMax();
    int16_t swing = (adcMax > adcMin) ? (adcMax - adcMin) : 0;
    printf(">swing:%d\n", swing);

    // Envelope value (should now be ~0 at rest, spike on pluck)
    printf(">env:%.3f\n", proc.getEnvelope());

    // Detected pitch in Hz (0 when not detected)
    printf(">pitch:%.1f\n", proc.getLatestPitch().frequencyHz);

    // Flush USB buffer immediately for real-time response
    stdio_flush();
}

} // namespace BassMINT
>>>>>>> 491b711ec2cfccc73c60c377d0b2d86c73cefe06
