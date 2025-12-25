#pragma once

#include "../dsp/PitchDetector.h"
#include "../dsp/StringActivity.h"
#include "StringFretResolver.h"
#include "../core/NoteTables.h"
#include "../core/NoteMapping.h"
#include "../hal/AdcDriver.h"
#include "../hal/LedDriver.h"
#include "../hal/MidiOutput.h"
#include "../hal/BoardConfig.h"
#include <array>
#include <cstdint>

namespace bassmint {

/**
 * Configuration for the App
 */
struct AppConfig {
    float sampleRate = BoardConfig::SampleRate;
    size_t pitchFrameSize = BoardConfig::PitchFrameSize;
    float minPitchConfidence = 0.5f;
};

/**
 * State tracking for note-on/off management
 */
struct NoteState {
    bool isPlaying = false;
    uint8_t currentNote = 0;
    uint8_t currentChannel = 0;
};

/**
 * Main application class - orchestrates all BassMINT components
 *
 * Flow:
 * 1. Read OPT101 sensors -> String Activity (which string is playing?)
 * 2. Read Piezo sensor -> Pitch Detection (what note?)
 * 3. Resolve string + pitch -> fret decision
 * 4. Output MIDI messages
 */
class App {
public:
    static constexpr size_t NumStrings = BoardConfig::NumStrings;

    /**
     * Construct with default config, or pass custom config
     */
    explicit App(const AppConfig& config = AppConfig{});

    /**
     * Initialize all subsystems - call once at startup
     */
    void init();

    /**
     * Main update loop - call once per sample
     */
    void update();

    /**
     * Reset all state
     */
    void reset();

    // Accessors for debugging/monitoring
    const StringActivity& getStringActivity() const { return stringActivity_; }
    const StringFretResolver& getResolver() const { return resolver_; }
    const NoteState& getNoteState() const { return noteState_; }

private:
    AppConfig config_;

    // Hardware drivers (default-constructed, initialized in init())
    AdcDriver adc_;
    LedDriver leds_;
    MidiOutput midi_;

    // DSP components (constructed with sample rate in constructor)
    PitchDetector pitchDetector_;
    StringActivity stringActivity_;
    StringFretResolver resolver_;

    // State management (default-initialized here, no need for init list)
    NoteState noteState_ = {};
    std::array<float, NumStrings> lastEnvelopes_ = {};
    std::array<bool, NumStrings> lastStates_ = {};

    // Debug state (Teleplot output)
    uint32_t debugSampleCounter_ = 0;
    float lastPitchHz_ = 0.0f;
    float lastConfidence_ = 0.0f;
    int lastActiveString_ = -1;
    int lastFret_ = -1;

    // Internal stages
    void updateStringActivity();
    void updatePitchDetection();
    void resolveAndOutput();
    void triggerNoteOn(const StringFretDecision& decision);
    void triggerNoteOff();
    void sendControlChanges();
    uint8_t envelopeToVelocity(float envelope) const;

    // Debug output (Teleplot)
    void sendDebugOutput();
};

} // namespace bassmint
