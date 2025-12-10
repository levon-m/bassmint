#include "NoteTables.h"
#include <cmath>

namespace bassmint {

// Static member definitions
std::array<std::array<float, NoteTables::NumFrets>, NoteTables::NumStrings> NoteTables::frequencyTable_;
std::array<std::array<uint8_t, NoteTables::NumFrets>, NoteTables::NumStrings> NoteTables::midiNoteTable_;
bool NoteTables::initialized_ = false;

void NoteTables::init() {
    if (initialized_) return;

    // Build frequency and MIDI note tables for all strings and frets
    for (size_t s = 0; s < NumStrings; ++s) {
        float openHz = Tuning::OpenStringHz[s];
        uint8_t openMidi = Tuning::OpenStringMidiNote[s];

        for (int f = 0; f <= MaxFret; ++f) {
            // Equal temperament: f = f0 * 2^(n/12)
            frequencyTable_[s][f] = openHz * std::pow(2.0f, static_cast<float>(f) / 12.0f);
            midiNoteTable_[s][f] = openMidi + static_cast<uint8_t>(f);
        }
    }

    initialized_ = true;
}

float NoteTables::getFrequency(size_t stringIndex, int fret) {
    if (stringIndex >= NumStrings || fret < 0 || fret > MaxFret) {
        return 0.0f;
    }
    return frequencyTable_[stringIndex][fret];
}

uint8_t NoteTables::getMidiNote(size_t stringIndex, int fret) {
    if (stringIndex >= NumStrings || fret < 0 || fret > MaxFret) {
        return 0;
    }
    return midiNoteTable_[stringIndex][fret];
}

float NoteTables::getMinFrequency(size_t stringIndex) {
    if (stringIndex >= NumStrings) return 0.0f;
    return frequencyTable_[stringIndex][0];
}

float NoteTables::getMaxFrequency(size_t stringIndex) {
    if (stringIndex >= NumStrings) return 0.0f;
    return frequencyTable_[stringIndex][MaxFret];
}

bool NoteTables::isInitialized() {
    return initialized_;
}

} // namespace bassmint
