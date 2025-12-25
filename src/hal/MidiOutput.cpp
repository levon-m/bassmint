#include "MidiOutput.h"
#include "BoardConfig.h"
#include <Arduino.h>

namespace bassmint {

void MidiOutput::init() {
    // Initialize hardware MIDI on Serial2
    Serial2.begin(BoardConfig::MidiBaudRate);
}

void MidiOutput::sendNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
    if (channel < 1 || channel > 16) return;
    note = clampMidi(note);
    velocity = clampMidi(velocity);

    uint8_t status = 0x90 | ((channel - 1) & 0x0F);  // Note On + channel
    Serial2.write(status);
    Serial2.write(note);
    Serial2.write(velocity);
}

void MidiOutput::sendNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
    if (channel < 1 || channel > 16) return;
    note = clampMidi(note);
    velocity = clampMidi(velocity);

    uint8_t status = 0x80 | ((channel - 1) & 0x0F);  // Note Off + channel
    Serial2.write(status);
    Serial2.write(note);
    Serial2.write(velocity);
}

void MidiOutput::sendControlChange(uint8_t channel, uint8_t cc, uint8_t value) {
    if (channel < 1 || channel > 16) return;
    cc = clampMidi(cc);
    value = clampMidi(value);

    uint8_t status = 0xB0 | ((channel - 1) & 0x0F);  // Control Change + channel
    Serial2.write(status);
    Serial2.write(cc);
    Serial2.write(value);
}

void MidiOutput::sendPitchBend(uint8_t channel, uint16_t value) {
    if (channel < 1 || channel > 16) return;
    if (value > 16383) value = 16383;

    uint8_t status = 0xE0 | ((channel - 1) & 0x0F);  // Pitch Bend + channel
    uint8_t lsb = value & 0x7F;         // Lower 7 bits
    uint8_t msb = (value >> 7) & 0x7F;  // Upper 7 bits
    Serial2.write(status);
    Serial2.write(lsb);
    Serial2.write(msb);
}

void MidiOutput::flush() {
    // Hardware MIDI sends immediately, no flush needed
}

void MidiOutput::sendStringNoteOn(uint8_t stringIndex, uint8_t note, uint8_t velocity) {
    sendNoteOn(stringToChannel(stringIndex), note, velocity);
}

void MidiOutput::sendStringNoteOff(uint8_t stringIndex, uint8_t note, uint8_t velocity) {
    sendNoteOff(stringToChannel(stringIndex), note, velocity);
}

void MidiOutput::sendStringEnvelope(uint8_t stringIndex, float envelope) {
    sendControlChange(stringToChannel(stringIndex), MidiCC::Envelope, floatToMidi(envelope));
}

void MidiOutput::sendStringState(uint8_t stringIndex, bool active) {
    sendControlChange(stringToChannel(stringIndex), MidiCC::State,
                      active ? MidiState::Active : MidiState::Idle);
}

void MidiOutput::sendStringConfidence(uint8_t stringIndex, float confidence) {
    sendControlChange(stringToChannel(stringIndex), MidiCC::Confidence, floatToMidi(confidence));
}

uint8_t MidiOutput::stringToChannel(uint8_t stringIndex) {
    return (stringIndex < 4) ? (stringIndex + 1) : 1;
}

uint8_t MidiOutput::clampMidi(int value) {
    if (value < 0) return 0;
    if (value > 127) return 127;
    return static_cast<uint8_t>(value);
}

uint8_t MidiOutput::floatToMidi(float value) {
    if (value <= 0.0f) return 0;
    if (value >= 1.0f) return 127;
    return static_cast<uint8_t>(value * 127.0f);
}

} // namespace bassmint
