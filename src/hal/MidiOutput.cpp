#include "MidiOutput.h"
#include <Arduino.h>

// Teensy USB MIDI object (defined in Teensy cores when USB_MIDI is enabled)
#if defined(USB_MIDI) || defined(USB_MIDI_SERIAL)
#include <usb_midi.h>
#endif

namespace bassmint {

void MidiOutput::init() {
    // USB MIDI is initialized by Teensy startup code when USB_MIDI is defined
}

void MidiOutput::sendNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
    if (channel < 1 || channel > 16) return;
    note = clampMidi(note);
    velocity = clampMidi(velocity);

    usbMIDI.sendNoteOn(note, velocity, channel);
}

void MidiOutput::sendNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
    if (channel < 1 || channel > 16) return;
    note = clampMidi(note);
    velocity = clampMidi(velocity);

    usbMIDI.sendNoteOff(note, velocity, channel);
}

void MidiOutput::sendControlChange(uint8_t channel, uint8_t cc, uint8_t value) {
    if (channel < 1 || channel > 16) return;
    cc = clampMidi(cc);
    value = clampMidi(value);

    usbMIDI.sendControlChange(cc, value, channel);
}

void MidiOutput::sendPitchBend(uint8_t channel, uint16_t value) {
    if (channel < 1 || channel > 16) return;
    if (value > 16383) value = 16383;

    // Teensy expects -8192 to 8191, convert from 0-16383
    int bend = static_cast<int>(value) - 8192;
    usbMIDI.sendPitchBend(bend, channel);
}

void MidiOutput::flush() {
    usbMIDI.send_now();
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
