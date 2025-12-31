#include "LedDriver.h"
#include <Arduino.h>

namespace bassmint {

void LedDriver::init() {
    // Configure all IR LED pins as outputs
    for (size_t i = 0; i < NumLeds; ++i) {
        pinMode(BoardConfig::IrLedPins[i], OUTPUT);
        // Start with LEDs on (normal operation)
        turnOn(i);
    }
}

void LedDriver::turnOn(size_t index) {
    if (index >= NumLeds) return;

    digitalWrite(BoardConfig::IrLedPins[index], BoardConfig::IrLedActiveState);
    ledStates_[index] = true;
}

void LedDriver::turnOff(size_t index) {
    if (index >= NumLeds) return;

    uint8_t offState = (BoardConfig::IrLedActiveState == 1) ? 0 : 1;
    digitalWrite(BoardConfig::IrLedPins[index], offState);
    ledStates_[index] = false;
}

void LedDriver::allOn() {
    for (size_t i = 0; i < NumLeds; ++i) {
        turnOn(i);
    }
}

void LedDriver::allOff() {
    for (size_t i = 0; i < NumLeds; ++i) {
        turnOff(i);
    }
}

void LedDriver::set(size_t index, bool on) {
    if (on) {
        turnOn(index);
    } else {
        turnOff(index);
    }
}

void LedDriver::toggle(size_t index) {
    if (index >= NumLeds) return;

    set(index, !ledStates_[index]);
}

bool LedDriver::isOn(size_t index) const {
    if (index >= NumLeds) return false;
    return ledStates_[index];
}

void LedDriver::initStatusLed() {
    pinMode(BoardConfig::StatusLedPin, OUTPUT);
    setStatusLed(false);
}

void LedDriver::setStatusLed(bool on) {
    digitalWrite(BoardConfig::StatusLedPin, on ? 1 : 0);
    statusLedState_ = on;
}

void LedDriver::toggleStatusLed() {
    setStatusLed(!statusLedState_);
}

} // namespace bassmint
