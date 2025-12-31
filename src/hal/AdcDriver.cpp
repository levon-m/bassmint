#include "AdcDriver.h"
#include <Arduino.h>

namespace bassmint {

void AdcDriver::init() {
    // Configure ADC resolution
    analogReadResolution(BoardConfig::AdcResolution);

    // Set averaging for smoother reads
    analogReadAveraging(averaging_);

    // Disable digital input keeper on all analog pins to prevent loading
    // high-impedance sources (OPT101 outputs).
    // The Teensy 4.0 GPIO keeper acts like ~100k pull and creates a resistor
    // divider that drags the ADC node away from the intended bias voltage.
    for (size_t i = 0; i < NumStrings; ++i) {
        pinMode(BoardConfig::Opt101Pins[i], INPUT_DISABLE);
    }
}

uint16_t AdcDriver::readAdc(uint8_t pin) {
    return analogRead(pin);
}

float AdcDriver::readStringSample(size_t stringIndex) {
    uint16_t raw = readStringRaw(stringIndex);

    // OPT101 output is DC (0 to VRef based on light intensity)
    float normalized = static_cast<float>(raw) / static_cast<float>(BoardConfig::AdcMaxValue);
    return normalized;
}

uint16_t AdcDriver::readStringRaw(size_t stringIndex) {
    if (stringIndex >= NumStrings) {
        return 0;
    }
    return readAdc(BoardConfig::Opt101Pins[stringIndex]);
}

void AdcDriver::setAveraging(uint8_t samples) {
    averaging_ = (samples > 32) ? 32 : ((samples < 1) ? 1 : samples);
    analogReadAveraging(averaging_);
}

} // namespace bassmint
