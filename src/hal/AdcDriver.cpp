#include "AdcDriver.h"
#include <Arduino.h>

namespace bassmint {

void AdcDriver::init() {
    // Configure ADC resolution
    analogReadResolution(BoardConfig::AdcResolution);

    // Set averaging for smoother reads
    analogReadAveraging(averaging_);
}

uint16_t AdcDriver::readAdc(uint8_t pin) {
    return analogRead(pin);
}

float AdcDriver::readPiezoSample() {
    uint16_t raw = readPiezoRaw();

    // Convert to -1 to +1 range
    // Piezo is AC-coupled, so signal oscillates around mid-scale
    constexpr float midScale = BoardConfig::AdcMaxValue / 2.0f;
    float normalized = (static_cast<float>(raw) - midScale) / midScale;
    return normalized;
}

float AdcDriver::readStringSample(size_t stringIndex) {
    uint16_t raw = readStringRaw(stringIndex);

    // OPT101 output is DC (0 to VRef based on light intensity)
    float normalized = static_cast<float>(raw) / static_cast<float>(BoardConfig::AdcMaxValue);
    return normalized;
}

uint16_t AdcDriver::readPiezoRaw() {
    return readAdc(BoardConfig::PiezoPin);
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
