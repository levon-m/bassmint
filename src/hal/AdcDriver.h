#pragma once

#include "BoardConfig.h"
#include <cstddef>
#include <cstdint>

namespace bassmint {

/**
 * Hardware abstraction for ADC reading on Teensy 4.0
 *
 * Reads from:
 * - Bridge piezo (primary pitch sensor)
 * - 4x OPT101 sensors (string activity detection)
 *
 * Pin assignments are configured in BoardConfig.h
 */
class AdcDriver {
public:
    static constexpr size_t NumStrings = BoardConfig::NumStrings;

    /**
     * Initialize ADC hardware
     */
    void init();

    /**
     * Read sample from piezo sensor (normalized -1 to +1)
     * The piezo signal is AC-coupled, centered around 0
     */
    float readPiezoSample();

    /**
     * Read sample from a string's OPT101 sensor (normalized 0 to 1)
     * @param stringIndex String index (0-3: E, A, D, G)
     */
    float readStringSample(size_t stringIndex);

    /**
     * Read raw ADC value from piezo (0 to 4095)
     */
    uint16_t readPiezoRaw();

    /**
     * Read raw ADC value from string sensor (0 to 4095)
     */
    uint16_t readStringRaw(size_t stringIndex);

    /**
     * Set averaging for smoother reads
     * @param samples Number of samples to average (1-32)
     */
    void setAveraging(uint8_t samples);

private:
    uint8_t averaging_ = BoardConfig::AdcAveraging;

    /**
     * Perform ADC read on a pin
     */
    uint16_t readAdc(uint8_t pin);
};

} // namespace bassmint
