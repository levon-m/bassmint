#pragma once

#include "BoardConfig.h"
#include <cstdint>
#include <cstddef>

namespace bassmint {

/**
 * Driver for IR LEDs used in break-beam string detection
 *
 * Controls the IR LEDs that pair with OPT101 sensors.
 * Each string has one IR LED + OPT101 pair for activity detection.
 */
class LedDriver {
public:
    static constexpr size_t NumLeds = BoardConfig::NumStrings;

    /**
     * Initialize all IR LED pins as outputs
     */
    void init();

    /**
     * Turn on a specific IR LED
     * @param index LED index (0-3, corresponding to string)
     */
    void turnOn(size_t index);

    /**
     * Turn off a specific IR LED
     * @param index LED index (0-3)
     */
    void turnOff(size_t index);

    /**
     * Turn on all IR LEDs
     */
    void allOn();

    /**
     * Turn off all IR LEDs
     */
    void allOff();

    /**
     * Set LED state
     * @param index LED index (0-3)
     * @param on True to turn on, false to turn off
     */
    void set(size_t index, bool on);

    /**
     * Toggle LED state
     * @param index LED index (0-3)
     */
    void toggle(size_t index);

    /**
     * Check if LED is currently on
     * @param index LED index (0-3)
     * @return True if LED is on
     */
    bool isOn(size_t index) const;

    // Status LED control (built-in Teensy LED)

    /**
     * Initialize status LED
     */
    void initStatusLed();

    /**
     * Set status LED state
     */
    void setStatusLed(bool on);

    /**
     * Toggle status LED
     */
    void toggleStatusLed();

private:
    bool ledStates_[NumLeds] = {false};
    bool statusLedState_ = false;
};

} // namespace bassmint
