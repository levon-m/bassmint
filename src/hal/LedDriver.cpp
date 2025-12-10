<<<<<<< HEAD
#include "LedDriver.h"
#include <Arduino.h>
=======
#include "hal/LedDriver.h"
#include "hal/BoardConfig.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <cstdio>
>>>>>>> 491b711ec2cfccc73c60c377d0b2d86c73cefe06

namespace bassmint {

void LedDriver::init() {
<<<<<<< HEAD
    // Configure all IR LED pins as outputs
    for (size_t i = 0; i < NumLeds; ++i) {
        pinMode(BoardConfig::IrLedPins[i], OUTPUT);
        // Start with LEDs on (normal operation)
        turnOn(i);
=======
    if (initialized_) {
        return;
    }

    printf("LedDriver: Initializing IR LEDs...\n");

    // Initialize all LED pins as outputs
    for (uint8_t i = 0; i < NUM_STRINGS; ++i) {
        printf("  String %d: GPIO%d -> HIGH\n", i, BoardConfig::LED_PINS[i]);
        gpio_init(BoardConfig::LED_PINS[i]);
        gpio_set_dir(BoardConfig::LED_PINS[i], GPIO_OUT);
        gpio_put(BoardConfig::LED_PINS[i], 1); // Default to ON for testing
    }

    // TODO: Initialize PWM for brightness control
    // For now, simple digital on/off

    initialized_ = true;
    printf("LedDriver: Init complete\n");
}

void LedDriver::setLedOn(StringId string) {
    if (!initialized_) {
        return;
    }

    uint8_t index = static_cast<uint8_t>(string);
    if (index < NUM_STRINGS) {
        gpio_put(BoardConfig::LED_PINS[index], 1);
>>>>>>> 491b711ec2cfccc73c60c377d0b2d86c73cefe06
    }
}

void LedDriver::turnOn(size_t index) {
    if (index >= NumLeds) return;

    digitalWrite(BoardConfig::IrLedPins[index], BoardConfig::IrLedActiveState);
    ledStates_[index] = true;
}

void LedDriver::turnOff(size_t index) {
    if (index >= NumLeds) return;

    uint8_t offState = (BoardConfig::IrLedActiveState == HIGH) ? LOW : HIGH;
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
    digitalWrite(BoardConfig::StatusLedPin, on ? HIGH : LOW);
    statusLedState_ = on;
}

void LedDriver::toggleStatusLed() {
    setStatusLed(!statusLedState_);
}

} // namespace bassmint
