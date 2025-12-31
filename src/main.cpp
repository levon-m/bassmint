/**
 * BassMINT - Bass Guitar MIDI Interface using Teensy 4.0
 *
 * Hardware:
 * - Teensy 4.0 (IMXRT1062, 600MHz ARM Cortex-M7)
 * - 4x OPT101 + IR LED break-beam sensors for string activity and pitch detection
 * - MIDI DIN output via Serial2
 *
 * Pin assignments are configured in hal/BoardConfig.h
 */

#include <Arduino.h>
#include "app/App.h"
#include "hal/BoardConfig.h"

using namespace bassmint;

// Cycles per sample for 44.1kHz timing
// At 600MHz: 600,000,000 / 44,100 = 13,605 cycles per sample
static constexpr uint32_t CYCLES_PER_SAMPLE =
    F_CPU / static_cast<uint32_t>(BoardConfig::SampleRate);

// Global application instance
static App* gApp = nullptr;

void setup() {
    // Enable ARM cycle counter for precise sample-rate timing
    // These are standard ARM Cortex-M debug registers
    ARM_DEMCR |= ARM_DEMCR_TRCENA;      // Enable trace
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;  // Enable cycle counter

    // Initialize Serial for Teleplot debug output
    if constexpr (BoardConfig::DebugEnabled) {
        Serial.begin(BoardConfig::DebugBaudRate);
        Serial.println("BassMINT starting...");
    }

    // Create and initialize application
    static App application;
    gApp = &application;
    gApp->init();

    if constexpr (BoardConfig::DebugEnabled) {
        Serial.println("BassMINT ready");
    }
}

void loop() {
    if (gApp == nullptr) return;

    // Wait until enough cycles have passed for next sample
    static uint32_t lastCycle = 0;
    uint32_t now = ARM_DWT_CYCCNT;

    if ((now - lastCycle) >= CYCLES_PER_SAMPLE) {
        lastCycle = now;
        gApp->update();
    }
}
