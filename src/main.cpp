/**
 * BassMINT - Bass Guitar MIDI Interface using Teensy 4.0
 *
 * Hardware:
 * - Teensy 4.0 (IMXRT1062, 600MHz ARM Cortex-M7)
 * - Bridge-mounted piezo for pitch detection
 * - 4x OPT101 + IR LED break-beam sensors for string activity
 * - USB MIDI output
 *
 * Pin assignments are configured in hal/BoardConfig.h
 */

#include <Arduino.h>
#include "app/App.h"
<<<<<<< HEAD
#include "hal/BoardConfig.h"

using namespace bassmint;
=======
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <cstdio>

// XIAO RP2040 onboard LEDs (accent-inverted: LOW = on, HIGH = off)
#define LED_BLUE 25
#define LED_RED 17
#define LED_GREEN 16

using namespace BassMINT;
>>>>>>> 491b711ec2cfccc73c60c377d0b2d86c73cefe06

// Cycles per sample for 44.1kHz timing
// At 600MHz: 600,000,000 / 44,100 = 13,605 cycles per sample
static constexpr uint32_t CYCLES_PER_SAMPLE =
    F_CPU / static_cast<uint32_t>(BoardConfig::SampleRate);

<<<<<<< HEAD
// Global application instance
static App* gApp = nullptr;
=======
    // Test onboard LED to verify GPIO works
    gpio_init(LED_BLUE);
    gpio_set_dir(LED_BLUE, GPIO_OUT);

    // Blink blue LED 3 times at startup
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_BLUE, 0);  // ON (active low)
        sleep_ms(200);
        gpio_put(LED_BLUE, 1);  // OFF
        sleep_ms(200);
    }

    // Small delay to allow USB serial to connect
    sleep_ms(500);
>>>>>>> 491b711ec2cfccc73c60c377d0b2d86c73cefe06

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
