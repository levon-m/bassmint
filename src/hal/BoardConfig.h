#pragma once

#include <cstdint>
#include <cstddef>

namespace bassmint {

/**
 * Centralized hardware pin configuration for BassMINT
 *
 * Edit this file to match your wiring setup.
 * All pin assignments are Teensy 4.0 pin numbers.
 */
namespace BoardConfig {

    // Number of strings
    static constexpr size_t NumStrings = 4;

    // =========================================================================
    // OPT101 SENSORS (String Detection + Pitch Detection)
    // =========================================================================
    // One OPT101 photodiode per string, analog outputs
    // Used for both string activity detection and pitch detection
    // String order: E(0), A(1), D(2), G(3)
    static constexpr uint8_t Opt101Pins[NumStrings] = {
        22,  // A8 - String 0 (E)
        21,  // A7 - String 1 (A)
        20,  // A6 - String 2 (D)
        19   // A5 - String 3 (G)
    };

    // =========================================================================
    // IR LED CONTROL (Paired with OPT101 sensors)
    // =========================================================================
    // Digital outputs to control IR LEDs for break-beam sensing
    // Each IR LED pairs with corresponding OPT101 sensor
    static constexpr uint8_t IrLedPins[NumStrings] = {
        2,   // String 0 (E) IR LED
        3,   // String 1 (A) IR LED
        4,   // String 2 (D) IR LED
        5    // String 3 (G) IR LED
    };

    // IR LED active state (1 = on, 0 = on for active-low drivers)
    static constexpr uint8_t IrLedActiveState = 1;

    // =========================================================================
    // MIDI OUTPUT (via Adafruit MIDI FeatherWing)
    // =========================================================================
    // Hardware MIDI DIN output via Serial2
    static constexpr uint8_t MidiTxPin = 8;   // Serial2 TX (TX2)
    static constexpr uint8_t MidiRxPin = 7;   // Serial2 RX (RX2)
    static constexpr uint32_t MidiBaudRate = 31250;     // Standard MIDI baud rate

    // =========================================================================
    // STATUS LED (Optional)
    // =========================================================================
    // Built-in LED for status indication
    static constexpr uint8_t StatusLedPin = 13;  // Teensy built-in LED

    // =========================================================================
    // ADC CONFIGURATION
    // =========================================================================
    static constexpr uint8_t AdcResolution = 12;          // 12-bit ADC
    static constexpr uint16_t AdcMaxValue = (1 << 12) - 1; // 4095
    static constexpr uint8_t AdcAveraging = 4;            // Hardware averaging samples
    static constexpr float VRef = 3.3f;                   // Reference voltage

    // =========================================================================
    // TIMING CONFIGURATION
    // =========================================================================
    static constexpr float SampleRate = 44100.0f;         // Audio sample rate (Hz)
    static constexpr size_t PitchFrameSize = 2048;        // Pitch detection frame size

    // =========================================================================
    // DEBUG CONFIGURATION
    // =========================================================================
    static constexpr bool DebugEnabled = true;
    static constexpr uint32_t DebugBaudRate = 115200;
    static constexpr uint32_t DebugOutputHz = 60;         // How often to send Teleplot data

    // Debug mode: true = Teleplot graphs, false = text messages (active string changes)
    static constexpr bool TeleplotMode = false;

} // namespace BoardConfig
} // namespace bassmint
